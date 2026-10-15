"""
蓝队第三阶段控制器：前沿探索 (Frontier Exploration) + 航位推算 (Dead Reckoning) + 抗传感器干扰
10 架无人机全并发全图探索与防打转版
文件路径：src/swarm_rescue/solutions/my_drone_phase3.py
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
from swarm_rescue.simulation.utils.utils import normalize_angle

WorldPoint = Tuple[float, float]

# 10 架无人机专属全图巡航扇区
DRONE_PATROL_SECTORS: List[List[WorldPoint]] = [
    [(-400.0, 240.0), (-350.0, 60.0), (-520.0, 160.0)],
    [(-350.0, -180.0), (-250.0, -280.0), (-450.0, -250.0)],
    [(50.0, -280.0), (180.0, -250.0), (100.0, -150.0)],
    [(0.0, -60.0), (150.0, 0.0), (50.0, 90.0)],
    [(0.0, 200.0), (150.0, 260.0), (200.0, 180.0)],
    [(180.0, 100.0), (80.0, 180.0), (160.0, 250.0)],
    [(400.0, 260.0), (530.0, 240.0), (480.0, 100.0)],
    [(450.0, 0.0), (550.0, -60.0), (400.0, -110.0)],
    [(420.0, -250.0), (540.0, -280.0), (480.0, -150.0)],
    [(520.0, 120.0), (500.0, -200.0), (380.0, 0.0)],
]


def resolve_topological_waypoint(current: WorldPoint, destination: WorldPoint) -> WorldPoint:
    D1_W: WorldPoint = (-220.0, -220.0)
    D1_E: WorldPoint = (0.0, -220.0)
    D2_W: WorldPoint = (170.0, 260.0)
    D2_E: WorldPoint = (390.0, 260.0)

    in_west = current[0] < -115.0
    in_center = -115.0 <= current[0] <= 285.0
    in_east = current[0] > 285.0

    dest_west = destination[0] < -115.0
    dest_center = -115.0 <= destination[0] <= 285.0
    dest_east = destination[0] > 285.0

    if in_west:
        if dest_center or dest_east:
            if current[1] > -120.0:
                return D1_W
            return D1_E

    if in_center:
        if dest_east:
            if current[1] < 180.0:
                return D2_W
            return D2_E
        if dest_west:
            if current[1] > -120.0:
                return D1_E
            return D1_W

    if in_east:
        if dest_center or dest_west:
            if current[1] < 180.0:
                return D2_E
            return D2_W

    return destination


class MyDronePhase3(DroneAbstract):
    """
    蓝队阶段三：前沿探索、航位推算与抗传感器干扰控制器
    """

    def __init__(
        self,
        identifier: Optional[int] = None,
        misc_data: Optional[Any] = None,
        **kwargs,
    ):
        super().__init__(identifier=identifier, misc_data=misc_data, **kwargs)
        self.drone_id: int = int(identifier or 0)

        self.sector_waypoint_index: int = 0
        self.waypoint_ticks: int = 0

        self.est_position: np.ndarray = np.array([-505.0, 98.0])
        self.home_pos: Optional[WorldPoint] = (-505.0, 98.0)
        self.known_dc_pos: Optional[WorldPoint] = (-505.0, 302.0)

        self.known_bombs: Dict[int, Dict[str, Any]] = {}
        self.disposed_bomb_ids: set[int] = set()
        self.my_target_bomb_key: Optional[int] = None

        self.carry_push_steps: int = 0
        self.carry_spin_steps: int = 0
        self.carry_spin_sign: float = 1.0 if self.drone_id % 2 == 0 else -1.0

        self.stuck_counter: int = 0
        self.escape_steps: int = 0
        self.escape_direction: float = 1.0 if self.drone_id % 2 == 0 else -1.0
        self.last_pos_for_stuck: Optional[np.ndarray] = None
        self.prev_heading_error: float = 0.0

    def define_message_for_all(self) -> Any:
        pos = self._get_best_pos()
        curr_pos = (float(pos[0]), float(pos[1])) if pos is not None else None

        compact_bombs = []
        for b_key, b_info in self.known_bombs.items():
            if not b_info.get("disposed", False):
                compact_bombs.append((
                    b_key,
                    b_info["pos"][0],
                    b_info["pos"][1],
                    b_info.get("claimed_by"),
                    b_info.get("claim_cost", 9999.0),
                    b_info.get("grasped", False),
                ))

        return {
            "id": self.drone_id,
            "pos": curr_pos,
            "carrying": len(self.grasped_bombs()) > 0,
            "target": self.my_target_bomb_key,
            "dc_pos": self.known_dc_pos,
            "bombs": compact_bombs,
            "disposed": list(self.disposed_bomb_ids),
        }

    def control(self) -> CommandsDict:
        cmd: CommandsDict = {
            "forward": 0.0,
            "lateral": 0.0,
            "rotation": 0.0,
            "grasper": 0,
        }

        self._update_dead_reckoning()
        self._process_mesh_communications()
        self._update_local_perception()

        if self._handle_escape_logic(cmd):
            return cmd

        if len(self.grasped_bombs()) > 0:
            cmd["grasper"] = 1
            return self._control_carry_and_release(cmd)

        self._evaluate_auction_and_claim()

        if self.my_target_bomb_key is not None and self.my_target_bomb_key in self.known_bombs:
            target_info = self.known_bombs[self.my_target_bomb_key]
            if not target_info.get("disposed", False) and not target_info.get("grasped", False):
                return self._control_approach_claimed_bomb(cmd, target_info["pos"])
            else:
                self.my_target_bomb_key = None

        visible_bomb = self._find_immediate_free_bomb()
        if visible_bomb is not None:
            return self._control_precise_grasp(cmd, visible_bomb)

        return self._control_explore_assigned_sector(cmd)

    def _update_dead_reckoning(self) -> None:
        gps_pos = self.measured_gps_position()
        if gps_pos is not None:
            self.est_position = np.array([float(gps_pos[0]), float(gps_pos[1])])
            if self.home_pos is None:
                self.home_pos = (float(gps_pos[0]), float(gps_pos[1]))
        else:
            vel = self.measured_velocity()
            compass = self.measured_compass_angle()
            odom = self.odometer_values()
            dt = 0.1
            if vel is not None:
                self.est_position += vel * dt
            elif odom is not None and compass is not None:
                speed = float(odom[0])
                alpha = float(odom[1])
                vx = speed * math.cos(compass + alpha)
                vy = speed * math.sin(compass + alpha)
                self.est_position += np.array([vx, vy]) * dt

    def _get_best_pos(self) -> np.ndarray:
        return self.est_position

    def _control_explore_assigned_sector(self, cmd: CommandsDict) -> CommandsDict:
        self.waypoint_ticks += 1
        pos = self._get_best_pos()
        curr_pt: WorldPoint = (float(pos[0]), float(pos[1]))

        my_sector = DRONE_PATROL_SECTORS[self.drone_id % len(DRONE_PATROL_SECTORS)]
        target_pt = my_sector[self.sector_waypoint_index % len(my_sector)]

        dist_to_target = math.hypot(target_pt[0] - pos[0], target_pt[1] - pos[1])
        if dist_to_target < 45.0 or self.waypoint_ticks > 180:
            self.waypoint_ticks = 0
            self.sector_waypoint_index = (self.sector_waypoint_index + 1) % len(my_sector)
            target_pt = my_sector[self.sector_waypoint_index % len(my_sector)]

        intermediate_goal = resolve_topological_waypoint(curr_pt, target_pt)

        peer_lat_push = 0.0
        compass = self.measured_compass_angle() or 0.0
        if not self.communicator_is_disabled():
            received = getattr(self.communicator, "received_messages", [])
            for item in received:
                if isinstance(item, tuple) and len(item) >= 2:
                    _, msg = item
                    if isinstance(msg, dict) and msg.get("pos") is not None:
                        nx, ny = msg["pos"]
                        pdx = pos[0] - nx
                        pdy = pos[1] - ny
                        d = float(math.hypot(pdx, pdy))
                        if 10.0 < d < 65.0:
                            lat_proj = -math.sin(compass) * pdx + math.cos(compass) * pdy
                            rep_f = min(0.35, 120.0 / (d * d))
                            peer_lat_push += (-1.0 if lat_proj > 0 else 1.0) * rep_f

        cmd["lateral"] = max(-0.4, min(0.4, peer_lat_push))
        return self._navigate_towards(cmd, intermediate_goal, max_fwd=0.82)

    def _process_mesh_communications(self) -> None:
        if self.communicator_is_disabled():
            return

        received = getattr(self.communicator, "received_messages", [])
        if not received:
            return

        my_pos = self._get_best_pos()

        for item in received:
            if not isinstance(item, tuple) or len(item) < 2:
                continue
            _, msg = item
            if not isinstance(msg, dict):
                continue

            sender_id = msg.get("id")
            if sender_id == self.drone_id:
                continue

            if msg.get("dc_pos") is not None:
                self.known_dc_pos = tuple(msg["dc_pos"])

            for disp_id in msg.get("disposed", []):
                self.disposed_bomb_ids.add(disp_id)
                if disp_id in self.known_bombs:
                    self.known_bombs[disp_id]["disposed"] = True
                    self.known_bombs[disp_id]["grasped"] = False
                if self.my_target_bomb_key == disp_id:
                    self.my_target_bomb_key = None

            for entry in msg.get("bombs", []):
                b_key = entry[0]
                bx, by = entry[1], entry[2]
                claimed_by = entry[3] if len(entry) > 3 else None
                claim_cost = entry[4] if len(entry) > 4 else 9999.0
                is_grasped = entry[5] if len(entry) > 5 else False

                if b_key in self.disposed_bomb_ids:
                    continue

                if b_key not in self.known_bombs:
                    self.known_bombs[b_key] = {
                        "pos": (bx, by),
                        "claimed_by": claimed_by,
                        "claim_cost": claim_cost,
                        "grasped": is_grasped,
                        "disposed": False,
                    }
                else:
                    curr = self.known_bombs[b_key]
                    curr["pos"] = (bx, by)
                    if is_grasped:
                        curr["grasped"] = True
                        if self.my_target_bomb_key == b_key:
                            self.my_target_bomb_key = None

                    if claimed_by is not None:
                        curr_claimed = curr.get("claimed_by")
                        if curr_claimed is None or claim_cost < curr.get("claim_cost", 9999.0) - 10.0:
                            curr["claimed_by"] = claimed_by
                            curr["claim_cost"] = claim_cost

            neighbor_target = msg.get("target")
            if (
                neighbor_target is not None
                and neighbor_target == self.my_target_bomb_key
                and my_pos is not None
            ):
                target_pos = self.known_bombs[self.my_target_bomb_key]["pos"]
                my_dist = float(math.hypot(target_pos[0] - my_pos[0], target_pos[1] - my_pos[1]))
                neighbor_dist = float(msg.get("claim_cost", 9999.0))
                if neighbor_dist < my_dist - 15.0 or (abs(neighbor_dist - my_dist) <= 15.0 and sender_id < self.drone_id):
                    self.known_bombs[self.my_target_bomb_key]["claimed_by"] = sender_id
                    self.my_target_bomb_key = None

    def _update_local_perception(self) -> None:
        pos = self._get_best_pos()
        compass = self.measured_compass_angle() or 0.0
        semantics = self.semantic_values() or []

        for hit in semantics:
            if hit.entity_type == DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER:
                global_ang = normalize_angle(compass + float(hit.angle))
                dc_x = float(pos[0]) + float(hit.distance) * math.cos(global_ang)
                dc_y = float(pos[1]) + float(hit.distance) * math.sin(global_ang)
                self.known_dc_pos = (dc_x, dc_y)

            elif hit.entity_type == DroneSemanticSensor.TypeEntity.BOMB:
                global_ang = normalize_angle(compass + float(hit.angle))
                bx = float(pos[0]) + float(hit.distance) * math.cos(global_ang)
                by = float(pos[1]) + float(hit.distance) * math.sin(global_ang)

                matched_key = None
                for b_key, b_info in self.known_bombs.items():
                    if math.hypot(b_info["pos"][0] - bx, b_info["pos"][1] - by) < 35.0:
                        matched_key = b_key
                        break

                if matched_key is None:
                    new_key = len(self.known_bombs) + 1
                    self.known_bombs[new_key] = {
                        "pos": (bx, by),
                        "claimed_by": None,
                        "claim_cost": 9999.0,
                        "grasped": hit.grasped,
                        "disposed": False,
                    }
                else:
                    self.known_bombs[matched_key]["pos"] = (bx, by)
                    self.known_bombs[matched_key]["grasped"] = hit.grasped
                    if hit.grasped and self.my_target_bomb_key == matched_key and len(self.grasped_bombs()) == 0:
                        self.my_target_bomb_key = None

    def _evaluate_auction_and_claim(self) -> None:
        if len(self.grasped_bombs()) > 0:
            return

        pos = self._get_best_pos()

        if self.my_target_bomb_key is not None:
            curr_target = self.known_bombs.get(self.my_target_bomb_key)
            if (
                curr_target is None
                or curr_target.get("disposed", False)
                or curr_target.get("grasped", False)
                or (curr_target.get("claimed_by") is not None and curr_target["claimed_by"] != self.drone_id)
            ):
                self.my_target_bomb_key = None
            else:
                t_pos = curr_target["pos"]
                my_cost = float(math.hypot(t_pos[0] - pos[0], t_pos[1] - pos[1]))
                curr_target["claimed_by"] = self.drone_id
                curr_target["claim_cost"] = my_cost
                return

        best_candidate: Optional[int] = None
        best_cost = 9999.0

        for b_key, b_info in self.known_bombs.items():
            if b_info.get("disposed", False) or b_info.get("grasped", False):
                continue

            bx, by = b_info["pos"]
            dist_to_bomb = float(math.hypot(bx - pos[0], by - pos[1]))
            claimed_by = b_info.get("claimed_by")
            existing_cost = b_info.get("claim_cost", 9999.0)

            if claimed_by is None or dist_to_bomb < (existing_cost - 30.0):
                if dist_to_bomb < best_cost:
                    best_cost = dist_to_bomb
                    best_candidate = b_key

        if best_candidate is not None:
            self.my_target_bomb_key = best_candidate
            self.known_bombs[best_candidate]["claimed_by"] = self.drone_id
            self.known_bombs[best_candidate]["claim_cost"] = best_cost

    def _find_immediate_free_bomb(self) -> Optional[DroneSemanticSensor.Data]:
        semantics = self.semantic_values() or []
        for h in semantics:
            if h.entity_type == DroneSemanticSensor.TypeEntity.BOMB and not h.grasped:
                return h
        return None

    def _control_approach_claimed_bomb(
        self, cmd: CommandsDict, target_pos: WorldPoint
    ) -> CommandsDict:
        semantics = self.semantic_values() or []
        visible_bomb = None
        for h in semantics:
            if h.entity_type == DroneSemanticSensor.TypeEntity.BOMB:
                if not h.grasped:
                    visible_bomb = h
                    break
                else:
                    self.my_target_bomb_key = None
                    return self._control_explore_assigned_sector(cmd)

        if visible_bomb is not None and float(visible_bomb.distance) < 95.0:
            return self._control_precise_grasp(cmd, visible_bomb)

        pos = self._get_best_pos()
        curr_pt: WorldPoint = (float(pos[0]), float(pos[1]))
        dist_to_record = math.hypot(target_pos[0] - pos[0], target_pos[1] - pos[1])

        if dist_to_record < 45.0:
            if self.my_target_bomb_key is not None and self.my_target_bomb_key in self.known_bombs:
                self.known_bombs[self.my_target_bomb_key]["disposed"] = True
            self.my_target_bomb_key = None
            return self._control_explore_assigned_sector(cmd)

        transit_waypoint = resolve_topological_waypoint(curr_pt, target_pos)
        return self._navigate_towards(cmd, transit_waypoint, max_fwd=0.82)

    def _control_precise_grasp(
        self, cmd: CommandsDict, bomb: DroneSemanticSensor.Data
    ) -> CommandsDict:
        angle_err = float(bomb.angle)
        dist = float(bomb.distance)

        kp = 1.6
        kd = 0.35
        derivative = angle_err - self.prev_heading_error
        self.prev_heading_error = angle_err

        cmd["rotation"] = max(-1.0, min(1.0, angle_err * kp + derivative * kd))

        if abs(angle_err) > 0.35:
            cmd["forward"] = 0.22
        else:
            cmd["forward"] = 0.80 if dist > 60.0 else (0.45 if dist > 32.0 else 0.28)

        if dist <= 38.0 and abs(angle_err) < 0.45:
            cmd["grasper"] = 1

        return cmd

    def _control_carry_and_release(self, cmd: CommandsDict) -> CommandsDict:
        cmd["grasper"] = 1

        if self.my_target_bomb_key is not None and self.my_target_bomb_key in self.known_bombs:
            self.known_bombs[self.my_target_bomb_key]["claimed_by"] = self.drone_id
            self.known_bombs[self.my_target_bomb_key]["grasped"] = True

        dc_target = self.known_dc_pos or (-505.0, 302.0)
        pos = self._get_best_pos()
        curr_pt: WorldPoint = (float(pos[0]), float(pos[1]))
        dist_to_dc = math.hypot(dc_target[0] - pos[0], dc_target[1] - pos[1])

        if dist_to_dc < 48.0 or self.carry_push_steps > 0:
            if self.carry_spin_steps > 0:
                self.carry_spin_steps -= 1
                cmd["forward"] = 0.18
                cmd["rotation"] = 0.90 * self.carry_spin_sign
                if self.carry_spin_steps <= 2:
                    cmd["grasper"] = 0
                    if self.my_target_bomb_key is not None:
                        self.disposed_bomb_ids.add(self.my_target_bomb_key)
                        if self.my_target_bomb_key in self.known_bombs:
                            self.known_bombs[self.my_target_bomb_key]["disposed"] = True
                            self.known_bombs[self.my_target_bomb_key]["grasped"] = False
                        self.my_target_bomb_key = None
                return cmd

            self.carry_push_steps += 1
            cmd["forward"] = 0.65
            if self.carry_push_steps >= 14 or dist_to_dc < 22.0:
                self.carry_spin_steps = 12
                self.carry_push_steps = 0
            return cmd

        next_waypoint = resolve_topological_waypoint(curr_pt, dc_target)
        return self._navigate_towards(cmd, next_waypoint, max_fwd=0.82)

    def _navigate_towards(
        self,
        cmd: CommandsDict,
        target_point: WorldPoint,
        max_fwd: float = 0.82,
    ) -> CommandsDict:
        pos = self._get_best_pos()
        compass = self.measured_compass_angle() or 0.0

        dx = target_point[0] - pos[0]
        dy = target_point[1] - pos[1]
        dist = math.hypot(dx, dy)

        if dist < 25.0:
            cmd["forward"] = max_fwd * 0.45
            cmd["rotation"] = 0.0
            return cmd

        target_rel_ang = normalize_angle(math.atan2(dy, dx) - compass)

        lidar_dists = self.lidar_values()
        lidar_angles = self.lidar_rays_angles()

        best_ang = target_rel_ang
        min_cost = 99999.0
        front_d = 999.0

        if lidar_dists is not None and lidar_angles is not None:
            for d_val, a_val in zip(lidar_dists, lidar_angles):
                d = float(d_val)
                a = float(a_val)
                if abs(a) < 0.35 and d < front_d:
                    front_d = d

                diff = abs(normalize_angle(a - target_rel_ang))
                cost = diff
                if d < 28.0:
                    cost += 45.0 / (d + 1.0)
                elif d < 45.0:
                    cost += 5.0 / (d + 1.0)

                if cost < min_cost:
                    min_cost = cost
                    best_ang = a

        rot_scale = max(0.25, dist / 50.0) if dist < 50.0 else 1.0
        cmd["rotation"] = max(-1.0, min(1.0, best_ang * 1.5 * rot_scale))

        if front_d < 18.0:
            cmd["forward"] = -0.30
            cmd["rotation"] = 0.80 if best_ang >= 0 else -0.80
        else:
            fwd_factor = max(0.40, math.cos(best_ang))
            cmd["forward"] = max_fwd * fwd_factor

        return cmd

    def _handle_escape_logic(self, cmd: CommandsDict) -> bool:
        pos = self._get_best_pos()

        if self.escape_steps > 0:
            self.escape_steps -= 1
            cmd["forward"] = -0.40
            cmd["rotation"] = 0.85 * self.escape_direction
            return True

        if self.last_pos_for_stuck is not None:
            dist_moved = float(np.linalg.norm(pos - self.last_pos_for_stuck))
            if dist_moved < 0.8:
                self.stuck_counter += 1
                if self.stuck_counter > 20:
                    self.escape_steps = 15
                    self.escape_direction = 1.0 if self.drone_id % 2 == 0 else -1.0
                    self.sector_waypoint_index += 1
                    self.waypoint_ticks = 0
                    self.stuck_counter = 0
                    cmd["forward"] = -0.40
                    cmd["rotation"] = 0.85 * self.escape_direction
                    return True
            else:
                self.stuck_counter = max(0, self.stuck_counter - 1)

        self.last_pos_for_stuck = pos.copy()
        return False
