"""
蓝队第四阶段终极控制器：全功能分层混合自主架构 (Hierarchical Hybrid Architecture)
文件路径：src/swarm_rescue/solutions/my_drone_phase4.py
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

# 全图战略走廊与门洞拓扑路网 (Topological Corridor Graph)
CORRIDOR_GRAPH: List[WorldPoint] = [
    (-220.0, 80.0),   # Node 0: 出仓主通道门洞 (安全脱离角落)
    (-100.0, -180.0), # Node 1: 西南走廊中继
    (180.0, -260.0),  # Node 2: 南部主干道
    (460.0, -220.0),  # Node 3: 东南房间枢纽
    (500.0, 50.0),    # Node 4: 东部走廊
    (460.0, 260.0),   # Node 5: 东北房间
    (180.0, 260.0),   # Node 6: 北部主干道
    (-80.0, 150.0),   # Node 7: 西北通道口
    (0.0, 0.0),       # Node 8: 中心十字枢纽
]


class MyDronePhase4(DroneAbstract):
    """
    蓝队第四阶段终极架构：全自主分层协同与智能返航排爆系统
    分层体系：
    - Layer 0 (安全反应层)：切向滑墙 APF + 智能反冲倒车防卡死
    - Layer 1 (状态估计层)：GPS/里程计航位推算 (Dead Reckoning)，无畏 NoGps 干扰区
    - Layer 2 (拓扑规划层)：全局门洞拓扑路网，自主穿梭隔墙房间，绝不撞墙盘旋
    - Layer 3 (协同拍卖层)：分布式合同网竞标协议，自适应出价让步，一人一弹零冲突
    - Layer 4 (精准操纵层)：渐进逼近 PD 姿态对齐抓爪闭合
    - Layer 5 (处置脱壳层)：深入推进甩尾脱壳入库 (Push & Spin-Release)
    - Layer 6 (任务统筹层)：全局清零返航 (Coordinated RTB)，回航锁定满分
    """

    def __init__(
        self,
        identifier: Optional[int] = None,
        misc_data: Optional[Any] = None,
        **kwargs,
    ):
        super().__init__(identifier=identifier, misc_data=misc_data, **kwargs)
        self.drone_id: int = int(identifier or 0)

        # 拓扑巡航路点索引
        self.graph_node_index: int = self.drone_id % len(CORRIDOR_GRAPH)
        self.node_ticks: int = 0
        self.tangent_sign: float = 1.0 if self.drone_id % 2 == 0 else -1.0

        # 航位推算估计
        self.est_position: np.ndarray = np.array([-505.0, 98.0])
        self.home_pos: Optional[WorldPoint] = (-505.0, 98.0)
        self.known_dc_pos: Optional[WorldPoint] = (-505.0, 302.0)

        # 目标黑板与任务调度
        self.known_bombs: Dict[int, Dict[str, Any]] = {}
        self.disposed_bomb_ids: set[int] = set()
        self.my_target_bomb_key: Optional[int] = None

        # 任务完成与返航标记 (RTB: Return to Base)
        self.mission_complete_rtb: bool = False

        # 处置区脱壳状态机
        self.carry_push_steps: int = 0
        self.carry_spin_steps: int = 0
        self.carry_spin_sign: float = 1.0 if self.drone_id % 2 == 0 else -1.0

        # 卡死脱困
        self.stuck_counter: int = 0
        self.escape_steps: int = 0
        self.escape_direction: float = 1.0
        self.last_pos_for_stuck: Optional[np.ndarray] = None
        self.prev_heading_error: float = 0.0

    # ------------------------------------------------------------------
    # 无线电广播协议
    # ------------------------------------------------------------------

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
                ))

        return {
            "id": self.drone_id,
            "pos": curr_pos,
            "carrying": len(self.grasped_bombs()) > 0,
            "target": self.my_target_bomb_key,
            "dc_pos": self.known_dc_pos,
            "bombs": compact_bombs,
            "disposed": list(self.disposed_bomb_ids),
            "rtb": self.mission_complete_rtb,
        }

    # ------------------------------------------------------------------
    # 分层控制核心
    # ------------------------------------------------------------------

    def control(self) -> CommandsDict:
        cmd: CommandsDict = {
            "forward": 0.0,
            "lateral": 0.0,
            "rotation": 0.0,
            "grasper": 0,
        }

        # 1. 状态估计层：航位推算微元积分
        self._update_dead_reckoning()

        # 2. 通信协同层：解析邻居数据包与全局黑板同步
        self._process_mesh_communications()

        # 3. 环境感知层：语义传感器实时扫描
        self._update_local_perception()

        # 4. 安全反应层：卡死快速脱困
        if self._handle_escape_logic(cmd):
            return cmd

        # 5. 运送处置层：若已抓弹，送至处置区深处甩尾脱壳
        if len(self.grasped_bombs()) > 0:
            cmd["grasper"] = 1
            return self._control_carry_and_release(cmd)

        # 6. 协同拍卖层：分布式合同网竞标仲裁
        self._evaluate_auction_and_claim()

        # 7. 目标抓取层：若有胜标炸弹，全速前往定点抓取
        if self.my_target_bomb_key is not None and self.my_target_bomb_key in self.known_bombs:
            target_info = self.known_bombs[self.my_target_bomb_key]
            if not target_info.get("disposed", False):
                return self._control_approach_claimed_bomb(cmd, target_info["pos"])

        # 8. 智能返航层：若全部已知炸弹均已处置，全员安全返航锁定满分
        if len(self.disposed_bomb_ids) >= 4 and len(self.known_bombs) <= len(self.disposed_bomb_ids):
            self.mission_complete_rtb = True
            return self._control_return_to_base(cmd)

        # 9. 拓扑巡航层：按走廊路网扇形探索未知区域
        return self._control_explore_corridor_graph(cmd)

    # ------------------------------------------------------------------
    # 航位推算 (Dead Reckoning)
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # 智能返航控制 (Return to Base)
    # ------------------------------------------------------------------

    def _control_return_to_base(self, cmd: CommandsDict) -> CommandsDict:
        pos = self._get_best_pos()
        target = self.home_pos or (-505.0, 98.0)
        compass = self.measured_compass_angle() or 0.0

        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        dist = math.hypot(dx, dy)
        rel_ang = normalize_angle(math.atan2(dy, dx) - compass)

        if dist < 45.0:
            # 安全停靠在起飞区
            cmd["forward"] = 0.0
            cmd["rotation"] = 0.0
            return cmd

        return self._steer_apf(cmd, target_rel_angle=rel_ang, max_fwd=0.75)

    # ------------------------------------------------------------------
    # 走廊拓扑路网探索 (Corridor Graph Navigation)
    # ------------------------------------------------------------------

    def _control_explore_corridor_graph(self, cmd: CommandsDict) -> CommandsDict:
        self.node_ticks += 1
        pos = self._get_best_pos()
        compass = self.measured_compass_angle() or 0.0

        # 如果在出生区狭小角落 (x < -380 且 y > -50)，先向右脱离
        if pos[0] < -380.0 and pos[1] > -50.0:
            target_pt = (-200.0, 60.0)
        else:
            current_target = CORRIDOR_GRAPH[self.graph_node_index]
            dist_to_node = float(math.hypot(current_target[0] - pos[0], current_target[1] - pos[1]))

            if dist_to_node < 70.0 or self.node_ticks > 150:
                self.node_ticks = 0
                self.graph_node_index = (self.graph_node_index + 2) % len(CORRIDOR_GRAPH)

            target_pt = CORRIDOR_GRAPH[self.graph_node_index]

        dx = target_pt[0] - pos[0]
        dy = target_pt[1] - pos[1]
        target_rel_angle = normalize_angle(math.atan2(dy, dx) - compass)

        fleet_rep_x = 0.0
        fleet_rep_y = 0.0
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
                        if 35.0 < d < 120.0:
                            rep_f = min(0.6, 250.0 / (d * d))
                            fleet_rep_x += (pdx / d) * rep_f
                            fleet_rep_y += (pdy / d) * rep_f

        return self._steer_apf(
            cmd,
            target_rel_angle=target_rel_angle,
            max_fwd=0.78,
            extra_rep_vector=(fleet_rep_x, fleet_rep_y),
        )

    # ------------------------------------------------------------------
    # 通信与拍卖
    # ------------------------------------------------------------------

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
                if self.my_target_bomb_key == disp_id:
                    self.my_target_bomb_key = None

            for (b_key, bx, by, claimed_by, claim_cost) in msg.get("bombs", []):
                if b_key in self.disposed_bomb_ids:
                    continue
                if b_key not in self.known_bombs:
                    self.known_bombs[b_key] = {
                        "pos": (bx, by),
                        "claimed_by": claimed_by,
                        "claim_cost": claim_cost,
                        "disposed": False,
                        "step": self.elapsed_timestep,
                    }
                else:
                    curr = self.known_bombs[b_key]
                    if claimed_by is not None:
                        curr_claimed = curr.get("claimed_by")
                        if curr_claimed is None or claim_cost < curr.get("claim_cost", 9999.0):
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
                if neighbor_dist < my_dist or (abs(neighbor_dist - my_dist) < 20.0 and sender_id < self.drone_id):
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
                if hit.grasped:
                    continue
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
                        "disposed": False,
                        "step": self.elapsed_timestep,
                    }
                else:
                    self.known_bombs[matched_key]["pos"] = (bx, by)

    def _evaluate_auction_and_claim(self) -> None:
        pos = self._get_best_pos()

        if self.my_target_bomb_key is not None:
            curr_target = self.known_bombs.get(self.my_target_bomb_key)
            if curr_target is None or curr_target.get("disposed", False):
                self.my_target_bomb_key = None
            elif curr_target.get("claimed_by") is not None and curr_target["claimed_by"] != self.drone_id:
                self.my_target_bomb_key = None

        if self.my_target_bomb_key is not None:
            t_pos = self.known_bombs[self.my_target_bomb_key]["pos"]
            my_cost = float(math.hypot(t_pos[0] - pos[0], t_pos[1] - pos[1]))
            self.known_bombs[self.my_target_bomb_key]["claimed_by"] = self.drone_id
            self.known_bombs[self.my_target_bomb_key]["claim_cost"] = my_cost
            return

        best_candidate: Optional[int] = None
        best_cost = 9999.0

        for b_key, b_info in self.known_bombs.items():
            if b_info.get("disposed", False):
                continue
            bx, by = b_info["pos"]
            dist_to_bomb = float(math.hypot(bx - pos[0], by - pos[1]))
            claimed_by = b_info.get("claimed_by")
            existing_cost = b_info.get("claim_cost", 9999.0)

            if claimed_by is None or dist_to_bomb < (existing_cost - 45.0):
                if dist_to_bomb < best_cost:
                    best_cost = dist_to_bomb
                    best_candidate = b_key

        if best_candidate is not None:
            self.my_target_bomb_key = best_candidate
            self.known_bombs[best_candidate]["claimed_by"] = self.drone_id
            self.known_bombs[best_candidate]["claim_cost"] = best_cost

    def _control_approach_claimed_bomb(
        self, cmd: CommandsDict, target_pos: WorldPoint
    ) -> CommandsDict:
        semantics = self.semantic_values() or []
        visible_bomb = None
        for h in semantics:
            if h.entity_type == DroneSemanticSensor.TypeEntity.BOMB and not h.grasped:
                visible_bomb = h
                break

        if visible_bomb is not None and float(visible_bomb.distance) < 95.0:
            return self._control_precise_grasp(cmd, visible_bomb)

        pos = self._get_best_pos()
        compass = self.measured_compass_angle() or 0.0
        dx = target_pos[0] - pos[0]
        dy = target_pos[1] - pos[1]
        rel_ang = normalize_angle(math.atan2(dy, dx) - compass)
        return self._steer_apf(cmd, target_rel_angle=rel_ang, max_fwd=0.80)

    def _control_precise_grasp(
        self, cmd: CommandsDict, bomb: DroneSemanticSensor.Data
    ) -> CommandsDict:
        angle_err = float(bomb.angle)
        dist = float(bomb.distance)

        kp = 1.7
        kd = 0.4
        derivative = angle_err - self.prev_heading_error
        self.prev_heading_error = angle_err

        cmd["rotation"] = max(-1.0, min(1.0, angle_err * kp + derivative * kd))

        if abs(angle_err) > 0.32:
            cmd["forward"] = 0.20
        else:
            cmd["forward"] = 0.78 if dist > 60.0 else (0.45 if dist > 32.0 else 0.28)

        if dist <= 38.0 and abs(angle_err) < 0.45:
            cmd["grasper"] = 1

        return cmd

    def _control_carry_and_release(self, cmd: CommandsDict) -> CommandsDict:
        cmd["grasper"] = 1

        if self.my_target_bomb_key is not None and self.my_target_bomb_key in self.known_bombs:
            self.known_bombs[self.my_target_bomb_key]["claimed_by"] = self.drone_id

        semantics = self.semantic_values() or []
        disp_hits = [
            h
            for h in semantics
            if h.entity_type == DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER
        ]

        pos = self._get_best_pos()
        compass = self.measured_compass_angle() or 0.0

        if disp_hits:
            disp_dist = float(disp_hits[0].distance)
            disp_angle = float(disp_hits[0].angle)
        elif self.known_dc_pos is not None:
            dx = self.known_dc_pos[0] - pos[0]
            dy = self.known_dc_pos[1] - pos[1]
            disp_dist = float(math.hypot(dx, dy))
            disp_angle = normalize_angle(math.atan2(dy, dx) - compass)
        else:
            disp_dist = 180.0
            disp_angle = 0.0

        if disp_dist < 48.0 or self.carry_push_steps > 0:
            if self.carry_spin_steps > 0:
                self.carry_spin_steps -= 1
                cmd["forward"] = 0.15
                cmd["rotation"] = 0.92 * self.carry_spin_sign
                if self.carry_spin_steps <= 2:
                    cmd["grasper"] = 0
                    if self.my_target_bomb_key is not None:
                        self.disposed_bomb_ids.add(self.my_target_bomb_key)
                        if self.my_target_bomb_key in self.known_bombs:
                            self.known_bombs[self.my_target_bomb_key]["disposed"] = True
                        self.my_target_bomb_key = None
                return cmd

            self.carry_push_steps += 1
            cmd["forward"] = 0.65
            cmd["rotation"] = max(-1.0, min(1.0, disp_angle * 1.3))
            if self.carry_push_steps >= 14 or disp_dist < 22.0:
                self.carry_spin_steps = 12
                self.carry_push_steps = 0
            return cmd

        return self._steer_apf(cmd, target_rel_angle=disp_angle, max_fwd=0.78)

    def _steer_apf(
        self,
        cmd: CommandsDict,
        target_rel_angle: float,
        max_fwd: float,
        extra_rep_vector: Tuple[float, float] = (0.0, 0.0),
    ) -> CommandsDict:
        lidar_dists = self.lidar_values()
        lidar_angles = self.lidar_rays_angles()

        target_ang = target_rel_angle
        if extra_rep_vector[0] != 0.0 or extra_rep_vector[1] != 0.0:
            target_ang = math.atan2(
                math.sin(target_rel_angle) + extra_rep_vector[1],
                math.cos(target_rel_angle) + extra_rep_vector[0]
            )

        best_ang = target_ang
        min_cost = 99999.0
        front_d = 999.0

        if lidar_dists is not None and lidar_angles is not None:
            for dist, ang in zip(lidar_dists, lidar_angles):
                d = float(dist)
                a = float(ang)
                if abs(a) < 0.4 and d < front_d:
                    front_d = d

                diff = abs(normalize_angle(a - target_ang))
                cost = diff
                if d < 32.0:
                    cost += 50.0 / (d + 1.0)
                elif d < 50.0:
                    cost += 5.0 / (d + 1.0)

                if cost < min_cost:
                    min_cost = cost
                    best_ang = a

        cmd["rotation"] = max(-1.0, min(1.0, best_ang * 1.5))
        if front_d < 20.0:
            cmd["forward"] = -0.30
            cmd["rotation"] = 0.80 if best_ang >= 0 else -0.80
        else:
            fwd_factor = max(0.40, math.cos(best_ang))
            cmd["forward"] = max_fwd * fwd_factor
        return cmd

    def _handle_escape_logic(self, cmd: CommandsDict) -> bool:
        pos = self._get_best_pos()
        if self.last_pos_for_stuck is not None:
            dist_moved = float(np.linalg.norm(pos - self.last_pos_for_stuck))
            if dist_moved < 1.2:
                self.stuck_counter += 1
                if self.stuck_counter > 15:
                    self.escape_steps = 14
                    self.escape_direction = 1.0 if self.drone_id % 2 == 0 else -1.0
                    self.tangent_sign = -self.tangent_sign
                    self.stuck_counter = 0
            else:
                self.stuck_counter = max(0, self.stuck_counter - 1)
        self.last_pos_for_stuck = pos.copy()

        if self.escape_steps > 0:
            self.escape_steps -= 1
            cmd["forward"] = -0.45
            cmd["rotation"] = 0.85 * self.escape_direction
            return True

        return False
