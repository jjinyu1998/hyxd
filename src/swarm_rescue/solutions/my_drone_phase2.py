"""
蓝队第二阶段控制器：多机分布式通信协同 + 动态任务竞标拍卖 + 全局目标黑板
文件路径：src/swarm_rescue/solutions/my_drone_phase2.py
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


class MyDronePhase2(DroneAbstract):
    """
    蓝队阶段二：多机通信协同与任务竞标拍卖控制器
    核心突破：
    1. 分布式无线电网络 (Mesh Comms)：250px 通信域内广播自身位姿、已探明炸弹与处置区全局坐标；
    2. 分布式合同网竞标协议 (Auction Protocol)：按距离代价出价认领，胜标者前往抓取，杜绝多机抢夺冲突；
    3. 目标黑板与信息中继 (Shared Blackboard Relay)：先遣机发现的炸弹/处置区经由通信链多跳同步全队；
    4. 友军动态空间软排斥 (Fleet Spatial Repulsion)：无任务巡航时机群相互排斥，呈气体状均匀扩散全图；
    5. 全图炸弹清零判定与返航集结 (Coordinated RTB)：全部目标清退后有序进入 Return Area 锁定满分。
    """

    def __init__(
        self,
        identifier: Optional[int] = None,
        misc_data: Optional[Any] = None,
        **kwargs,
    ):
        super().__init__(identifier=identifier, misc_data=misc_data, **kwargs)
        self.drone_id: int = int(identifier or 0)

        # 初始扇区分散航向与巡航计数器
        self.sector_angle: float = (float(self.drone_id) * (2.0 * math.pi)) / 10.0
        self.explore_tick: int = self.drone_id * 20

        # 起始点（返航基准点）
        self.home_pos: Optional[WorldPoint] = None

        # 处置中心已知全局坐标
        self.known_dc_pos: Optional[WorldPoint] = None

        # 全局炸弹记忆库: bomb_key -> {"pos": (x, y), "claimed_by": id, "claim_cost": float, "disposed": bool, "step": int}
        self.known_bombs: Dict[int, Dict[str, Any]] = {}
        self.disposed_bomb_ids: set[int] = set()

        # 本机当前认领锁定的目标炸弹 key
        self.my_target_bomb_key: Optional[int] = None

        # 运送与处置脱壳状态机
        self.carry_push_steps: int = 0
        self.carry_spin_steps: int = 0
        self.carry_spin_sign: float = 1.0 if self.drone_id % 2 == 0 else -1.0

        # 脱困与卡死检测
        self.stuck_counter: int = 0
        self.escape_steps: int = 0
        self.escape_direction: float = 1.0
        self.last_pos: Optional[np.ndarray] = None
        self.prev_heading_error: float = 0.0

    # ------------------------------------------------------------------
    # 无线电广播协议 (通信半径 250px)
    # ------------------------------------------------------------------

    def define_message_for_all(self) -> Any:
        """
        每步向通信范围内的友军无人机广播状态数据包
        """
        pos = self.measured_gps_position()
        curr_pos = (float(pos[0]), float(pos[1])) if pos is not None else None

        # 压缩共享的炸弹数据（仅同步未处置的目标列表，控制通信开销）
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

        message = {
            "id": self.drone_id,
            "pos": curr_pos,
            "carrying": len(self.grasped_bombs()) > 0,
            "target": self.my_target_bomb_key,
            "dc_pos": self.known_dc_pos,
            "bombs": compact_bombs,
            "disposed": list(self.disposed_bomb_ids),
        }
        return message

    # ------------------------------------------------------------------
    # 核心控制主循环
    # ------------------------------------------------------------------

    def control(self) -> CommandsDict:
        cmd: CommandsDict = {
            "forward": 0.0,
            "lateral": 0.0,
            "rotation": 0.0,
            "grasper": 0,
        }

        # 记录初始出生点作为返航基准点
        pos = self.measured_gps_position()
        if self.home_pos is None and pos is not None:
            self.home_pos = (float(pos[0]), float(pos[1]))

        # 0. 接收并解析所有通信范围内的邻居消息（合同网竞标协商）
        self._process_incoming_messages()

        # 1. 扫描自身传感器，更新本地世界黑板（炸弹去重与处置中心建档）
        self._update_local_perception()

        # 2. 卡死/死胡同脱困自解算
        if self._handle_escape_logic(cmd):
            return cmd

        # 3. 如果当前正在抓取/运送炸弹 -> 导航送往处置中心并甩尾入库
        if len(self.grasped_bombs()) > 0:
            cmd["grasper"] = 1
            return self._control_carry_and_release(cmd)

        # 4. 执行分布式竞标决策：为自己选定胜标的最优炸弹
        self._evaluate_auction_and_claim()

        # 5. 如果成功锁定到竞标炸弹 -> 全速导航前往逼近与抓取
        if self.my_target_bomb_key is not None and self.my_target_bomb_key in self.known_bombs:
            target_info = self.known_bombs[self.my_target_bomb_key]
            if not target_info.get("disposed", False):
                return self._control_approach_claimed_bomb(cmd, target_info["pos"])

        # 6. 无锁定目标 -> 全机群软排斥自主发散巡航（高覆盖度探图）
        return self._control_explore_with_fleet_repulsion(cmd)

    # ------------------------------------------------------------------
    # 通信消息解析与合同网竞标协商 (Distributed Auction Process)
    # ------------------------------------------------------------------

    def _process_incoming_messages(self) -> None:
        """
        解析并合并邻近无人机的广播数据包，执行竞标比价仲裁
        """
        if self.communicator_is_disabled():
            return

        received = getattr(self.communicator, "received_messages", [])
        if not received:
            return

        my_pos = self.measured_gps_position()

        for item in received:
            if not isinstance(item, tuple) or len(item) < 2:
                continue
            _, msg = item
            if not isinstance(msg, dict):
                continue

            sender_id = msg.get("id")
            if sender_id == self.drone_id:
                continue

            # 1. 共享处置中心坐标（发现即全网同步）
            if self.known_dc_pos is None and msg.get("dc_pos") is not None:
                self.known_dc_pos = tuple(msg["dc_pos"])

            # 2. 合并已处置的炸弹集合
            for disp_id in msg.get("disposed", []):
                self.disposed_bomb_ids.add(disp_id)
                if disp_id in self.known_bombs:
                    self.known_bombs[disp_id]["disposed"] = True
                if self.my_target_bomb_key == disp_id:
                    self.my_target_bomb_key = None

            # 3. 炸弹竞标信息同步与最低出价仲裁
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

            # 4. 如果邻居的认领目标与我冲突，且邻居代价比我小，我主动让出，避免扎堆争抢！
            neighbor_target = msg.get("target")
            if (
                neighbor_target is not None
                and neighbor_target == self.my_target_bomb_key
                and my_pos is not None
            ):
                target_pos = self.known_bombs[self.my_target_bomb_key]["pos"]
                my_dist = float(math.hypot(target_pos[0] - my_pos[0], target_pos[1] - my_pos[1]))
                neighbor_dist = float(msg.get("claim_cost", 9999.0))
                if neighbor_dist < my_dist or (abs(neighbor_dist - my_dist) < 15.0 and sender_id < self.drone_id):
                    self.known_bombs[self.my_target_bomb_key]["claimed_by"] = sender_id
                    self.my_target_bomb_key = None

    # ------------------------------------------------------------------
    # 本地感知与目标黑板建档
    # ------------------------------------------------------------------

    def _update_local_perception(self) -> None:
        pos = self.measured_gps_position()
        compass = self.measured_compass_angle() or 0.0
        semantics = self.semantic_values() or []

        for hit in semantics:
            # 发现处置中心：记录全局坐标
            if hit.entity_type == DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER:
                if self.known_dc_pos is None and pos is not None:
                    global_ang = normalize_angle(compass + float(hit.angle))
                    dc_x = float(pos[0]) + float(hit.distance) * math.cos(global_ang)
                    dc_y = float(pos[1]) + float(hit.distance) * math.sin(global_ang)
                    self.known_dc_pos = (dc_x, dc_y)

            # 发现炸弹：解算全局坐标并聚类去重
            elif hit.entity_type == DroneSemanticSensor.TypeEntity.BOMB:
                if hit.grasped:
                    continue
                if pos is not None:
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

    # ------------------------------------------------------------------
    # 分布式拍卖竞标评估 (Auction Evaluation)
    # ------------------------------------------------------------------

    def _evaluate_auction_and_claim(self) -> None:
        pos = self.measured_gps_position()
        if pos is None:
            return

        # 检查当前认领的目标是否仍然有效
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

            if claimed_by is None or dist_to_bomb < (existing_cost - 40.0):
                if dist_to_bomb < best_cost:
                    best_cost = dist_to_bomb
                    best_candidate = b_key

        if best_candidate is not None:
            self.my_target_bomb_key = best_candidate
            self.known_bombs[best_candidate]["claimed_by"] = self.drone_id
            self.known_bombs[best_candidate]["claim_cost"] = best_cost

    # ------------------------------------------------------------------
    # 逼近与抓取认领目标
    # ------------------------------------------------------------------

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

        pos = self.measured_gps_position()
        compass = self.measured_compass_angle() or 0.0
        if pos is not None:
            dx = target_pos[0] - pos[0]
            dy = target_pos[1] - pos[1]
            rel_ang = normalize_angle(math.atan2(dy, dx) - compass)
            return self._steer_apf(cmd, target_rel_angle=rel_ang, max_fwd=0.80)

        return self._steer_apf(cmd, target_rel_angle=0.0, max_fwd=0.50)

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
            cmd["forward"] = 0.15
        else:
            cmd["forward"] = 0.75 if dist > 60.0 else (0.45 if dist > 32.0 else 0.28)

        if dist <= 38.0 and abs(angle_err) < 0.45:
            cmd["grasper"] = 1

        return cmd

    # ------------------------------------------------------------------
    # 运送与甩尾脱壳入库 (Push & Spin-Release)
    # ------------------------------------------------------------------

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

        pos = self.measured_gps_position()
        compass = self.measured_compass_angle() or 0.0

        if disp_hits:
            disp_dist = float(disp_hits[0].distance)
            disp_angle = float(disp_hits[0].angle)
        elif self.known_dc_pos is not None and pos is not None:
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
                cmd["forward"] = 0.12
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

        return self._steer_apf(cmd, target_rel_angle=disp_angle, max_fwd=0.75)

    # ------------------------------------------------------------------
    # 友军空间软排斥巡航探索 (Fleet Soft Repulsion)
    # ------------------------------------------------------------------

    def _control_explore_with_fleet_repulsion(self, cmd: CommandsDict) -> CommandsDict:
        self.explore_tick += 1
        pos = self.measured_gps_position()
        compass = self.measured_compass_angle() or 0.0

        base_angle = self.sector_angle + math.sin(self.explore_tick * 0.04) * 0.8
        target_rel_angle = normalize_angle(base_angle - compass)

        fleet_rep_x = 0.0
        fleet_rep_y = 0.0

        if pos is not None and not self.communicator_is_disabled():
            received = getattr(self.communicator, "received_messages", [])
            for item in received:
                if isinstance(item, tuple) and len(item) >= 2:
                    _, msg = item
                    if isinstance(msg, dict) and msg.get("pos") is not None:
                        nx, ny = msg["pos"]
                        dx = pos[0] - nx
                        dy = pos[1] - ny
                        d = float(math.hypot(dx, dy))
                        if 1.0 < d < 120.0:
                            rep_f = 600.0 / (d * d)
                            fleet_rep_x += (dx / d) * rep_f
                            fleet_rep_y += (dy / d) * rep_f

        return self._steer_apf(
            cmd,
            target_rel_angle=target_rel_angle,
            max_fwd=0.72,
            extra_rep_vector=(fleet_rep_x, fleet_rep_y),
        )

    # ------------------------------------------------------------------
    # APF 人工势场法核心计算 (融合 Lidar 墙壁斥力与友军排斥)
    # ------------------------------------------------------------------

    def _steer_apf(
        self,
        cmd: CommandsDict,
        target_rel_angle: float,
        max_fwd: float,
        extra_rep_vector: Tuple[float, float] = (0.0, 0.0),
    ) -> CommandsDict:
        att_gain = 1.0
        fx = math.cos(target_rel_angle) * att_gain + extra_rep_vector[0]
        fy = math.sin(target_rel_angle) * att_gain + extra_rep_vector[1]

        lidar_dists = self.lidar_values()
        lidar_angles = self.lidar_rays_angles()
        min_dist = 999.0

        if lidar_dists is not None and lidar_angles is not None:
            rep_gain = 1400.0
            for dist, ang in zip(lidar_dists, lidar_angles):
                d = float(dist)
                if d < min_dist:
                    min_dist = d
                if d < 52.0:
                    rep = rep_gain / ((d + 5.0) * (d + 5.0))
                    fx -= math.cos(float(ang)) * rep
                    fy -= math.sin(float(ang)) * rep

        desired_ang = math.atan2(fy, fx)
        cmd["rotation"] = max(-1.0, min(1.0, desired_ang * 1.45))

        if abs(desired_ang) > 0.6:
            cmd["forward"] = 0.15
        elif min_dist < 28.0:
            cmd["forward"] = 0.10
        elif min_dist < 42.0:
            cmd["forward"] = max_fwd * 0.55
        else:
            cmd["forward"] = max_fwd

        return cmd

    # ------------------------------------------------------------------
    # 卡死自解困状态机
    # ------------------------------------------------------------------

    def _handle_escape_logic(self, cmd: CommandsDict) -> bool:
        pos = self.measured_gps_position()
        if pos is not None and self.last_pos is not None:
            dist_moved = float(np.linalg.norm(pos - self.last_pos))
            if dist_moved < 1.2:
                self.stuck_counter += 1
                if self.stuck_counter > 16:
                    self.escape_steps = 15
                    self.escape_direction = 1.0 if self.drone_id % 2 == 0 else -1.0
                    self.stuck_counter = 0
            else:
                self.stuck_counter = max(0, self.stuck_counter - 1)
        self.last_pos = pos

        if self.escape_steps > 0:
            self.escape_steps -= 1
            cmd["forward"] = -0.45
            cmd["rotation"] = 0.85 * self.escape_direction
            return True

        return False
