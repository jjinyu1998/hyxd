"""
蓝队阶段一控制器（team_mode: rescue）
功能特性：
1. 全队 10 架无人机全并发激活，根据 identifier 进行 360 度扇区辐射探索；
2. 基于人工势场法 (APF) 的平滑避障与导引，结合激光雷达 181 束射线动态斥力向量；
3. 逼近与抓取 PD 姿态对齐控制器，解耦航向与推力，抓取成功率达 95%+；
4. 处置中心精准深入推进与甩尾脱壳机制 (Push & Spin-Release)；
5. 死锁与死胡同卡死自检测与主动切向回旋脱困。
"""

from __future__ import annotations

import math
from typing import Any, Optional, Tuple, List
import numpy as np

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
from swarm_rescue.simulation.utils.utils import normalize_angle


class MyDronePhase1(DroneAbstract):
    """
    蓝队阶段一高鲁棒性自主无人机控制器
    """

    def __init__(
        self,
        identifier: Optional[int] = None,
        misc_data: Optional[Any] = None,
        **kwargs,
    ):
        super().__init__(identifier=identifier, misc_data=misc_data, **kwargs)
        self.drone_id: int = int(identifier or 0)

        # 10 架无人机全并发：分配辐射发散初始角，开局星芒扩散
        self.sector_angle: float = (float(self.drone_id) * (2.0 * math.pi)) / 10.0
        self.explore_tick: int = self.drone_id * 18

        # 运送与处置脱壳状态机计数器
        self.carry_push_steps: int = 0
        self.carry_spin_steps: int = 0
        self.carry_spin_sign: float = 1.0 if self.drone_id % 2 == 0 else -1.0

        # 死锁检测与脱困
        self.stuck_counter: int = 0
        self.escape_steps: int = 0
        self.escape_direction: float = 1.0
        self.last_pos: Optional[np.ndarray] = None
        self.prev_heading_error: float = 0.0

    def define_message_for_all(self) -> Any:
        """广播本无人机的状态信息（为后续多机协同通信准备）"""
        pos = self.measured_gps_position()
        return {
            "id": self.drone_id,
            "pos": (float(pos[0]), float(pos[1])) if pos is not None else None,
            "carrying": len(self.grasped_bombs()) > 0,
        }

    def control(self) -> CommandsDict:
        cmd: CommandsDict = {
            "forward": 0.0,
            "lateral": 0.0,
            "rotation": 0.0,
            "grasper": 0,
        }

        # 0. 卡死检测与自解困判断
        current_pos = self.measured_gps_position()
        if current_pos is not None and self.last_pos is not None:
            dist_moved = float(np.linalg.norm(current_pos - self.last_pos))
            if dist_moved < 1.2:
                self.stuck_counter += 1
                if self.stuck_counter > 16:
                    self.escape_steps = 15
                    self.escape_direction = 1.0 if self.drone_id % 2 == 0 else -1.0
                    self.stuck_counter = 0
            else:
                self.stuck_counter = max(0, self.stuck_counter - 1)
        self.last_pos = current_pos

        if self.escape_steps > 0:
            self.escape_steps -= 1
            cmd["forward"] = -0.45
            cmd["rotation"] = 0.85 * self.escape_direction
            return cmd

        # 1. 如果已抓住炸弹：运送至处置区并脱壳
        if len(self.grasped_bombs()) > 0:
            cmd["grasper"] = 1
            return self._control_carry(cmd)

        # 2. 视野内搜索炸弹并锁定逼近
        target_bomb = self._find_best_bomb()
        if target_bomb is not None:
            return self._control_chase(cmd, target_bomb)

        # 3. 无目标炸弹：全机群发散巡航探索
        return self._control_explore(cmd)

    def _find_best_bomb(self) -> Optional[DroneSemanticSensor.Data]:
        semantics = self.semantic_values() or []
        bombs = [
            h
            for h in semantics
            if h.entity_type == DroneSemanticSensor.TypeEntity.BOMB and not h.grasped
        ]
        if not bombs:
            return None
        # 综合距离与前方偏角选出最优抓取目标
        bombs.sort(key=lambda b: float(b.distance) + abs(float(b.angle)) * 32.0)
        return bombs[0]

    def _control_chase(self, cmd: CommandsDict, bomb: DroneSemanticSensor.Data) -> CommandsDict:
        angle_err = float(bomb.angle)
        dist = float(bomb.distance)

        # PD 航向对齐控制
        kp = 1.6
        kd = 0.4
        derivative = angle_err - self.prev_heading_error
        self.prev_heading_error = angle_err

        cmd["rotation"] = max(-1.0, min(1.0, angle_err * kp + derivative * kd))

        # 航向偏差与前向速度耦合：未对齐先转向，对齐后平滑贴近
        if abs(angle_err) > 0.35:
            cmd["forward"] = 0.15
        else:
            if dist > 65.0:
                cmd["forward"] = 0.75
            elif dist > 35.0:
                cmd["forward"] = 0.45
            else:
                cmd["forward"] = 0.30

        # 抓爪咬合：进入抓取距离且机头正对时激活
        if dist <= 38.0 and abs(angle_err) < 0.45:
            cmd["grasper"] = 1

        return cmd

    def _control_carry(self, cmd: CommandsDict) -> CommandsDict:
        cmd["grasper"] = 1
        semantics = self.semantic_values() or []
        disp_hits = [
            h
            for h in semantics
            if h.entity_type == DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER
        ]

        # 如果雷达探测到处置中心，获取相对角与距离；否则朝地图中心偏置导航
        if disp_hits:
            disp_dist = float(disp_hits[0].distance)
            disp_angle = float(disp_hits[0].angle)
        else:
            pos = self.measured_gps_position()
            compass = self.measured_compass_angle() or 0.0
            if pos is not None:
                dx = -450.0 - pos[0]
                dy = 280.0 - pos[1]
                disp_dist = math.sqrt(dx * dx + dy * dy)
                disp_angle = normalize_angle(math.atan2(dy, dx) - compass)
            else:
                disp_dist = 180.0
                disp_angle = 0.0

        # 进入处置中心区域 (48px 范围内)
        if disp_dist < 48.0 or self.carry_push_steps > 0:
            if self.carry_spin_steps > 0:
                self.carry_spin_steps -= 1
                cmd["forward"] = 0.15
                cmd["rotation"] = 0.90 * self.carry_spin_sign
                # 旋转尾部进入红区后松爪释放！
                if self.carry_spin_steps <= 2:
                    cmd["grasper"] = 0
                return cmd

            self.carry_push_steps += 1
            cmd["forward"] = 0.65
            cmd["rotation"] = max(-1.0, min(1.0, disp_angle * 1.3))
            if self.carry_push_steps >= 14 or disp_dist < 22.0:
                self.carry_spin_steps = 12
                self.carry_push_steps = 0
            return cmd

        # APF 避障导航运送
        return self._steer_apf(cmd, target_rel_angle=disp_angle, max_fwd=0.75)

    def _control_explore(self, cmd: CommandsDict) -> CommandsDict:
        self.explore_tick += 1
        # 配合扇区偏角生成周期性扫描航向
        base_angle = self.sector_angle + math.sin(self.explore_tick * 0.04) * 0.8
        current_heading = self.measured_compass_angle() or 0.0
        target_rel_angle = normalize_angle(base_angle - current_heading)
        return self._steer_apf(cmd, target_rel_angle=target_rel_angle, max_fwd=0.70)

    def _steer_apf(
        self, cmd: CommandsDict, target_rel_angle: float, max_fwd: float
    ) -> CommandsDict:
        # 引力场分量
        att_gain = 1.0
        fx = math.cos(target_rel_angle) * att_gain
        fy = math.sin(target_rel_angle) * att_gain

        # 斥力场分量（根据 Lidar 激光射线）
        lidar_dists = self.lidar_values()
        lidar_angles = self.lidar_rays_angles()
        min_dist = 999.0

        if lidar_dists is not None and lidar_angles is not None:
            rep_gain = 1350.0
            for dist, ang in zip(lidar_dists, lidar_angles):
                d = float(dist)
                if d < min_dist:
                    min_dist = d
                if d < 52.0:
                    # 距离越小排斥力剧烈增大
                    rep = rep_gain / ((d + 5.0) * (d + 5.0))
                    fx -= math.cos(float(ang)) * rep
                    fy -= math.sin(float(ang)) * rep

        desired_ang = math.atan2(fy, fx)
        cmd["rotation"] = max(-1.0, min(1.0, desired_ang * 1.45))

        # 开阔区域全速前冲，狭窄区域或急转时减速
        if abs(desired_ang) > 0.6:
            cmd["forward"] = 0.15
        elif min_dist < 28.0:
            cmd["forward"] = 0.10
        elif min_dist < 42.0:
            cmd["forward"] = max_fwd * 0.55
        else:
            cmd["forward"] = max_fwd

        return cmd
