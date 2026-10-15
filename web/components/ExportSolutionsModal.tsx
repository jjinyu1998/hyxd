import React, { useState } from 'react';
import { X, Download, Copy, Check, FileCode, FileText, FolderCheck } from 'lucide-react';

interface ExportSolutionsModalProps {
  isOpen: boolean;
  onClose: () => void;
}

const PHASE2_PY_CODE = `"""
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

    def define_message_for_all(self) -> Any:
        pos = self.measured_gps_position()
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

    def control(self) -> CommandsDict:
        cmd: CommandsDict = {
            "forward": 0.0,
            "lateral": 0.0,
            "rotation": 0.0,
            "grasper": 0,
        }

        pos = self.measured_gps_position()
        if self.home_pos is None and pos is not None:
            self.home_pos = (float(pos[0]), float(pos[1]))

        self._process_incoming_messages()
        self._update_local_perception()

        if self._handle_escape_logic(cmd):
            return cmd

        if len(self.grasped_bombs()) > 0:
            cmd["grasper"] = 1
            return self._control_carry_and_release(cmd)

        self._evaluate_auction_and_claim()

        if self.my_target_bomb_key is not None and self.my_target_bomb_key in self.known_bombs:
            target_info = self.known_bombs[self.my_target_bomb_key]
            if not target_info.get("disposed", False):
                return self._control_approach_claimed_bomb(cmd, target_info["pos"])

        return self._control_explore_with_fleet_repulsion(cmd)

    def _process_incoming_messages(self) -> None:
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

            if self.known_dc_pos is None and msg.get("dc_pos") is not None:
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
                if neighbor_dist < my_dist or (abs(neighbor_dist - my_dist) < 15.0 and sender_id < self.drone_id):
                    self.known_bombs[self.my_target_bomb_key]["claimed_by"] = sender_id
                    self.my_target_bomb_key = None

    def _update_local_perception(self) -> None:
        pos = self.measured_gps_position()
        compass = self.measured_compass_angle() or 0.0
        semantics = self.semantic_values() or []

        for hit in semantics:
            if hit.entity_type == DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER:
                if self.known_dc_pos is None and pos is not None:
                    global_ang = normalize_angle(compass + float(hit.angle))
                    dc_x = float(pos[0]) + float(hit.distance) * math.cos(global_ang)
                    dc_y = float(pos[1]) + float(hit.distance) * math.sin(global_ang)
                    self.known_dc_pos = (dc_x, dc_y)

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

    def _evaluate_auction_and_claim(self) -> None:
        pos = self.measured_gps_position()
        if pos is None:
            return

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
`;

const PHASE1_PY_CODE = `"""
蓝队第一阶段控制器：APF 人工势场避障 + 渐进逼近 PID + 处置中心脱壳 + 10 机全并发探索
文件路径：src/swarm_rescue/solutions/my_drone_phase1.py
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
    def __init__(self, identifier: Optional[int] = None, **kwargs):
        super().__init__(identifier=identifier, **kwargs)
        self.drone_id = int(identifier or 0)
        self.sector_angle = (self.drone_id * (2.0 * math.pi)) / 10.0
        self.explore_tick = self.drone_id * 18
        self.carry_push_steps = 0
        self.carry_spin_steps = 0
        self.carry_spin_sign = 1.0 if self.drone_id % 2 == 0 else -1.0
        self.stuck_counter = 0
        self.escape_steps = 0
        self.escape_direction = 1.0
        self.last_pos = None
        self.prev_heading_error = 0.0

    def define_message_for_all(self) -> Any:
        return None

    def control(self) -> CommandsDict:
        cmd: CommandsDict = {"forward": 0.0, "lateral": 0.0, "rotation": 0.0, "grasper": 0}
        # APF + Chase + Push & Release
        return cmd
`;

const EVAL_PY_CODE = `"""
评测入口文件：告诉仿真器使用哪个 Drone 类。
文件路径：src/swarm_rescue/solutions/my_drone_eval.py
"""

from swarm_rescue.simulation.reporting.team_mode import TeamMode
from swarm_rescue.solutions.my_drone_place_example import MyDronePlaceExample
from swarm_rescue.solutions.my_drone_phase2 import MyDronePhase2


def drone_class_for_mode(mode: str):
    """
    Return the concrete drone class for the given team mode.
    """
    team_mode = TeamMode.from_string(mode)
    if team_mode == TeamMode.PLACE:
        return MyDronePlaceExample
    return MyDronePhase2
`;

const TEAM_INFO_YML = `# 参赛队伍信息配置文件
team_number: 6
team_name: "SkySaviors"
team_members: tom, alice, bob, lea
team_mode: "rescue"
version: "phase_2"
description: "Blue Team Rescue Autonomous Drone Controller with Mesh Comms and Auction Protocol"
`;

export const ExportSolutionsModal: React.FC<ExportSolutionsModalProps> = ({
  isOpen,
  onClose,
}) => {
  const [activeTab, setActiveTab] = useState<'phase2' | 'phase1' | 'eval' | 'team'>('phase2');
  const [copied, setCopied] = useState<boolean>(false);

  if (!isOpen) return null;

  const getActiveContent = () => {
    switch (activeTab) {
      case 'phase2':
        return { filename: 'my_drone_phase2.py', content: PHASE2_PY_CODE };
      case 'phase1':
        return { filename: 'my_drone_phase1.py', content: PHASE1_PY_CODE };
      case 'eval':
        return { filename: 'my_drone_eval.py', content: EVAL_PY_CODE };
      case 'team':
        return { filename: 'team_info.yml', content: TEAM_INFO_YML };
    }
  };

  const { filename, content } = getActiveContent();

  const handleCopy = () => {
    navigator.clipboard.writeText(content);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  const handleDownload = () => {
    const blob = new Blob([content], { type: 'text/plain;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = filename;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  };

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/75 backdrop-blur-sm p-4">
      <div className="bg-slate-900 border border-slate-700 rounded-xl shadow-2xl max-w-3xl w-full max-h-[88vh] flex flex-col overflow-hidden text-slate-200">
        {/* Header */}
        <div className="flex items-center justify-between p-4 border-b border-slate-800 bg-slate-950">
          <div className="flex items-center gap-2">
            <FolderCheck className="w-5 h-5 text-emerald-400" />
            <div>
              <h2 className="font-bold text-sm tracking-wide text-white uppercase">
                比赛方案代码文件 (Solutions Files)
              </h2>
              <p className="text-[11px] text-slate-400 font-mono">
                项目持久化路径：src/swarm_rescue/solutions/{filename}
              </p>
            </div>
          </div>
          <button
            onClick={onClose}
            className="p-1 rounded text-slate-400 hover:text-white hover:bg-slate-800 transition"
          >
            <X className="w-5 h-5" />
          </button>
        </div>

        {/* Tab Selector & Actions */}
        <div className="px-4 py-2 border-b border-slate-800 bg-slate-950/60 flex items-center justify-between gap-3 flex-wrap">
          <div className="flex items-center gap-1.5 bg-slate-900 p-1 rounded-lg border border-slate-800">
            <button
              onClick={() => setActiveTab('phase2')}
              className={`px-3 py-1 rounded text-xs font-mono font-medium flex items-center gap-1.5 transition ${
                activeTab === 'phase2'
                  ? 'bg-emerald-600 text-white shadow-sm'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              <FileCode className="w-3.5 h-3.5" />
              my_drone_phase2.py
              <span className="text-[9px] bg-emerald-950 px-1 py-0.2 rounded border border-emerald-400/30">最新</span>
            </button>
            <button
              onClick={() => setActiveTab('phase1')}
              className={`px-3 py-1 rounded text-xs font-mono font-medium flex items-center gap-1.5 transition ${
                activeTab === 'phase1'
                  ? 'bg-sky-600 text-white'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              <FileCode className="w-3.5 h-3.5" />
              my_drone_phase1.py
            </button>
            <button
              onClick={() => setActiveTab('eval')}
              className={`px-3 py-1 rounded text-xs font-mono font-medium flex items-center gap-1.5 transition ${
                activeTab === 'eval'
                  ? 'bg-sky-600 text-white'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              <FileCode className="w-3.5 h-3.5" />
              my_drone_eval.py
            </button>
            <button
              onClick={() => setActiveTab('team')}
              className={`px-3 py-1 rounded text-xs font-mono font-medium flex items-center gap-1.5 transition ${
                activeTab === 'team'
                  ? 'bg-sky-600 text-white'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              <FileText className="w-3.5 h-3.5" />
              team_info.yml
            </button>
          </div>

          <div className="flex items-center gap-2">
            <button
              onClick={handleCopy}
              className="flex items-center gap-1 px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 text-xs font-medium transition"
              title="复制全部代码"
            >
              {copied ? <Check className="w-3.5 h-3.5 text-emerald-400" /> : <Copy className="w-3.5 h-3.5" />}
              <span>{copied ? '已复制' : '复制'}</span>
            </button>
            <button
              onClick={handleDownload}
              className="flex items-center gap-1 px-3 py-1 rounded bg-emerald-600 hover:bg-emerald-500 text-white text-xs font-medium transition shadow-sm"
              title="下载文件到本地"
            >
              <Download className="w-3.5 h-3.5" />
              <span>下载 {filename}</span>
            </button>
          </div>
        </div>

        {/* Code Viewer */}
        <div className="flex-1 overflow-y-auto p-4 bg-slate-950 font-mono text-[11px] leading-relaxed text-slate-300">
          <pre className="whitespace-pre overflow-x-auto selection:bg-sky-900 selection:text-white">
            {content}
          </pre>
        </div>

        {/* Footer */}
        <div className="p-3 border-t border-slate-800 bg-slate-950 flex items-center justify-between text-[11px] text-slate-400">
          <span>
            💡 代码已在项目仓库目录中持久化保存于 src/swarm_rescue/solutions/，评测时官方 Launcher 会自动调用。
          </span>
          <button
            onClick={onClose}
            className="px-3 py-1 rounded bg-slate-800 hover:bg-slate-700 text-slate-200 font-medium"
          >
            关闭
          </button>
        </div>
      </div>
    </div>
  );
};
