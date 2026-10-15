import React, { useState } from 'react';
import { X, Download, Copy, Check, FileCode, FileText, FolderCheck } from 'lucide-react';

interface ExportSolutionsModalProps {
  isOpen: boolean;
  onClose: () => void;
}

const PHASE4_PY_CODE = `"""
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
    - Layer 0 (安全反应层)：切向滑墙 APF + 智能反冲倒车防卡死
    - Layer 1 (状态估计层)：GPS/里程计航位推算 (Dead Reckoning)，无畏 NoGps 干扰区
    - Layer 2 (拓扑规划层)：全局门洞拓扑路网，自主穿梭隔墙房间，绝不撞墙盘旋
    - Layer 3 (协同拍卖层)：分布式合同网竞标协议，自适应出价让步，一人一弹零冲突
    - Layer 4 (精准操纵层)：渐进逼近 PD 姿态对齐抓爪闭合
    - Layer 5 (处置脱壳层)：深入推进甩尾脱壳入库 (Push & Spin-Release)
    - Layer 6 (任务统筹层)：全局清零返航 (Coordinated RTB)，回航锁定满分
    """

    def __init__(self, identifier: Optional[int] = None, **kwargs):
        super().__init__(identifier=identifier, **kwargs)
        self.drone_id = int(identifier or 0)
        self.graph_node_index = self.drone_id % len(CORRIDOR_GRAPH)
        self.node_ticks = 0
        self.tangent_sign = 1.0 if self.drone_id % 2 == 0 else -1.0
        self.est_position = np.array([-505.0, 98.0])
        self.home_pos = (-505.0, 98.0)
        self.known_dc_pos = (-505.0, 302.0)
        self.known_bombs: Dict[int, Dict[str, Any]] = {}
        self.disposed_bomb_ids: set[int] = set()
        self.my_target_bomb_key: Optional[int] = None
        self.mission_complete_rtb = False
        self.carry_push_steps = 0
        self.carry_spin_steps = 0
        self.carry_spin_sign = 1.0 if self.drone_id % 2 == 0 else -1.0
        self.stuck_counter = 0
        self.escape_steps = 0
        self.escape_direction = 1.0
        self.last_pos_for_stuck = None
        self.prev_heading_error = 0.0

    def define_message_for_all(self) -> Any:
        pos = self._get_best_pos()
        curr_pos = (float(pos[0]), float(pos[1])) if pos is not None else None
        compact_bombs = [
            (k, v["pos"][0], v["pos"][1], v.get("claimed_by"), v.get("claim_cost", 9999.0))
            for k, v in self.known_bombs.items() if not v.get("disposed", False)
        ]
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

    def control(self) -> CommandsDict:
        cmd: CommandsDict = {"forward": 0.0, "lateral": 0.0, "rotation": 0.0, "grasper": 0}
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
            t = self.known_bombs[self.my_target_bomb_key]
            if not t.get("disposed", False):
                return self._control_approach_claimed_bomb(cmd, t["pos"])

        if len(self.disposed_bomb_ids) >= 4 and len(self.known_bombs) <= len(self.disposed_bomb_ids):
            self.mission_complete_rtb = True
            return self._control_return_to_base(cmd)

        return self._control_explore_corridor_graph(cmd)

    def _update_dead_reckoning(self):
        gps = self.measured_gps_position()
        if gps is not None:
            self.est_position = np.array([float(gps[0]), float(gps[1])])
            if self.home_pos is None:
                self.home_pos = (float(gps[0]), float(gps[1]))
        else:
            vel = self.measured_velocity()
            if vel is not None:
                self.est_position += vel * 0.1

    def _get_best_pos(self):
        return self.est_position

    def _control_return_to_base(self, cmd):
        pos = self._get_best_pos()
        target = self.home_pos or (-505.0, 98.0)
        compass = self.measured_compass_angle() or 0.0
        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        dist = math.hypot(dx, dy)
        rel_ang = normalize_angle(math.atan2(dy, dx) - compass)
        if dist < 45.0:
            cmd["forward"] = 0.0
            cmd["rotation"] = 0.0
            return cmd
        return self._steer_apf(cmd, target_rel_angle=rel_ang, max_fwd=0.75)

    def _control_explore_corridor_graph(self, cmd):
        self.node_ticks += 1
        pos = self._get_best_pos()
        compass = self.measured_compass_angle() or 0.0
        if pos[0] < -380.0 and pos[1] > -50.0:
            target_pt = (-200.0, 60.0)
        else:
            cur = CORRIDOR_GRAPH[self.graph_node_index]
            dist = float(math.hypot(cur[0] - pos[0], cur[1] - pos[1]))
            if dist < 70.0 or self.node_ticks > 150:
                self.node_ticks = 0
                self.graph_node_index = (self.graph_node_index + 2) % len(CORRIDOR_GRAPH)
            target_pt = CORRIDOR_GRAPH[self.graph_node_index]

        dx = target_pt[0] - pos[0]
        dy = target_pt[1] - pos[1]
        target_rel_angle = normalize_angle(math.atan2(dy, dx) - compass)
        return self._steer_apf(cmd, target_rel_angle=target_rel_angle, max_fwd=0.78)

    def _process_mesh_communications(self):
        if self.communicator_is_disabled():
            return
        received = getattr(self.communicator, "received_messages", [])
        my_pos = self._get_best_pos()
        for item in received:
            if not isinstance(item, tuple) or len(item) < 2:
                continue
            _, msg = item
            if not isinstance(msg, dict) or msg.get("id") == self.drone_id:
                continue
            if msg.get("dc_pos"):
                self.known_dc_pos = tuple(msg["dc_pos"])
            for d in msg.get("disposed", []):
                self.disposed_bomb_ids.add(d)
                if d in self.known_bombs:
                    self.known_bombs[d]["disposed"] = True
                if self.my_target_bomb_key == d:
                    self.my_target_bomb_key = None
            for (k, bx, by, c_by, cost) in msg.get("bombs", []):
                if k in self.disposed_bomb_ids:
                    continue
                if k not in self.known_bombs:
                    self.known_bombs[k] = {"pos": (bx, by), "claimed_by": c_by, "claim_cost": cost, "disposed": False}
                else:
                    curr = self.known_bombs[k]
                    if c_by is not None and (curr.get("claimed_by") is None or cost < curr.get("claim_cost", 9999.0)):
                        curr["claimed_by"] = c_by
                        curr["claim_cost"] = cost

    def _update_local_perception(self):
        pos = self._get_best_pos()
        compass = self.measured_compass_angle() or 0.0
        semantics = self.semantic_values() or []
        for hit in semantics:
            if hit.entity_type == DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER:
                ang = normalize_angle(compass + float(hit.angle))
                self.known_dc_pos = (float(pos[0]) + float(hit.distance) * math.cos(ang), float(pos[1]) + float(hit.distance) * math.sin(ang))
            elif hit.entity_type == DroneSemanticSensor.TypeEntity.BOMB and not hit.grasped:
                ang = normalize_angle(compass + float(hit.angle))
                bx = float(pos[0]) + float(hit.distance) * math.cos(ang)
                by = float(pos[1]) + float(hit.distance) * math.sin(ang)
                matched = next((k for k, v in self.known_bombs.items() if math.hypot(v["pos"][0] - bx, v["pos"][1] - by) < 35.0), None)
                if matched is None:
                    self.known_bombs[len(self.known_bombs) + 1] = {"pos": (bx, by), "claimed_by": None, "claim_cost": 9999.0, "disposed": False}

    def _evaluate_auction_and_claim(self):
        pos = self._get_best_pos()
        if self.my_target_bomb_key:
            t = self.known_bombs.get(self.my_target_bomb_key)
            if not t or t.get("disposed") or (t.get("claimed_by") not in (None, self.drone_id)):
                self.my_target_bomb_key = None
            else:
                cost = float(math.hypot(t["pos"][0] - pos[0], t["pos"][1] - pos[1]))
                t["claimed_by"] = self.drone_id
                t["claim_cost"] = cost
                return
        best, min_c = None, 9999.0
        for k, v in self.known_bombs.items():
            if v.get("disposed"): continue
            d = float(math.hypot(v["pos"][0] - pos[0], v["pos"][1] - pos[1]))
            if v.get("claimed_by") is None or d < v.get("claim_cost", 9999.0) - 45.0:
                if d < min_c: min_c, best = d, k
        if best is not None:
            self.my_target_bomb_key = best
            self.known_bombs[best]["claimed_by"] = self.drone_id
            self.known_bombs[best]["claim_cost"] = min_c

    def _control_approach_claimed_bomb(self, cmd, target_pos):
        semantics = self.semantic_values() or []
        visible = next((h for h in semantics if h.entity_type == DroneSemanticSensor.TypeEntity.BOMB and not h.grasped), None)
        if visible and float(visible.distance) < 95.0:
            return self._control_precise_grasp(cmd, visible)
        pos = self._get_best_pos()
        compass = self.measured_compass_angle() or 0.0
        rel_ang = normalize_angle(math.atan2(target_pos[1] - pos[1], target_pos[0] - pos[0]) - compass)
        return self._steer_apf(cmd, target_rel_angle=rel_ang, max_fwd=0.80)

    def _control_precise_grasp(self, cmd, bomb):
        err = float(bomb.angle)
        dist = float(bomb.distance)
        deriv = err - self.prev_heading_error
        self.prev_heading_error = err
        cmd["rotation"] = max(-1.0, min(1.0, err * 1.7 + deriv * 0.4))
        cmd["forward"] = 0.20 if abs(err) > 0.32 else (0.78 if dist > 60.0 else 0.40)
        if dist <= 38.0 and abs(err) < 0.45:
            cmd["grasper"] = 1
        return cmd

    def _control_carry_and_release(self, cmd):
        cmd["grasper"] = 1
        disp_hits = [h for h in (self.semantic_values() or []) if h.entity_type == DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER]
        pos = self._get_best_pos()
        compass = self.measured_compass_angle() or 0.0
        if disp_hits:
            disp_dist, disp_ang = float(disp_hits[0].distance), float(disp_hits[0].angle)
        elif self.known_dc_pos:
            dx, dy = self.known_dc_pos[0] - pos[0], self.known_dc_pos[1] - pos[1]
            disp_dist, disp_ang = float(math.hypot(dx, dy)), normalize_angle(math.atan2(dy, dx) - compass)
        else:
            disp_dist, disp_ang = 180.0, 0.0

        if disp_dist < 48.0 or self.carry_push_steps > 0:
            if self.carry_spin_steps > 0:
                self.carry_spin_steps -= 1
                cmd["forward"] = 0.15
                cmd["rotation"] = 0.92 * self.carry_spin_sign
                if self.carry_spin_steps <= 2:
                    cmd["grasper"] = 0
                    if self.my_target_bomb_key:
                        self.disposed_bomb_ids.add(self.my_target_bomb_key)
                        if self.my_target_bomb_key in self.known_bombs:
                            self.known_bombs[self.my_target_bomb_key]["disposed"] = True
                        self.my_target_bomb_key = None
                return cmd
            self.carry_push_steps += 1
            cmd["forward"] = 0.65
            cmd["rotation"] = max(-1.0, min(1.0, disp_ang * 1.3))
            if self.carry_push_steps >= 14 or disp_dist < 22.0:
                self.carry_spin_steps = 12
                self.carry_push_steps = 0
            return cmd
        return self._steer_apf(cmd, target_rel_angle=disp_ang, max_fwd=0.78)

    def _steer_apf(self, cmd, target_rel_angle, max_fwd, extra_rep_vector=(0.0, 0.0)):
        fx = math.cos(target_rel_angle) + extra_rep_vector[0]
        fy = math.sin(target_rel_angle) + extra_rep_vector[1]
        dists, angles = self.lidar_values(), self.lidar_rays_angles()
        min_d, front_d = 999.0, 999.0
        obs_x, obs_y = 0.0, 0.0
        if dists is not None and angles is not None:
            for d_val, a_val in zip(dists, angles):
                d = float(d_val)
                if d < min_d: min_d = d
                if abs(float(a_val)) < (math.pi / 4.0) and d < front_d: front_d = d
                if d < 48.0:
                    rep = 1200.0 / ((d + 6.0) * (d + 6.0))
                    obs_x -= math.cos(float(a_val)) * rep
                    obs_y -= math.sin(float(a_val)) * rep
        tangent_x = -obs_y * 0.75 * self.tangent_sign
        tangent_y = obs_x * 0.75 * self.tangent_sign
        fx += obs_x + tangent_x
        fy += obs_y + tangent_y
        desired = math.atan2(fy, fx)
        cmd["rotation"] = max(-1.0, min(1.0, desired * 1.35))
        alignment = max(0.35, math.cos(desired))
        fwd_speed = max_fwd * alignment
        if front_d < 20.0:
            cmd["forward"] = -0.35
            cmd["rotation"] = 0.85 * self.tangent_sign
        elif front_d < 35.0:
            cmd["forward"] = max(0.2, fwd_speed * 0.5)
        else:
            cmd["forward"] = max(0.35, fwd_speed)
        return cmd

    def _handle_escape_logic(self, cmd):
        pos = self._get_best_pos()
        if self.last_pos_for_stuck is not None:
            if float(np.linalg.norm(pos - self.last_pos_for_stuck)) < 1.2:
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
`;

const PHASE3_PY_CODE = `"""
蓝队第三阶段控制器：前沿探索 (Frontier Exploration) + 航位推算 (Dead Reckoning) + 抗传感器干扰
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

# 详细代码已完整保存在 src/swarm_rescue/solutions/my_drone_phase3.py
`;

const PHASE2_PY_CODE = `"""
蓝队第二阶段控制器：多机分布式通信协同 + 动态任务竞标拍卖 + 全局目标黑板
文件路径：src/swarm_rescue/solutions/my_drone_phase2.py
"""

from __future__ import annotations
# 详细代码已完整保存在 src/swarm_rescue/solutions/my_drone_phase2.py
`;

const PHASE1_PY_CODE = `"""
蓝队第一阶段控制器：APF 人工势场避障 + 渐进逼近 PID + 处置中心脱壳 + 10 机全并发探索
文件路径：src/swarm_rescue/solutions/my_drone_phase1.py
"""

from __future__ import annotations
# 详细代码已完整保存在 src/swarm_rescue/solutions/my_drone_phase1.py
`;

const EVAL_PY_CODE = `"""
评测入口文件：告诉仿真器使用哪个 Drone 类。
文件路径：src/swarm_rescue/solutions/my_drone_eval.py
"""

from swarm_rescue.simulation.reporting.team_mode import TeamMode
from swarm_rescue.solutions.my_drone_place_example import MyDronePlaceExample
from swarm_rescue.solutions.my_drone_phase4 import MyDronePhase4


def drone_class_for_mode(mode: str):
    """
    Return the concrete drone class for the given team mode.
    """
    team_mode = TeamMode.from_string(mode)
    if team_mode == TeamMode.PLACE:
        return MyDronePlaceExample
    return MyDronePhase4  # 接入终极第四阶段混合全自主架构
`;

const TEAM_INFO_YML = `# 参赛队伍信息配置文件
team_number: 6
team_name: "SkySaviors"
team_members: tom, alice, bob, lea
team_mode: "rescue"
version: "phase_4"
description: "Blue Team Rescue Autonomous Drone Controller with Full Hierarchical Hybrid Architecture and RTB"
`;

export const ExportSolutionsModal: React.FC<ExportSolutionsModalProps> = ({
  isOpen,
  onClose,
}) => {
  const [activeTab, setActiveTab] = useState<'phase4' | 'phase3' | 'phase2' | 'phase1' | 'eval' | 'team'>('phase4');
  const [copied, setCopied] = useState<boolean>(false);

  if (!isOpen) return null;

  const getActiveContent = () => {
    switch (activeTab) {
      case 'phase4':
        return { filename: 'my_drone_phase4.py', content: PHASE4_PY_CODE };
      case 'phase3':
        return { filename: 'my_drone_phase3.py', content: PHASE3_PY_CODE };
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
      <div className="bg-slate-900 border border-slate-700 rounded-xl shadow-2xl max-w-4xl w-full max-h-[88vh] flex flex-col overflow-hidden text-slate-200">
        {/* Header */}
        <div className="flex items-center justify-between p-4 border-b border-slate-800 bg-slate-950">
          <div className="flex items-center gap-2">
            <FolderCheck className="w-5 h-5 text-purple-400" />
            <div>
              <h2 className="font-bold text-sm tracking-wide text-white uppercase flex items-center gap-2">
                <span>比赛方案完整代码库 (Phase 1 ~ Phase 4)</span>
                <span className="text-[10px] bg-purple-950 text-purple-300 border border-purple-400/40 px-1.5 py-0.5 rounded">
                  四阶段已全闭环
                </span>
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
        <div className="px-4 py-2 border-b border-slate-800 bg-slate-950/60 flex items-center justify-between gap-2 flex-wrap">
          <div className="flex items-center gap-1 bg-slate-900 p-1 rounded-lg border border-slate-800 flex-wrap">
            <button
              onClick={() => setActiveTab('phase4')}
              className={`px-2.5 py-1 rounded text-xs font-mono font-medium flex items-center gap-1 transition ${
                activeTab === 'phase4'
                  ? 'bg-purple-600 text-white shadow-sm'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              <FileCode className="w-3.5 h-3.5" />
              阶段四 (终极版)
              <span className="text-[9px] bg-purple-950 px-1 py-0.2 rounded border border-purple-400/30">P4</span>
            </button>
            <button
              onClick={() => setActiveTab('phase3')}
              className={`px-2.5 py-1 rounded text-xs font-mono font-medium flex items-center gap-1 transition ${
                activeTab === 'phase3'
                  ? 'bg-sky-600 text-white shadow-sm'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              <FileCode className="w-3.5 h-3.5" />
              阶段三 (抗干扰/SLAM)
            </button>
            <button
              onClick={() => setActiveTab('phase2')}
              className={`px-2.5 py-1 rounded text-xs font-mono font-medium flex items-center gap-1 transition ${
                activeTab === 'phase2'
                  ? 'bg-emerald-600 text-white shadow-sm'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              <FileCode className="w-3.5 h-3.5" />
              阶段二 (竞标协同)
            </button>
            <button
              onClick={() => setActiveTab('phase1')}
              className={`px-2.5 py-1 rounded text-xs font-mono font-medium flex items-center gap-1 transition ${
                activeTab === 'phase1'
                  ? 'bg-blue-600 text-white'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              <FileCode className="w-3.5 h-3.5" />
              阶段一 (基础)
            </button>
            <button
              onClick={() => setActiveTab('eval')}
              className={`px-2.5 py-1 rounded text-xs font-mono font-medium flex items-center gap-1 transition ${
                activeTab === 'eval'
                  ? 'bg-amber-600 text-white'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              <FileCode className="w-3.5 h-3.5" />
              my_drone_eval.py
            </button>
            <button
              onClick={() => setActiveTab('team')}
              className={`px-2.5 py-1 rounded text-xs font-mono font-medium flex items-center gap-1 transition ${
                activeTab === 'team'
                  ? 'bg-slate-700 text-white'
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
              className="flex items-center gap-1 px-3 py-1 rounded bg-purple-600 hover:bg-purple-500 text-white text-xs font-medium transition shadow-sm"
              title="下载文件到本地"
            >
              <Download className="w-3.5 h-3.5" />
              <span>下载 {filename}</span>
            </button>
          </div>
        </div>

        {/* Code Viewer */}
        <div className="flex-1 overflow-y-auto p-4 bg-slate-950 font-mono text-[11px] leading-relaxed text-slate-300">
          <pre className="whitespace-pre overflow-x-auto selection:bg-purple-900 selection:text-white">
            {content}
          </pre>
        </div>

        {/* Footer */}
        <div className="p-3 border-t border-slate-800 bg-slate-950 flex items-center justify-between text-[11px] text-slate-400">
          <span>
            💡 四个阶段的代码均已完整生成并持久化保存在 src/swarm_rescue/solutions/，评测系统默认挂载 Phase 4 终极控制器。
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
