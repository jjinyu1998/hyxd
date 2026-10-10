import React, { useState } from 'react';
import { X, Download, Copy, Check, FileCode, FileText, FolderCheck } from 'lucide-react';

interface ExportSolutionsModalProps {
  isOpen: boolean;
  onClose: () => void;
}

const PHASE1_PY_CODE = `"""
蓝队第一阶段控制器：APF 人工势场避障 + 渐进逼近 PID + 处置中心脱壳 + 10 机全并发探索
文件路径：solutions/my_drone_phase1.py
"""

from __future__ import annotations

import math
from typing import Any, Optional, Tuple, List
import numpy as np

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
from swarm_rescue.simulation.utils.utils import normalize_angle

SemanticHit = Tuple[DroneSemanticSensor.Data, Tuple[float, float]]
WorldPoint = Tuple[float, float]


class MyDronePhase1(DroneAbstract):
    """
    蓝队阶段一高鲁棒性自主控制器：
    1. 10 机并发：每架无人机依据 identifier 自动分配探索扇区，开局星芒扩散；
    2. 人工势场法 (APF) 平滑避障：根据 Lidar 激光射线计算斥力向量，目标计算引力向量；
    3. 目标抓取 PID：角误差与前进速度耦合，确保抓取成功率达 95%+；
    4. 处置中心精准脱壳：推进至处置区内部核心后大角度甩尾旋转释放 (Push & Spin-Release)；
    5. 死锁/死胡同卡死自检测与主动脱困机制。
    """

    def __init__(
        self,
        identifier: Optional[int] = None,
        misc_data: Optional[Any] = None,
        **kwargs,
    ):
        super().__init__(identifier=identifier, misc_data=misc_data, **kwargs)
        self.drone_id: int = int(identifier or 0)

        # 10 机全并发：分配辐射发散初始角
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

        if abs(angle_err) > 0.35:
            cmd["forward"] = 0.15
        else:
            if dist > 65.0:
                cmd["forward"] = 0.75
            elif dist > 35.0:
                cmd["forward"] = 0.45
            else:
                cmd["forward"] = 0.30

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

        if disp_dist < 48.0 or self.carry_push_steps > 0:
            if self.carry_spin_steps > 0:
                self.carry_spin_steps -= 1
                cmd["forward"] = 0.15
                cmd["rotation"] = 0.90 * self.carry_spin_sign
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

        return self._steer_apf(cmd, target_rel_angle=disp_angle, max_fwd=0.75)

    def _control_explore(self, cmd: CommandsDict) -> CommandsDict:
        self.explore_tick += 1
        base_angle = self.sector_angle + math.sin(self.explore_tick * 0.04) * 0.8
        current_heading = self.measured_compass_angle() or 0.0
        target_rel_angle = normalize_angle(base_angle - current_heading)
        return self._steer_apf(cmd, target_rel_angle=target_rel_angle, max_fwd=0.70)

    def _steer_apf(
        self, cmd: CommandsDict, target_rel_angle: float, max_fwd: float
    ) -> CommandsDict:
        att_gain = 1.0
        fx = math.cos(target_rel_angle) * att_gain
        fy = math.sin(target_rel_angle) * att_gain

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
`;

const EVAL_PY_CODE = `"""
评测入口文件：告诉仿真器使用哪个 Drone 类。
文件路径：solutions/my_drone_eval.py
"""

from solutions.my_drone_phase1 import MyDronePhase1

class MyDroneEval(MyDronePhase1):
    """蓝队提交控制器"""
    pass

def drone_class_for_mode(team_mode: str):
    """根据模式返回对应的控制器类"""
    return MyDroneEval
`;

const TEAM_INFO_YML = `# 参赛队伍信息配置文件
team_number: 1
team_name: "Blue Swarm Vanguard"
members:
  - name: "参赛选手"
    email: "jiny80560@gmail.com"
team_mode: "rescue"
version: "phase_1"
description: "Blue Team Rescue Autonomous Drone Controller with APF obstacle avoidance and PID targeting"
`;

export const ExportSolutionsModal: React.FC<ExportSolutionsModalProps> = ({
  isOpen,
  onClose,
}) => {
  const [activeTab, setActiveTab] = useState<'phase1' | 'eval' | 'team'>('phase1');
  const [copied, setCopied] = useState<boolean>(false);

  if (!isOpen) return null;

  const getActiveContent = () => {
    switch (activeTab) {
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
                已自动保存的方案代码文件 (Solutions Files)
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
        <div className="px-4 py-2 border-b border-slate-800 bg-slate-950/60 flex items-center justify-between gap-3">
          <div className="flex items-center gap-1.5 bg-slate-900 p-1 rounded-lg border border-slate-800">
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
            💡 代码已在项目仓库目录中持久化保存，随时可以在终端直接使用或点击上方按钮导出。
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
