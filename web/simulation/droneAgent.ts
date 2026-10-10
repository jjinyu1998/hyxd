import {
  CommandsDict,
  DroneState,
  DroneStateMachine,
  ZoneRect,
  Vector2,
  TeamMode,
  SemanticHit,
} from '../types/simulation';
import { normalizeAngle, distance } from './physics';

/**
 * 阶段一：高鲁棒性单机运动控制与状态机
 * 包含：
 * 1. 人工势场法 (APF) 平滑避障与局部极小值（死胡同）脱困
 * 2. 目标逼近 PID 航向对齐与速度耦合（高成功率抓取）
 * 3. 处置中心精准深入运送与甩尾脱壳（Push & Spin-Release）
 * 4. 10 机全并发自主行动（扇区辐射探索）
 */
export class DroneAgentController {
  private exploreTick: number = 0;
  private sectorAngle: number = 0;

  // 状态机子阶段计时器
  private carryPushSteps: number = 0;
  private carrySpinReleaseSteps: number = 0;
  private carrySpinSign: number = 1.0;

  // 死锁 / 卡死检测
  private stuckCounter: number = 0;
  private lastPosition: Vector2 = { x: 0, y: 0 };
  private escapeCounter: number = 0;
  private escapeDirection: number = 1;

  // PID 控制器历史误差
  private prevHeadingError: number = 0;

  constructor(public droneId: number) {
    this.reset();
  }

  public reset() {
    this.exploreTick = Math.floor(Math.random() * 80);
    // 10 架无人机初始分散朝向不同的扇区辐射开
    this.sectorAngle = (this.droneId * (2 * Math.PI)) / 10;
    this.carryPushSteps = 0;
    this.carrySpinReleaseSteps = 0;
    this.carrySpinSign = this.droneId % 2 === 0 ? 1.0 : -1.0;
    this.stuckCounter = 0;
    this.escapeCounter = 0;
    this.lastPosition = { x: 0, y: 0 };
    this.prevHeadingError = 0;
  }

  public step(
    drone: DroneState,
    teamMode: TeamMode,
    disposalCenter: ZoneRect,
    keyboardInputs?: CommandsDict
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    if (drone.health <= 0) {
      return {
        command: { forward: 0, lateral: 0, rotation: 0, grasper: 0 },
        stateMachine: 'dead',
      };
    }

    if (teamMode === 'manual') {
      const cmd: CommandsDict = keyboardInputs || {
        forward: 0,
        lateral: 0,
        rotation: 0,
        grasper: 0,
      };
      return { command: cmd, stateMachine: 'manual' };
    }

    if (teamMode === 'rescue') {
      return this.stepRescue(drone, disposalCenter);
    } else {
      return this.stepPlace(drone);
    }
  }

  /**
   * 蓝方救援模式状态机核心
   */
  private stepRescue(
    drone: DroneState,
    disposalCenter: ZoneRect
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    const cmd: CommandsDict = { forward: 0, lateral: 0, rotation: 0, grasper: 0 };

    // 0. 卡死检测与脱困状态检查
    this.updateStuckDetection(drone);
    if (this.escapeCounter > 0) {
      this.escapeCounter--;
      cmd.forward = -0.4;
      cmd.rotation = 0.85 * this.escapeDirection;
      return { command: cmd, stateMachine: 'escape' };
    }

    // 1. 如果已抓取炸弹 -> 执行运送与入库逻辑
    if (drone.graspedBombId !== null) {
      cmd.grasper = 1;
      const disposalCenterPt: Vector2 = {
        x: disposalCenter.x + disposalCenter.width / 2,
        y: disposalCenter.y + disposalCenter.height / 2,
      };

      const distToDisposal = distance(drone.position, disposalCenterPt);

      // 进入处置区附近 (48px)
      if (distToDisposal < 48 || this.carryPushSteps > 0) {
        // 如果正在执行甩尾释放
        if (this.carrySpinReleaseSteps > 0) {
          this.carrySpinReleaseSteps--;
          cmd.forward = 0.1;
          cmd.rotation = 0.9 * this.carrySpinSign;
          if (this.carrySpinReleaseSteps <= 2) {
            cmd.grasper = 0; // 松爪脱壳！炸弹滑入处置中心
          }
          return { command: cmd, stateMachine: 'spin_release' };
        }

        // 推入处置中心核心深处
        this.carryPushSteps++;
        this.steerWithAPF(drone, disposalCenterPt, cmd, 0.65, 30);

        if (this.carryPushSteps >= 14 || distToDisposal < 25) {
          // 深入足够深度后，开始快速甩尾脱壳
          this.carrySpinReleaseSteps = 12;
          this.carrySpinSign = Math.random() > 0.5 ? 1.0 : -1.0;
          this.carryPushSteps = 0;
        }

        return { command: cmd, stateMachine: 'push' };
      }

      // 在运送途中，使用人工势场导航前往处置中心
      this.steerWithAPF(drone, disposalCenterPt, cmd, 0.75, 45);
      return { command: cmd, stateMachine: 'carry' };
    }

    // 2. 检查是否有视野内的未抓取炸弹 (语义雷达)
    const targetBomb = this.findBestTargetBomb(drone);

    if (targetBomb) {
      this.carryPushSteps = 0;
      this.carrySpinReleaseSteps = 0;

      // 执行近身逼近 PID 控制
      return this.chaseAndGraspBomb(drone, targetBomb, cmd);
    }

    // 3. 无目标炸弹 -> 全机群发散探索模式 (APF 避障巡航)
    return this.exploreMap(drone, cmd);
  }

  /**
   * 筛选最合适的近身炸弹
   */
  private findBestTargetBomb(drone: DroneState): SemanticHit | null {
    const ungraspedBombs = drone.semanticHits.filter(
      (h) => h.entityType === 'BOMB' && !h.grasped
    );

    if (ungraspedBombs.length === 0) return null;

    // 优先选择最近且前方夹角较小的炸弹
    ungraspedBombs.sort((a, b) => {
      const scoreA = a.distance + Math.abs(a.angle) * 30;
      const scoreB = b.distance + Math.abs(b.angle) * 30;
      return scoreA - scoreB;
    });

    return ungraspedBombs[0];
  }

  /**
   * 逼近与精准抓取 PID 控制器
   */
  private chaseAndGraspBomb(
    drone: DroneState,
    bomb: SemanticHit,
    cmd: CommandsDict
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    const angleErr = bomb.angle; // 相对角度 [-pi, pi]
    const dist = bomb.distance;

    // 比例-微分 PID 航向控制
    const Kp = 1.6;
    const Kd = 0.4;
    const derivative = angleErr - this.prevHeadingError;
    this.prevHeadingError = angleErr;

    const rotCmd = Math.max(-1.0, Math.min(1.0, angleErr * Kp + derivative * Kd));
    cmd.rotation = rotCmd;

    // 速度与航向误差耦合：航向误差大时减速对正，对正后平滑前冲
    if (Math.abs(angleErr) > 0.3) {
      cmd.forward = 0.15; // 减速转向
    } else {
      // 距离适中时全速接近，极近时保持稳定推力
      if (dist > 60) {
        cmd.forward = 0.75;
      } else if (dist > 32) {
        cmd.forward = 0.45;
      } else {
        cmd.forward = 0.3;
      }
    }

    // 抓爪开合逻辑：当距离小于 36px 且基本对正前向时，激活抓爪咬合！
    if (dist <= 36 && Math.abs(angleErr) < 0.45) {
      cmd.grasper = 1;
    }

    return { command: cmd, stateMachine: 'chase' };
  }

  /**
   * 基于人工势场 (APF) 的全域发散探索
   */
  private exploreMap(
    drone: DroneState,
    cmd: CommandsDict
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    this.exploreTick++;

    // 周期性调整航向，配合自身的扇区基准角
    const baseHeading = this.sectorAngle + Math.sin(this.exploreTick * 0.04) * 0.8;
    const virtualGoalDist = 180;
    const virtualGoal: Vector2 = {
      x: drone.position.x + Math.cos(baseHeading) * virtualGoalDist,
      y: drone.position.y + Math.sin(baseHeading) * virtualGoalDist,
    };

    // 使用 APF 进行平滑避障与巡航
    this.steerWithAPF(drone, virtualGoal, cmd, 0.7, 50);

    return { command: cmd, stateMachine: 'explore' };
  }

  /**
   * 人工势场法 (APF) 核心计算函数
   * 引力场：指向 target
   * 斥力场：所有 Lidar 射线检测到的近距离墙体产生反向斥力
   */
  private steerWithAPF(
    drone: DroneState,
    target: Vector2,
    cmd: CommandsDict,
    maxForward: number = 0.7,
    dangerDist: number = 50
  ) {
    // 1. 引力向量 (Attractive Force)
    const dx = target.x - drone.position.x;
    const dy = target.y - drone.position.y;
    const targetDist = Math.sqrt(dx * dx + dy * dy);
    const attGain = 1.0;

    let fx = targetDist > 0 ? (dx / targetDist) * attGain : 0;
    let fy = targetDist > 0 ? (dy / targetDist) * attGain : 0;

    // 2. 斥力向量 (Repulsive Force from Lidar rays)
    const repGain = 1400.0;
    let minObsDist = 999;

    for (const ray of drone.lidarRays) {
      if (ray.distance < minObsDist) {
        minObsDist = ray.distance;
      }

      if (ray.distance < dangerDist) {
        // 射线方向的绝对全局角度
        const rayAngle = normalizeAngle(drone.angle + ray.angle);
        // 距离越近，排斥力按平方反比急剧增大
        const repMag = repGain / ((ray.distance + 5) * (ray.distance + 5));
        fx -= Math.cos(rayAngle) * repMag;
        fy -= Math.sin(rayAngle) * repMag;
      }
    }

    // 3. 合成期望航向角
    const desiredHeading = Math.atan2(fy, fx);
    const headingError = normalizeAngle(desiredHeading - drone.angle);

    // 4. 计算旋转与前向动力
    if (Math.abs(headingError) > 0.6) {
      // 角度偏差较大时，主要转向，小油门
      cmd.rotation = Math.sign(headingError) * 0.85;
      cmd.forward = 0.15;
    } else {
      cmd.rotation = headingError * 1.4;
      // 若极度靠近障碍物则减速，开阔区全速前冲
      if (minObsDist < 25) {
        cmd.forward = 0.1;
      } else if (minObsDist < 40) {
        cmd.forward = maxForward * 0.5;
      } else {
        cmd.forward = maxForward;
      }
    }
  }

  /**
   * 卡死/震荡自检测机制
   */
  private updateStuckDetection(drone: DroneState) {
    const moved = distance(drone.position, this.lastPosition);
    this.lastPosition = { ...drone.position };

    // 如果连续 18 个 timestep 几乎没有移动且油门不为 0
    if (moved < 1.0 && drone.health > 0) {
      this.stuckCounter++;
      if (this.stuckCounter > 18) {
        // 触发脱困动作：后退并顺时针/逆时针大角度旋转
        this.escapeCounter = 16;
        this.escapeDirection = Math.random() > 0.5 ? 1 : -1;
        this.stuckCounter = 0;
      }
    } else {
      this.stuckCounter = Math.max(0, this.stuckCounter - 1);
    }
  }

  /**
   * 红方布雷模式 (保持稳健巡航与布雷)
   */
  private stepPlace(
    drone: DroneState
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    const cmd: CommandsDict = { forward: 0, lateral: 0, rotation: 0, grasper: 0, placeBomb: false };
    this.exploreTick++;

    // 周期性在开阔区域布雷
    if (drone.carriedBombsCount > 0 && this.exploreTick % 100 === 0) {
      let frontClear = true;
      for (const r of drone.lidarRays) {
        if (Math.abs(r.angle) < Math.PI / 4 && r.distance < 45) {
          frontClear = false;
          break;
        }
      }
      if (frontClear) {
        cmd.placeBomb = true;
      }
    }

    const baseHeading = this.sectorAngle + Math.sin(this.exploreTick * 0.05) * 0.7;
    const virtualGoal: Vector2 = {
      x: drone.position.x + Math.cos(baseHeading) * 150,
      y: drone.position.y + Math.sin(baseHeading) * 150,
    };

    this.steerWithAPF(drone, virtualGoal, cmd, 0.65, 45);
    return { command: cmd, stateMachine: 'explore' };
  }
}
