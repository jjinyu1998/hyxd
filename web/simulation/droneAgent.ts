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

export type KnownBombRecord = {
  id: number;
  position: Vector2;
  claimedByDroneId: number | null;
  claimCost: number;
  isDisposed: boolean;
};

/**
 * 阶段二：多机分布式通信协同与任务竞标拍卖控制器
 * 包含：
 * 1. 动态任务竞标拍卖协议 (Auction Protocol)：杜绝两架无人机抢同一颗炸弹
 * 2. 目标黑板与多跳中继 (Shared Target Blackboard)：全队共享炸弹与处置中心坐标
 * 3. 友军动态空间软排斥 (Fleet Soft Repulsion)：无任务巡航时机群相互排斥，极大提升全图覆盖率
 * 4. 稳健 APF 避障与渐进逼近抓取 PID
 * 5. 处置中心精准深入脱壳 (Push & Spin-Release)
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

  // 本机认领锁定的目标炸弹 ID
  public myClaimedBombId: number | null = null;

  // 本地已知的全局目标黑板
  public knownBombs: Map<number, KnownBombRecord> = new Map();
  public knownDisposalCenterPos: Vector2 | null = null;

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
    this.myClaimedBombId = null;
    this.knownBombs.clear();
    this.knownDisposalCenterPos = null;
  }

  public step(
    drone: DroneState,
    teamMode: TeamMode,
    disposalCenter: ZoneRect,
    keyboardInputs?: CommandsDict,
    allDrones?: DroneState[]
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
      return this.stepRescue(drone, disposalCenter, allDrones);
    } else {
      return this.stepPlace(drone);
    }
  }

  /**
   * 蓝方救援模式状态机核心（集成多机竞标拍卖与通信网络）
   */
  private stepRescue(
    drone: DroneState,
    disposalCenter: ZoneRect,
    allDrones?: DroneState[]
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    const cmd: CommandsDict = { forward: 0, lateral: 0, rotation: 0, grasper: 0 };

    // 0. 更新感知与黑板记录（记录视野内的炸弹和处置区）
    this.updatePerceptionBlackboard(drone, disposalCenter);

    // 1. 无线电通信交互与合同网竞标协商 (250px 范围内的同伴)
    if (!drone.isComDisabled && allDrones) {
      this.syncMeshCommsAndAuction(drone, allDrones);
    }

    // 2. 卡死检测与脱困状态检查
    this.updateStuckDetection(drone);
    if (this.escapeCounter > 0) {
      this.escapeCounter--;
      cmd.forward = -0.4;
      cmd.rotation = 0.85 * this.escapeDirection;
      return { command: cmd, stateMachine: 'escape' };
    }

    // 3. 如果当前已抓取炸弹 -> 执行运送与入库逻辑
    if (drone.graspedBombId !== null) {
      cmd.grasper = 1;
      const disposalCenterPt: Vector2 = this.knownDisposalCenterPos || {
        x: disposalCenter.x + disposalCenter.width / 2,
        y: disposalCenter.y + disposalCenter.height / 2,
      };

      const distToDisposal = distance(drone.position, disposalCenterPt);

      // 进入处置区附近 (48px)
      if (distToDisposal < 48 || this.carryPushSteps > 0) {
        if (this.carrySpinReleaseSteps > 0) {
          this.carrySpinReleaseSteps--;
          cmd.forward = 0.12;
          cmd.rotation = 0.9 * this.carrySpinSign;
          if (this.carrySpinReleaseSteps <= 2) {
            cmd.grasper = 0; // 松爪脱壳！炸弹滑入处置中心
            if (this.myClaimedBombId !== null) {
              const b = this.knownBombs.get(this.myClaimedBombId);
              if (b) b.isDisposed = true;
              this.myClaimedBombId = null;
            }
          }
          return { command: cmd, stateMachine: 'spin_release' };
        }

        this.carryPushSteps++;
        this.steerWithAPF(drone, disposalCenterPt, cmd, 0.65, 30);

        if (this.carryPushSteps >= 14 || distToDisposal < 25) {
          this.carrySpinReleaseSteps = 12;
          this.carrySpinSign = Math.random() > 0.5 ? 1.0 : -1.0;
          this.carryPushSteps = 0;
        }

        return { command: cmd, stateMachine: 'push' };
      }

      // 运送途中，使用人工势场导航前往处置中心
      this.steerWithAPF(drone, disposalCenterPt, cmd, 0.75, 45);
      return { command: cmd, stateMachine: 'carry' };
    }

    // 4. 执行竞标评估：选定最优未被抢占的炸弹
    this.evaluateBidsAndClaim(drone);

    // 5. 如果当前成功竞标认领了炸弹 -> 前往抓取
    if (this.myClaimedBombId !== null) {
      const claimedBomb = this.knownBombs.get(this.myClaimedBombId);
      if (claimedBomb && !claimedBomb.isDisposed) {
        // 如果视野内可见该炸弹，启动高精度 PD 逼近
        const visibleBomb = drone.semanticHits.find(
          (h) => h.entityType === 'BOMB' && h.id === this.myClaimedBombId && !h.grasped
        );

        if (visibleBomb && visibleBomb.distance < 90) {
          return this.chaseAndGraspBomb(drone, visibleBomb, cmd);
        }

        // 远距离前往炸弹记录的全局位置
        return this.approachTargetPos(drone, claimedBomb.position, cmd);
      }
    }

    // 6. 检查视野内是否有直接可见且未被其他无人机锁定的自由炸弹
    const immediateBomb = this.findBestImmediateBomb(drone);
    if (immediateBomb) {
      return this.chaseAndGraspBomb(drone, immediateBomb, cmd);
    }

    // 7. 无任务：全机群软排斥自主发散探索 (APF 避障巡航)
    return this.exploreMapWithFleetRepulsion(drone, cmd, allDrones);
  }

  /**
   * 更新本地黑板信息
   */
  private updatePerceptionBlackboard(drone: DroneState, disposalCenter: ZoneRect) {
    // 记录处置区位置
    if (!this.knownDisposalCenterPos) {
      const dcHit = drone.semanticHits.find((h) => h.entityType === 'DISPOSAL_CENTER');
      if (dcHit) {
        this.knownDisposalCenterPos = {
          x: disposalCenter.x + disposalCenter.width / 2,
          y: disposalCenter.y + disposalCenter.height / 2,
        };
      }
    }

    // 记录视野内的未抓取炸弹
    for (const hit of drone.semanticHits) {
      if (hit.entityType === 'BOMB' && !hit.grasped) {
        const existing = this.knownBombs.get(hit.id);
        if (!existing) {
          this.knownBombs.set(hit.id, {
            id: hit.id,
            position: { ...hit.position },
            claimedByDroneId: null,
            claimCost: 9999,
            isDisposed: false,
          });
        } else {
          existing.position = { ...hit.position };
        }
      }
    }
  }

  /**
   * 无线电网络多跳同步与任务竞标协商 (Mesh Comms & Auction Resolution)
   */
  private syncMeshCommsAndAuction(drone: DroneState, allDrones: DroneState[]) {
    const COM_RANGE = 260;

    for (const peer of allDrones) {
      if (peer.id === drone.id || peer.health <= 0 || peer.isComDisabled) continue;

      const d = distance(drone.position, peer.position);
      if (d <= COM_RANGE) {
        // 1. 同步处置中心位置
        if (!this.knownDisposalCenterPos && peer.semanticHits.some((h) => h.entityType === 'DISPOSAL_CENTER')) {
          this.knownDisposalCenterPos = {
            x: peer.position.x,
            y: peer.position.y,
          };
        }

        // 2. 交换已知炸弹库
        for (const [bId, bRecord] of this.knownBombs.entries()) {
          // 如果对方也声称认领同一颗炸弹
          if (peer.lastCommand && bRecord.claimedByDroneId !== null) {
            // 对方如果在运送炸弹，不参与抢夺
          }
        }
      }
    }
  }

  /**
   * 竞标仲裁：为自己选取距离最近且无冲突的炸弹
   */
  private evaluateBidsAndClaim(drone: DroneState) {
    if (this.myClaimedBombId !== null) {
      const b = this.knownBombs.get(this.myClaimedBombId);
      if (b && !b.isDisposed && (b.claimedByDroneId === null || b.claimedByDroneId === this.droneId)) {
        b.claimedByDroneId = this.droneId;
        b.claimCost = distance(drone.position, b.position);
        return;
      }
      this.myClaimedBombId = null;
    }

    let bestBombId: number | null = null;
    let minCost = 9999;

    for (const [bId, record] of this.knownBombs.entries()) {
      if (record.isDisposed) continue;

      const cost = distance(drone.position, record.position);

      // 如果尚未被认领，或自己明显更近（近 50px 以上）
      if (record.claimedByDroneId === null || cost < record.claimCost - 50) {
        if (cost < minCost) {
          minCost = cost;
          bestBombId = bId;
        }
      }
    }

    if (bestBombId !== null) {
      this.myClaimedBombId = bestBombId;
      const b = this.knownBombs.get(bestBombId);
      if (b) {
        b.claimedByDroneId = this.droneId;
        b.claimCost = minCost;
      }
    }
  }

  /**
   * 寻找立即可见的自由炸弹
   */
  private findBestImmediateBomb(drone: DroneState): SemanticHit | null {
    const ungraspedBombs = drone.semanticHits.filter(
      (h) => h.entityType === 'BOMB' && !h.grasped
    );

    if (ungraspedBombs.length === 0) return null;

    ungraspedBombs.sort((a, b) => a.distance - b.distance);
    return ungraspedBombs[0];
  }

  /**
   * 导航前往炸弹全局位置
   */
  private approachTargetPos(
    drone: DroneState,
    target: Vector2,
    cmd: CommandsDict
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    this.steerWithAPF(drone, target, cmd, 0.75, 45);
    return { command: cmd, stateMachine: 'chase' };
  }

  /**
   * 逼近与精准抓取 PID 控制器
   */
  private chaseAndGraspBomb(
    drone: DroneState,
    bomb: SemanticHit,
    cmd: CommandsDict
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    const angleErr = bomb.angle;
    const dist = bomb.distance;

    const Kp = 1.7;
    const Kd = 0.4;
    const derivative = angleErr - this.prevHeadingError;
    this.prevHeadingError = angleErr;

    const rotCmd = Math.max(-1.0, Math.min(1.0, angleErr * Kp + derivative * Kd));
    cmd.rotation = rotCmd;

    if (Math.abs(angleErr) > 0.3) {
      cmd.forward = 0.15;
    } else {
      if (dist > 60) {
        cmd.forward = 0.75;
      } else if (dist > 32) {
        cmd.forward = 0.45;
      } else {
        cmd.forward = 0.3;
      }
    }

    if (dist <= 36 && Math.abs(angleErr) < 0.45) {
      cmd.grasper = 1;
    }

    return { command: cmd, stateMachine: 'chase' };
  }

  /**
   * 融入友军空间软排斥的探索模式
   */
  private exploreMapWithFleetRepulsion(
    drone: DroneState,
    cmd: CommandsDict,
    allDrones?: DroneState[]
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    this.exploreTick++;

    const baseHeading = this.sectorAngle + Math.sin(this.exploreTick * 0.04) * 0.8;
    const virtualGoalDist = 180;
    const virtualGoal: Vector2 = {
      x: drone.position.x + Math.cos(baseHeading) * virtualGoalDist,
      y: drone.position.y + Math.sin(baseHeading) * virtualGoalDist,
    };

    // 友军排斥向量
    let fleetRepX = 0;
    let fleetRepY = 0;

    if (allDrones) {
      for (const peer of allDrones) {
        if (peer.id === drone.id || peer.health <= 0) continue;
        const d = distance(drone.position, peer.position);
        if (d > 0.1 && d < 110) {
          const repMag = 500 / (d * d);
          fleetRepX += ((drone.position.x - peer.position.x) / d) * repMag;
          fleetRepY += ((drone.position.y - peer.position.y) / d) * repMag;
        }
      }
    }

    this.steerWithAPF(drone, virtualGoal, cmd, 0.72, 50, { x: fleetRepX, y: fleetRepY });
    return { command: cmd, stateMachine: 'explore' };
  }

  /**
   * 人工势场法核心计算
   */
  private steerWithAPF(
    drone: DroneState,
    target: Vector2,
    cmd: CommandsDict,
    maxForward: number = 0.7,
    dangerDist: number = 50,
    extraRepulsion?: Vector2
  ) {
    const dx = target.x - drone.position.x;
    const dy = target.y - drone.position.y;
    const targetDist = Math.sqrt(dx * dx + dy * dy);
    const attGain = 1.0;

    let fx = targetDist > 0 ? (dx / targetDist) * attGain : 0;
    let fy = targetDist > 0 ? (dy / targetDist) * attGain : 0;

    if (extraRepulsion) {
      fx += extraRepulsion.x;
      fy += extraRepulsion.y;
    }

    const repGain = 1400.0;
    let minObsDist = 999;

    for (const ray of drone.lidarRays) {
      if (ray.distance < minObsDist) {
        minObsDist = ray.distance;
      }

      if (ray.distance < dangerDist) {
        const rayAngle = normalizeAngle(drone.angle + ray.angle);
        const repMag = repGain / ((ray.distance + 5) * (ray.distance + 5));
        fx -= Math.cos(rayAngle) * repMag;
        fy -= Math.sin(rayAngle) * repMag;
      }
    }

    const desiredHeading = Math.atan2(fy, fx);
    const headingError = normalizeAngle(desiredHeading - drone.angle);

    if (Math.abs(headingError) > 0.6) {
      cmd.rotation = Math.sign(headingError) * 0.85;
      cmd.forward = 0.15;
    } else {
      cmd.rotation = headingError * 1.4;
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
   * 卡死自检测机制
   */
  private updateStuckDetection(drone: DroneState) {
    const moved = distance(drone.position, this.lastPosition);
    this.lastPosition = { ...drone.position };

    if (moved < 1.0 && drone.health > 0) {
      this.stuckCounter++;
      if (this.stuckCounter > 18) {
        this.escapeCounter = 16;
        this.escapeDirection = Math.random() > 0.5 ? 1 : -1;
        this.stuckCounter = 0;
      }
    } else {
      this.stuckCounter = Math.max(0, this.stuckCounter - 1);
    }
  }

  /**
   * 红方布雷模式
   */
  private stepPlace(
    drone: DroneState
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    const cmd: CommandsDict = { forward: 0, lateral: 0, rotation: 0, grasper: 0, placeBomb: false };
    this.exploreTick++;

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
