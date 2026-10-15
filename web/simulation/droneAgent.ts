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

// 全图 10 大战略探索路点（精确穿过门洞，覆盖各个房间，绝不撞隔墙死锁）
const EXPLORATION_WAYPOINTS: Vector2[] = [
  { x: 380, y: 350 }, // 0: 出生区前方主走廊
  { x: 420, y: 600 }, // 1: 穿向中央大厅的南门
  { x: 650, y: 620 }, // 2: 东南大厅
  { x: 720, y: 350 }, // 3: 中央大厅东侧
  { x: 800, y: 160 }, // 4: 通往远东走廊的北门前
  { x: 1050, y: 150 }, // 5: 穿过北门进入东北房间
  { x: 1080, y: 550 }, // 6: 东南远端房间
  { x: 700, y: 550 }, // 7: 中央南部
  { x: 350, y: 600 }, // 8: 西南大厅
  { x: 250, y: 400 }, // 9: 回到西北通道
];

/**
 * 阶段二：多机分布式通信协同与任务竞标拍卖控制器 (Anti-Spinning & Anti-Clustering Edition)
 */
export class DroneAgentController {
  private waypointIndex: number = 0;
  private waypointTicks: number = 0;

  // 状态机子阶段计时器
  private carryPushSteps: number = 0;
  private carrySpinReleaseSteps: number = 0;
  private carrySpinSign: number = 1.0;

  // 偏向滑墙方向 (左偏或右偏，打破局部极小值死锁)
  private tangentSign: number = 1.0;

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
    this.waypointIndex = this.droneId % EXPLORATION_WAYPOINTS.length;
    this.waypointTicks = 0;
    this.tangentSign = this.droneId % 2 === 0 ? 1.0 : -1.0;
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
   * 蓝方救援模式状态机核心
   */
  private stepRescue(
    drone: DroneState,
    disposalCenter: ZoneRect,
    allDrones?: DroneState[]
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    const cmd: CommandsDict = { forward: 0, lateral: 0, rotation: 0, grasper: 0 };

    // 0. 更新感知与黑板记录
    this.updatePerceptionBlackboard(drone, disposalCenter);

    // 1. 无线电通信多跳同步 (260px)
    if (!drone.isComDisabled && allDrones) {
      this.syncMeshCommsAndAuction(drone, allDrones);
    }

    // 2. 卡死检测与脱困
    this.updateStuckDetection(drone);
    if (this.escapeCounter > 0) {
      this.escapeCounter--;
      cmd.forward = -0.5;
      cmd.rotation = 0.9 * this.escapeDirection;
      return { command: cmd, stateMachine: 'escape' };
    }

    // 3. 如果当前已抓取炸弹 -> 运送至处置中心并甩尾入库
    if (drone.graspedBombId !== null) {
      cmd.grasper = 1;
      const disposalCenterPt: Vector2 = this.knownDisposalCenterPos || {
        x: disposalCenter.x + disposalCenter.width / 2,
        y: disposalCenter.y + disposalCenter.height / 2,
      };

      const distToDisposal = distance(drone.position, disposalCenterPt);

      // 进入处置区核心深处 (48px)
      if (distToDisposal < 48 || this.carryPushSteps > 0) {
        if (this.carrySpinReleaseSteps > 0) {
          this.carrySpinReleaseSteps--;
          cmd.forward = 0.15;
          cmd.rotation = 0.92 * this.carrySpinSign;
          if (this.carrySpinReleaseSteps <= 2) {
            cmd.grasper = 0; // 松爪脱壳！
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

      // 运送途中：切向 APF 导航送往处置中心
      this.steerWithAPF(drone, disposalCenterPt, cmd, 0.78, 45);
      return { command: cmd, stateMachine: 'carry' };
    }

    // 4. 执行分布式竞标拍卖评估
    this.evaluateBidsAndClaim(drone);

    // 5. 如果当前成功竞标认领了炸弹 -> 全速前往抓取
    if (this.myClaimedBombId !== null) {
      const claimedBomb = this.knownBombs.get(this.myClaimedBombId);
      if (claimedBomb && !claimedBomb.isDisposed) {
        // 如果视野内可见该炸弹，启动近身 PD 对齐抓取
        const visibleBomb = drone.semanticHits.find(
          (h) => h.entityType === 'BOMB' && h.id === this.myClaimedBombId && !h.grasped
        );

        if (visibleBomb && visibleBomb.distance < 95) {
          return this.chaseAndGraspBomb(drone, visibleBomb, cmd);
        }

        // 远距离飞往炸弹全局坐标
        return this.approachTargetPos(drone, claimedBomb.position, cmd);
      }
    }

    // 6. 检查视野内是否有直接可见的自由炸弹
    const immediateBomb = this.findBestImmediateBomb(drone);
    if (immediateBomb) {
      return this.chaseAndGraspBomb(drone, immediateBomb, cmd);
    }

    // 7. 无特定任务：全机群房间路点网络自主发散探索 (绝不扎堆、绝不原地打转)
    return this.exploreWaypointsWithAntiClustering(drone, cmd, allDrones);
  }

  /**
   * 更新本地黑板
   */
  private updatePerceptionBlackboard(drone: DroneState, disposalCenter: ZoneRect) {
    if (!this.knownDisposalCenterPos) {
      const dcHit = drone.semanticHits.find((h) => h.entityType === 'DISPOSAL_CENTER');
      if (dcHit) {
        this.knownDisposalCenterPos = {
          x: disposalCenter.x + disposalCenter.width / 2,
          y: disposalCenter.y + disposalCenter.height / 2,
        };
      }
    }

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
   * 无线电网络多跳同步
   */
  private syncMeshCommsAndAuction(drone: DroneState, allDrones: DroneState[]) {
    const COM_RANGE = 260;

    for (const peer of allDrones) {
      if (peer.id === drone.id || peer.health <= 0 || peer.isComDisabled) continue;

      const d = distance(drone.position, peer.position);
      if (d <= COM_RANGE) {
        // 同步处置中心位置
        if (!this.knownDisposalCenterPos && peer.semanticHits.some((h) => h.entityType === 'DISPOSAL_CENTER')) {
          this.knownDisposalCenterPos = {
            x: peer.position.x,
            y: peer.position.y,
          };
        }
      }
    }
  }

  /**
   * 竞标仲裁
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

      if (record.claimedByDroneId === null || cost < record.claimCost - 45) {
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

  private findBestImmediateBomb(drone: DroneState): SemanticHit | null {
    const ungraspedBombs = drone.semanticHits.filter(
      (h) => h.entityType === 'BOMB' && !h.grasped
    );
    if (ungraspedBombs.length === 0) return null;
    ungraspedBombs.sort((a, b) => a.distance - b.distance);
    return ungraspedBombs[0];
  }

  private approachTargetPos(
    drone: DroneState,
    target: Vector2,
    cmd: CommandsDict
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    this.steerWithAPF(drone, target, cmd, 0.80, 45);
    return { command: cmd, stateMachine: 'chase' };
  }

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

    cmd.rotation = Math.max(-1.0, Math.min(1.0, angleErr * Kp + derivative * Kd));

    if (Math.abs(angleErr) > 0.32) {
      cmd.forward = 0.2;
    } else {
      if (dist > 60) {
        cmd.forward = 0.78;
      } else if (dist > 32) {
        cmd.forward = 0.45;
      } else {
        cmd.forward = 0.3;
      }
    }

    if (dist <= 38 && Math.abs(angleErr) < 0.45) {
      cmd.grasper = 1;
    }

    return { command: cmd, stateMachine: 'chase' };
  }

  /**
   * 全局房间路点巡航探索（彻底打破原地盘旋）
   */
  private exploreWaypointsWithAntiClustering(
    drone: DroneState,
    cmd: CommandsDict,
    allDrones?: DroneState[]
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    this.waypointTicks++;

    // 获取本机当前的战略路点
    const currentWaypoint = EXPLORATION_WAYPOINTS[this.waypointIndex];
    const distToWaypoint = distance(drone.position, currentWaypoint);

    // 抵达路点附近 (80px) 或在该路点超时巡航 (>180步) -> 轮转到下一个互斥路点
    if (distToWaypoint < 80 || this.waypointTicks > 180) {
      this.waypointTicks = 0;
      // 步长为 3 轮转，确保相邻无人机不会去同一个目标
      this.waypointIndex = (this.waypointIndex + 3) % EXPLORATION_WAYPOINTS.length;
    }

    let targetPos = EXPLORATION_WAYPOINTS[this.waypointIndex];

    // 如果仍在出生区狭小房间 (x < 300, y < 420)，优先引导冲出房门
    if (drone.position.x < 300 && drone.position.y < 420) {
      targetPos = { x: 400, y: 350 };
    }

    // 计算轻度友军疏散排斥（仅在中等距离生效，避免起点爆炸）
    let fleetRepX = 0;
    let fleetRepY = 0;

    if (allDrones) {
      for (const peer of allDrones) {
        if (peer.id === drone.id || peer.health <= 0) continue;
        const d = distance(drone.position, peer.position);
        if (d > 35 && d < 120) {
          const repMag = Math.min(0.6, 250 / (d * d));
          fleetRepX += ((drone.position.x - peer.position.x) / d) * repMag;
          fleetRepY += ((drone.position.y - peer.position.y) / d) * repMag;
        }
      }
    }

    this.steerWithAPF(drone, targetPos, cmd, 0.78, 48, { x: fleetRepX, y: fleetRepY });
    return { command: cmd, stateMachine: 'explore' };
  }

  /**
   * 人工势场法核心计算（加入切向滑墙与防盘旋自适应前向动力）
   */
  private steerWithAPF(
    drone: DroneState,
    target: Vector2,
    cmd: CommandsDict,
    maxForward: number = 0.75,
    dangerDist: number = 48,
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

    const repGain = 1200.0;
    let minObsDist = 999;
    let frontMinDist = 999;

    let obsRepX = 0;
    let obsRepY = 0;

    for (const ray of drone.lidarRays) {
      if (ray.distance < minObsDist) {
        minObsDist = ray.distance;
      }
      if (Math.abs(ray.angle) < Math.PI / 4 && ray.distance < frontMinDist) {
        frontMinDist = ray.distance;
      }

      if (ray.distance < dangerDist) {
        const rayAngle = normalizeAngle(drone.angle + ray.angle);
        const repMag = repGain / ((ray.distance + 6) * (ray.distance + 6));
        obsRepX -= Math.cos(rayAngle) * repMag;
        obsRepY -= Math.sin(rayAngle) * repMag;
      }
    }

    // 加入切向分量 (90度旋转)：沿墙面切向滑行，彻底消除死锁震荡
    const tangentX = -obsRepY * 0.75 * this.tangentSign;
    const tangentY = obsRepX * 0.75 * this.tangentSign;

    fx += obsRepX + tangentX;
    fy += obsRepY + tangentY;

    const desiredHeading = Math.atan2(fy, fx);
    const headingError = normalizeAngle(desiredHeading - drone.angle);

    // 平滑比例偏航
    cmd.rotation = Math.max(-1.0, Math.min(1.0, headingError * 1.35));

    // 防盘旋：前向速度与角度误差平滑耦合，永不停滞打转
    const alignment = Math.max(0.35, Math.cos(headingError));
    let fwdSpeed = maxForward * alignment;

    // 前方紧贴障碍时，智能小幅倒车脱离死角
    if (frontMinDist < 20) {
      cmd.forward = -0.35;
      cmd.rotation = 0.85 * this.tangentSign;
    } else if (frontMinDist < 35) {
      cmd.forward = Math.max(0.2, fwdSpeed * 0.5);
    } else {
      cmd.forward = Math.max(0.35, fwdSpeed);
    }
  }

  /**
   * 卡死自检测机制
   */
  private updateStuckDetection(drone: DroneState) {
    const moved = distance(drone.position, this.lastPosition);
    this.lastPosition = { ...drone.position };

    if (moved < 1.2 && drone.health > 0) {
      this.stuckCounter++;
      if (this.stuckCounter > 15) {
        this.escapeCounter = 14;
        this.escapeDirection = Math.random() > 0.5 ? 1 : -1;
        this.tangentSign = -this.tangentSign; // 换反向滑墙
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
    this.waypointTicks++;

    if (drone.carriedBombsCount > 0 && this.waypointTicks % 90 === 0) {
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

    const targetPos = EXPLORATION_WAYPOINTS[(this.droneId + 2) % EXPLORATION_WAYPOINTS.length];
    this.steerWithAPF(drone, targetPos, cmd, 0.7, 45);
    return { command: cmd, stateMachine: 'explore' };
  }
}
