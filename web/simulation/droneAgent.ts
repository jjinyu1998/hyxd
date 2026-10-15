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
  isGrasped: boolean;
  isDisposed: boolean;
};

export type DroneBroadcastPacket = {
  droneId: number;
  position: Vector2;
  graspedBombId: number | null;
  claimedBombId: number | null;
  claimCost: number;
  disposedBombIds: number[];
  sensedBombs: { id: number; position: Vector2; grasped: boolean }[];
};

// 为 10 架无人机分配专属的全图扇区巡航航线（分工明确，全图覆盖西区、中区、东区 100% 盲区）
const DRONE_PATROL_SECTORS: Vector2[][] = [
  // Drone 0: 西区北部 (处置中心外围与西北走廊)
  [ { x: 350, y: 150 }, { x: 380, y: 350 }, { x: 180, y: 220 } ],

  // Drone 1: 西区南部 (西南大厅与门洞 1 西侧走廊)
  [ { x: 220, y: 520 }, { x: 360, y: 700 }, { x: 420, y: 560 } ],

  // Drone 2: 中区南部 (穿过 Door 1 巡航中南大厅)
  [ { x: 650, y: 650 }, { x: 780, y: 680 }, { x: 700, y: 500 } ],

  // Drone 3: 中区中庭 (穿过 Door 1 巡航中枢十字枢纽)
  [ { x: 620, y: 380 }, { x: 760, y: 400 }, { x: 680, y: 520 } ],

  // Drone 4: 中区北部 (穿过 Door 1 巡航中北走廊与 Door 2 前哨)
  [ { x: 620, y: 180 }, { x: 750, y: 200 }, { x: 820, y: 150 } ],

  // Drone 5: 中区东北过渡区 (巡游中区与 Door 2 衔接走廊)
  [ { x: 780, y: 280 }, { x: 680, y: 220 }, { x: 760, y: 140 } ],

  // Drone 6: 东区深处北部 (穿过 Door 1 和 Door 2 直达最东侧东北房间)
  [ { x: 1020, y: 160 }, { x: 1160, y: 180 }, { x: 1100, y: 320 } ],

  // Drone 7: 东区主大厅 (穿过 Door 1 和 Door 2 巡航东侧中央主大厅)
  [ { x: 1050, y: 400 }, { x: 1180, y: 450 }, { x: 1020, y: 540 } ],

  // Drone 8: 东区东南房间 (穿过 Door 1 和 Door 2 巡航最东侧深处房间)
  [ { x: 1020, y: 650 }, { x: 1160, y: 680 }, { x: 1100, y: 520 } ],

  // Drone 9: 东区全景南北往返巡航 (东区大厅至门洞 2 机动接力)
  [ { x: 1140, y: 320 }, { x: 1120, y: 620 }, { x: 980, y: 280 } ],
];

/**
 * 拓扑跨区通道路由器 (Topological Corridor Router)
 * 地图被 Wall 8 (x=510, y: 0~480) 和 Wall 9 (x=910, y: 260~800) 分为三大区域：
 * - 门洞 1：位于 Wall 8 南部通道，通过点为 西侧(420, 620) -> 东侧(590, 620)
 * - 门洞 2：位于 Wall 9 北部通道，通过点为 西侧(830, 150) -> 东侧(990, 150)
 */
export function resolveTopologicalWaypoint(current: Vector2, destination: Vector2): Vector2 {
  const D1_W: Vector2 = { x: 400, y: 620 };
  const D1_E: Vector2 = { x: 620, y: 620 };
  const D2_W: Vector2 = { x: 800, y: 140 };
  const D2_E: Vector2 = { x: 1020, y: 140 };

  const inWest = current.x < 510;
  const inCenter = current.x >= 510 && current.x <= 910;
  const inEast = current.x > 910;

  const destWest = destination.x < 510;
  const destCenter = destination.x >= 510 && destination.x <= 910;
  const destEast = destination.x > 910;

  // 1. 从西区前往中区或东区
  if (inWest) {
    if (destCenter || destEast) {
      // 如果还在 Wall 8 挡墙北侧 (y < 520)，先下潜到南门洞前
      if (current.y < 520) return D1_W;
      // 已经处于门洞开阔带 (y >= 520)，全速径直穿过门洞到中区
      return D1_E;
    }
  }

  // 2. 在中区
  if (inCenter) {
    if (destEast) {
      // 前往东区：若还在 Wall 9 挡墙南侧 (y > 220)，先上浮到北门洞前
      if (current.y > 220) return D2_W;
      // 已经处于门洞开阔带 (y <= 220)，径直穿过门洞到东区
      return D2_E;
    }
    if (destWest) {
      // 回西区：若在北侧先下潜到南门洞前
      if (current.y < 520) return D1_E;
      // 径直穿过门洞到西区
      return D1_W;
    }
  }

  // 3. 从东区回中区或西区
  if (inEast) {
    if (destCenter || destWest) {
      // 若在 Wall 9 挡墙南侧，先上浮到北门洞前
      if (current.y > 220) return D2_E;
      // 径直穿过门洞回到中区
      return D2_W;
    }
  }

  // 同区域内无隔墙阻挡，直达目标
  return destination;
}

/**
 * 阶段四终极全自主多机协同控制器 (10 架无人机全并发全图探索版)
 */
export class DroneAgentController {
  private sectorWaypointIndex: number = 0;
  private waypointTicks: number = 0;

  // 运送与处置脱壳
  private carryPushSteps: number = 0;
  private carrySpinReleaseSteps: number = 0;
  private carrySpinSign: number = 1.0;

  // 卡死与碰撞脱困机制 (彻底杜绝原地打转)
  private stuckCounter: number = 0;
  private escapeSteps: number = 0;
  private escapeDirection: number = 1.0;
  private lastPosition: Vector2 = { x: 0, y: 0 };

  // PID 控制器历史误差
  private prevHeadingError: number = 0;

  // 本机认领锁定的目标炸弹 ID
  public myClaimedBombId: number | null = null;

  // 本地已知的全局目标黑板
  public knownBombs: Map<number, KnownBombRecord> = new Map();
  public disposedBombIds: Set<number> = new Set();
  public knownDisposalCenterPos: Vector2 | null = null;

  constructor(public droneId: number) {
    this.reset();
  }

  public reset() {
    this.sectorWaypointIndex = 0;
    this.waypointTicks = 0;
    this.carryPushSteps = 0;
    this.carrySpinReleaseSteps = 0;
    this.carrySpinSign = this.droneId % 2 === 0 ? 1.0 : -1.0;
    this.stuckCounter = 0;
    this.escapeSteps = 0;
    this.escapeDirection = this.droneId % 2 === 0 ? 1.0 : -1.0;
    this.lastPosition = { x: 0, y: 0 };
    this.prevHeadingError = 0;
    this.myClaimedBombId = null;
    this.knownBombs.clear();
    this.disposedBombIds.clear();
    this.knownDisposalCenterPos = null;
  }

  public getBroadcastPacket(drone: DroneState): DroneBroadcastPacket {
    const sensed = drone.semanticHits
      .filter((h) => h.entityType === 'BOMB')
      .map((h) => ({ id: h.id, position: { ...h.position }, grasped: h.grasped }));

    const claimCost =
      this.myClaimedBombId !== null
        ? (this.knownBombs.get(this.myClaimedBombId)?.claimCost ?? 9999)
        : 9999;

    return {
      droneId: this.droneId,
      position: { ...drone.position },
      graspedBombId: drone.graspedBombId,
      claimedBombId: this.myClaimedBombId,
      claimCost,
      disposedBombIds: Array.from(this.disposedBombIds),
      sensedBombs: sensed,
    };
  }

  public step(
    drone: DroneState,
    teamMode: TeamMode,
    disposalCenter: ZoneRect,
    keyboardInputs?: CommandsDict,
    allDrones?: DroneState[],
    allControllers?: DroneAgentController[]
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
      return this.stepRescue(drone, disposalCenter, allDrones, allControllers);
    } else {
      return this.stepPlace(drone);
    }
  }

  /**
   * 蓝方救援模式状态机核心 (10 架全并发独立任务路由)
   */
  private stepRescue(
    drone: DroneState,
    disposalCenter: ZoneRect,
    allDrones?: DroneState[],
    allControllers?: DroneAgentController[]
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    const cmd: CommandsDict = { forward: 0, lateral: 0, rotation: 0, grasper: 0 };

    // 0. 更新感知与黑板记录
    this.updatePerceptionBlackboard(drone, disposalCenter);

    // 1. 无线电通信多跳同步与分布式任务竞标同步
    if (!drone.isComDisabled && allDrones && allControllers) {
      this.syncMeshCommsAndAuction(drone, allDrones, allControllers);
    }

    // 2. 卡死脱困机制 (防转圈死锁)
    if (this.handleEscapeLogic(cmd, drone)) {
      return { command: cmd, stateMachine: 'explore' };
    }

    // 3. 如果当前已抓取炸弹 -> 运送至处置中心并甩尾入库
    if (drone.graspedBombId !== null) {
      cmd.grasper = 1;
      const disposalCenterPt: Vector2 = {
        x: disposalCenter.x + disposalCenter.width / 2,
        y: disposalCenter.y + disposalCenter.height / 2,
      };

      const distToDisposal = distance(drone.position, disposalCenterPt);

      // 进入处置区核心深处 (50px)
      if (distToDisposal < 50 || this.carryPushSteps > 0) {
        if (this.carrySpinReleaseSteps > 0) {
          this.carrySpinReleaseSteps--;
          cmd.forward = 0.20;
          cmd.rotation = 0.90 * this.carrySpinSign;
          if (this.carrySpinReleaseSteps <= 2) {
            cmd.grasper = 0; // 松爪脱壳入库！
            if (this.myClaimedBombId !== null) {
              this.disposedBombIds.add(this.myClaimedBombId);
              const b = this.knownBombs.get(this.myClaimedBombId);
              if (b) {
                b.isDisposed = true;
                b.isGrasped = false;
              }
              this.myClaimedBombId = null;
            }
          }
          return { command: cmd, stateMachine: 'spin_release' };
        }

        this.carryPushSteps++;
        this.navigateTowards(drone, disposalCenterPt, cmd, 0.65);

        if (this.carryPushSteps >= 14 || distToDisposal < 25) {
          this.carrySpinReleaseSteps = 12;
          this.carrySpinSign = Math.random() > 0.5 ? 1.0 : -1.0;
          this.carryPushSteps = 0;
        }

        return { command: cmd, stateMachine: 'push' };
      }

      // 运送途中：利用拓扑路由器跨区导航回处置中心
      const nextWaypoint = resolveTopologicalWaypoint(drone.position, disposalCenterPt);
      this.navigateTowards(drone, nextWaypoint, cmd, 0.85);
      return { command: cmd, stateMachine: 'carry' };
    }

    // 4. 执行分布式竞标拍卖评估 (一弹一人，互不干扰)
    this.evaluateBidsAndClaim(drone);

    // 5. 如果当前成功竞标认领了炸弹 -> 前往抓取
    if (this.myClaimedBombId !== null) {
      const claimedBomb = this.knownBombs.get(this.myClaimedBombId);
      if (claimedBomb && !claimedBomb.isDisposed && !claimedBomb.isGrasped) {
        // 视野内搜索该炸弹
        const visibleBomb = drone.semanticHits.find(
          (h) => h.entityType === 'BOMB' && h.id === this.myClaimedBombId
        );

        if (visibleBomb) {
          if (visibleBomb.grasped) {
            // 被其他无人机提前抓取，立即放弃该目标
            claimedBomb.isGrasped = true;
            this.myClaimedBombId = null;
          } else {
            // 近身对齐与稳妥抓取
            return this.chaseAndGraspBomb(drone, visibleBomb, cmd);
          }
        } else {
          // 检查是否已经抵达炸弹记录坐标附近但该炸弹已不在（被带走或处置）
          const distToRecord = distance(drone.position, claimedBomb.position);
          if (distToRecord < 45) {
            // 炸弹已不在原处，解除锁定，继续巡航
            claimedBomb.isDisposed = true;
            this.myClaimedBombId = null;
          } else {
            // 远距离：利用拓扑路由器跨区飞往炸弹全局坐标
            const transitWaypoint = resolveTopologicalWaypoint(drone.position, claimedBomb.position);
            this.navigateTowards(drone, transitWaypoint, cmd, 0.85);
            return { command: cmd, stateMachine: 'chase' };
          }
        }
      } else {
        this.myClaimedBombId = null;
      }
    }

    // 6. 检查视野内是否有直接可见的未认领自由炸弹
    const immediateBomb = this.findBestImmediateBomb(drone);
    if (immediateBomb) {
      return this.chaseAndGraspBomb(drone, immediateBomb, cmd);
    }

    // 7. 无特定任务：执行分配给本架无人机的专属扇区全图探索 (10 架各司其职，覆盖全图)
    return this.exploreAssignedSector(drone, cmd, allDrones);
  }

  /**
   * 更新本地黑板
   */
  private updatePerceptionBlackboard(drone: DroneState, disposalCenter: ZoneRect) {
    if (!this.knownDisposalCenterPos) {
      this.knownDisposalCenterPos = {
        x: disposalCenter.x + disposalCenter.width / 2,
        y: disposalCenter.y + disposalCenter.height / 2,
      };
    }

    for (const hit of drone.semanticHits) {
      if (hit.entityType === 'BOMB') {
        const existing = this.knownBombs.get(hit.id);
        if (!existing) {
          this.knownBombs.set(hit.id, {
            id: hit.id,
            position: { ...hit.position },
            claimedByDroneId: null,
            claimCost: 9999,
            isGrasped: hit.grasped,
            isDisposed: false,
          });
        } else {
          existing.position = { ...hit.position };
          existing.isGrasped = hit.grasped;
        }

        if (hit.grasped && hit.id === this.myClaimedBombId && drone.graspedBombId !== hit.id) {
          // 别人抓取了我认领的炸弹，我立刻释放认领
          this.myClaimedBombId = null;
        }
      }
    }
  }

  /**
   * 无线电网络多跳同步与分布式竞标仲裁 (Mesh Communication & Bidding)
   */
  private syncMeshCommsAndAuction(
    drone: DroneState,
    allDrones: DroneState[],
    allControllers: DroneAgentController[]
  ) {
    const COM_RANGE = 350;

    for (let i = 0; i < allDrones.length; i++) {
      const peerDrone = allDrones[i];
      const peerCtrl = allControllers[i];

      if (peerDrone.id === drone.id || peerDrone.health <= 0 || peerDrone.isComDisabled) {
        continue;
      }

      const d = distance(drone.position, peerDrone.position);
      if (d <= COM_RANGE) {
        const packet = peerCtrl.getBroadcastPacket(peerDrone);

        // 1. 同步已处置炸弹
        for (const dispId of packet.disposedBombIds) {
          this.disposedBombIds.add(dispId);
          const b = this.knownBombs.get(dispId);
          if (b) {
            b.isDisposed = true;
            b.isGrasped = false;
          }
          if (this.myClaimedBombId === dispId) {
            this.myClaimedBombId = null;
          }
        }

        // 2. 同步友军正在运送的炸弹
        if (packet.graspedBombId !== null) {
          const gb = this.knownBombs.get(packet.graspedBombId);
          if (gb) {
            gb.isGrasped = true;
            gb.claimedByDroneId = packet.droneId;
          }
          if (this.myClaimedBombId === packet.graspedBombId) {
            this.myClaimedBombId = null;
          }
        }

        // 3. 同步友军视野中发现的炸弹
        for (const sb of packet.sensedBombs) {
          if (this.disposedBombIds.has(sb.id)) continue;

          let b = this.knownBombs.get(sb.id);
          if (!b) {
            b = {
              id: sb.id,
              position: { ...sb.position },
              claimedByDroneId: null,
              claimCost: 9999,
              isGrasped: sb.grasped,
              isDisposed: false,
            };
            this.knownBombs.set(sb.id, b);
          } else {
            b.position = { ...sb.position };
            b.isGrasped = sb.grasped;
          }

          if (sb.grasped && this.myClaimedBombId === sb.id && drone.graspedBombId !== sb.id) {
            this.myClaimedBombId = null;
          }
        }

        // 4. 仲裁竞标认领冲突 (竞标拍卖：距离更近者获胜，等距时 ID 小者获胜)
        if (packet.claimedBombId !== null && !this.disposedBombIds.has(packet.claimedBombId)) {
          const bombRecord = this.knownBombs.get(packet.claimedBombId);
          if (bombRecord) {
            const peerCost = packet.claimCost;
            const myCurrentCost =
              this.myClaimedBombId === packet.claimedBombId
                ? distance(drone.position, bombRecord.position)
                : 9999;

            if (
              bombRecord.claimedByDroneId === null ||
              peerCost < bombRecord.claimCost - 10 ||
              (Math.abs(peerCost - bombRecord.claimCost) <= 10 && packet.droneId < (bombRecord.claimedByDroneId ?? 999))
            ) {
              bombRecord.claimedByDroneId = packet.droneId;
              bombRecord.claimCost = peerCost;
            }

            // 如果我正在认领同一个炸弹，但友军出价更低（距离更近）-> 主动避让！
            if (this.myClaimedBombId === packet.claimedBombId) {
              if (
                peerCost < myCurrentCost - 15 ||
                (Math.abs(peerCost - myCurrentCost) <= 15 && packet.droneId < this.droneId)
              ) {
                this.myClaimedBombId = null; // 礼貌放弃，继续全图探索
              }
            }
          }
        }
      }
    }
  }

  /**
   * 分布式任务竞标仲裁 (Auction Evaluation: 一人一弹，无任务者坚守全图巡航)
   */
  private evaluateBidsAndClaim(drone: DroneState) {
    if (drone.graspedBombId !== null) {
      // 正在运送炸弹的无人机不参与任何新竞标
      return;
    }

    // 检查已认领的目标是否仍然有效
    if (this.myClaimedBombId !== null) {
      const b = this.knownBombs.get(this.myClaimedBombId);
      if (
        b &&
        !b.isDisposed &&
        !b.isGrasped &&
        (b.claimedByDroneId === null || b.claimedByDroneId === this.droneId)
      ) {
        b.claimedByDroneId = this.droneId;
        b.claimCost = distance(drone.position, b.position);
        return;
      }
      this.myClaimedBombId = null;
    }

    // 寻找最近的未被他人胜标的炸弹
    let bestBombId: number | null = null;
    let minCost = 9999;

    for (const [bId, record] of this.knownBombs.entries()) {
      if (record.isDisposed || record.isGrasped || this.disposedBombIds.has(bId)) {
        continue;
      }

      const cost = distance(drone.position, record.position);

      // 如果未被认领，或自己比当前认领者距离明显更近 (胜标优势 > 30px)
      if (record.claimedByDroneId === null || cost < record.claimCost - 30) {
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
    if (drone.graspedBombId !== null) return null;

    const ungraspedBombs = drone.semanticHits.filter(
      (h) => h.entityType === 'BOMB' && !h.grasped && !this.disposedBombIds.has(h.id)
    );
    if (ungraspedBombs.length === 0) return null;

    ungraspedBombs.sort((a, b) => a.distance - b.distance);

    // 优先选择本机已认领的，或尚未被其他人认领的
    for (const b of ungraspedBombs) {
      const record = this.knownBombs.get(b.id);
      if (!record || record.claimedByDroneId === null || record.claimedByDroneId === this.droneId) {
        return b;
      }
    }

    return null;
  }

  private chaseAndGraspBomb(
    drone: DroneState,
    bomb: SemanticHit,
    cmd: CommandsDict
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    const angleErr = bomb.angle;
    const dist = bomb.distance;

    const Kp = 1.6;
    const Kd = 0.35;
    const derivative = angleErr - this.prevHeadingError;
    this.prevHeadingError = angleErr;

    // 角度转向
    cmd.rotation = Math.max(-1.0, Math.min(1.0, angleErr * Kp + derivative * Kd));

    if (Math.abs(angleErr) > 0.40) {
      cmd.forward = 0.25;
    } else {
      if (dist > 65) {
        cmd.forward = 0.85;
      } else if (dist > 35) {
        cmd.forward = 0.50;
      } else {
        cmd.forward = 0.32;
      }
    }

    if (dist <= 38 && Math.abs(angleErr) < 0.45) {
      cmd.grasper = 1;
    }

    return { command: cmd, stateMachine: 'chase' };
  }

  /**
   * 专属扇区巡航探索（10 架无人机分道扬镳，各自覆盖全图专属区域）
   */
  private exploreAssignedSector(
    drone: DroneState,
    cmd: CommandsDict,
    allDrones?: DroneState[]
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    this.waypointTicks++;

    // 获取本机专属扇区的航路点列表
    const mySector = DRONE_PATROL_SECTORS[this.droneId % DRONE_PATROL_SECTORS.length];
    const targetPoint = mySector[this.sectorWaypointIndex % mySector.length];

    const distToTarget = distance(drone.position, targetPoint);

    // 抵达路点附近 (45px) 或停留超时 (180步) -> 轮转到下一个专属航路点
    if (distToTarget < 45 || this.waypointTicks > 180) {
      this.waypointTicks = 0;
      this.sectorWaypointIndex = (this.sectorWaypointIndex + 1) % mySector.length;
    }

    // 利用拓扑通道路由器求取下一跳通畅路点
    const intermediateGoal = resolveTopologicalWaypoint(drone.position, targetPoint);

    // 友军侧向轻微避让力 (只作为 lateral 辅助推力，不干扰航向转角)
    let peerLatPush = 0;
    if (allDrones) {
      for (const peer of allDrones) {
        if (peer.id === drone.id || peer.health <= 0) continue;
        const d = distance(drone.position, peer.position);
        if (d > 10 && d < 65) {
          const heading = drone.angle;
          const toPeerX = peer.position.x - drone.position.x;
          const toPeerY = peer.position.y - drone.position.y;
          // 投影到无人机侧向轴 (lateral axis: -sin(heading), cos(heading))
          const lateralProj = -Math.sin(heading) * toPeerX + Math.cos(heading) * toPeerY;
          const repMag = Math.min(0.35, 120 / (d * d));
          peerLatPush += (lateralProj > 0 ? -1 : 1) * repMag;
        }
      }
    }

    cmd.lateral = Math.max(-0.4, Math.min(0.4, peerLatPush));

    this.navigateTowards(drone, intermediateGoal, cmd, 0.85);
    return { command: cmd, stateMachine: 'explore' };
  }

  /**
   * 清晰通道平滑导航 (防打转防原地盘旋算法)
   */
  private navigateTowards(
    drone: DroneState,
    target: Vector2,
    cmd: CommandsDict,
    maxForward: number = 0.85
  ) {
    const dx = target.x - drone.position.x;
    const dy = target.y - drone.position.y;
    const dist = Math.sqrt(dx * dx + dy * dy);

    // 已经极度接近目标点 (< 25px)：平稳前推，严禁急转打转
    if (dist < 25) {
      cmd.forward = maxForward * 0.45;
      cmd.rotation = 0;
      return;
    }

    const targetAngle = Math.atan2(dy, dx);

    // 从激光雷达中选取无障碍且最接近目标方向的射线航向
    let bestAngle = targetAngle;
    let minCost = Infinity;

    for (const ray of drone.lidarRays) {
      const globalRayAngle = normalizeAngle(drone.angle + ray.angle);
      const angleDiff = Math.abs(normalizeAngle(globalRayAngle - targetAngle));

      let cost = angleDiff;
      if (ray.distance < 28) {
        cost += 45.0 / (ray.distance + 1);
      } else if (ray.distance < 45) {
        cost += 5.0 / (ray.distance + 1);
      }

      if (cost < minCost) {
        minCost = cost;
        bestAngle = globalRayAngle;
      }
    }

    const headingError = normalizeAngle(bestAngle - drone.angle);

    // 距离目标较近时平滑减小旋转增益，消除奇点原地打转
    const rotScale = dist < 50 ? Math.max(0.25, dist / 50) : 1.0;
    cmd.rotation = Math.max(-1.0, Math.min(1.0, headingError * 1.5 * rotScale));

    // 检测正面超近障碍 (雷达检测)
    let minFrontDist = 999;
    for (const ray of drone.lidarRays) {
      if (Math.abs(ray.angle) < 0.35 && ray.distance < minFrontDist) {
        minFrontDist = ray.distance;
      }
    }

    if (minFrontDist < 18) {
      cmd.forward = -0.30;
      cmd.rotation = (headingError >= 0 ? 1 : -1) * 0.8;
    } else {
      // 当航向误差过大时减速转向，但维持基本推力
      const forwardFactor = Math.max(0.40, Math.cos(headingError));
      cmd.forward = maxForward * forwardFactor;
    }
  }

  /**
   * 防卡死与自动解困机制 (卡在墙角或障碍物时主动反向脱困)
   */
  private handleEscapeLogic(cmd: CommandsDict, drone: DroneState): boolean {
    const moved = distance(drone.position, this.lastPosition);
    this.lastPosition = { ...drone.position };

    if (this.escapeSteps > 0) {
      this.escapeSteps--;
      cmd.forward = -0.40;
      cmd.rotation = 0.85 * this.escapeDirection;
      return true;
    }

    // 如果连续 20 步位移低于 0.8px，判定卡死
    if (moved < 0.8 && drone.health > 0) {
      this.stuckCounter++;
      if (this.stuckCounter > 20) {
        this.escapeSteps = 15;
        this.escapeDirection = this.droneId % 2 === 0 ? 1.0 : -1.0;
        this.sectorWaypointIndex++;
        this.waypointTicks = 0;
        this.stuckCounter = 0;
        cmd.forward = -0.40;
        cmd.rotation = 0.85 * this.escapeDirection;
        return true;
      }
    } else {
      this.stuckCounter = Math.max(0, this.stuckCounter - 1);
    }

    return false;
  }

  /**
   * 红方布雷模式
   */
  private stepPlace(
    drone: DroneState
  ): { command: CommandsDict; stateMachine: DroneStateMachine } {
    const cmd: CommandsDict = { forward: 0, lateral: 0, rotation: 0, grasper: 0, placeBomb: false };
    this.waypointTicks++;

    if (drone.carriedBombsCount > 0 && this.waypointTicks % 80 === 0) {
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

    const mySector = DRONE_PATROL_SECTORS[this.droneId % DRONE_PATROL_SECTORS.length];
    const targetPoint = mySector[this.sectorWaypointIndex % mySector.length];
    const waypoint = resolveTopologicalWaypoint(drone.position, targetPoint);
    this.navigateTowards(drone, waypoint, cmd, 0.8);
    return { command: cmd, stateMachine: 'explore' };
  }
}
