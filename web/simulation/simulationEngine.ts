import {
  MapDefinition,
  DroneState,
  BombState,
  TeamMode,
  SimulationScores,
  DroneMessage,
  Vector2,
  CommandsDict,
} from '../types/simulation';
import {
  distance,
  distanceSquared,
  normalizeAngle,
  resolveCircleWallCollision,
  isPointInRect,
} from './physics';
import {
  computeLidar,
  computeSemanticSensor,
  computeGps,
  isComDisabled,
} from './sensors';
import { DroneAgentController } from './droneAgent';

const DRONE_COLORS = [
  '#38bdf8', // sky-400 (Drone 0 - Leader)
  '#a855f7', // purple-500
  '#f59e0b', // amber-500
  '#ec4899', // pink-500
  '#10b981', // emerald-500
  '#06b6d4', // cyan-500
  '#8b5cf6', // violet-500
  '#f97316', // orange-500
  '#14b8a6', // teal-500
  '#6366f1', // indigo-500
];

export class SimulationEngine {
  public map: MapDefinition;
  public teamMode: TeamMode;
  public drones: DroneState[] = [];
  public bombs: BombState[] = [];
  public controllers: DroneAgentController[] = [];
  public messages: DroneMessage[] = [];
  public timestep: number = 0;
  public maxTimesteps: number = 3000;
  public isRunning: boolean = false;
  public simulationSpeed: number = 1;
  public selectedDroneId: number = 0;
  public manualInputs: CommandsDict = { forward: 0, lateral: 0, rotation: 0, grasper: 0, placeBomb: false };

  // Explored grid for SLAM mapping (binary / probability grid)
  public gridCellSize: number = 10;
  public gridCols: number = 0;
  public gridRows: number = 0;
  public explorationGrid: Uint8Array = new Uint8Array(0); // 0: unknown, 1: free, 2: wall

  constructor(map: MapDefinition, teamMode: TeamMode = 'rescue', droneCount: number = 10) {
    this.map = map;
    this.teamMode = teamMode;
    this.initGrid();
    this.reset(droneCount);
  }

  private initGrid() {
    this.gridCols = Math.ceil(this.map.width / this.gridCellSize);
    this.gridRows = Math.ceil(this.map.height / this.gridCellSize);
    this.explorationGrid = new Uint8Array(this.gridCols * this.gridRows);
  }

  public reset(droneCount: number = 10) {
    this.timestep = 0;
    this.isRunning = false;
    this.messages = [];
    this.explorationGrid.fill(0);

    // 1. Initialize Drones inside start area
    this.drones = [];
    this.controllers = [];

    const startX = this.map.startArea.x + 20;
    const startY = this.map.startArea.y + 20;
    const perRow = Math.min(5, Math.ceil(Math.sqrt(droneCount)));
    const spacing = 32;

    for (let i = 0; i < droneCount; i++) {
      const col = i % perRow;
      const row = Math.floor(i / perRow);
      const pos: Vector2 = {
        x: startX + col * spacing,
        y: startY + row * spacing,
      };

      const drone: DroneState = {
        id: i,
        position: pos,
        velocity: { x: 0, y: 0 },
        angle: 0, // facing right
        angularVelocity: 0,
        health: 100,
        graspedBombId: null,
        carriedBombsCount: this.teamMode === 'place' ? 2 : 0,
        stateMachine: 'idle',
        lidarRays: [],
        semanticHits: [],
        gpsPosition: pos,
        compassAngle: 0,
        speed: 0,
        isGpsDisabled: false,
        isComDisabled: false,
        lastCommand: { forward: 0, lateral: 0, rotation: 0, grasper: 0 },
        trail: [pos],
        color: DRONE_COLORS[i % DRONE_COLORS.length],
      };

      this.drones.push(drone);
      this.controllers.push(new DroneAgentController(i));
    }

    // 2. Initialize Bombs
    this.bombs = [];
    let bombId = 1;

    if (this.teamMode === 'rescue' || this.teamMode === 'manual') {
      for (const bPos of this.map.defaultBombs) {
        this.bombs.push({
          id: bombId++,
          position: { ...bPos },
          velocity: { x: 0, y: 0 },
          isGrasped: false,
          graspedByDroneId: null,
          isDisposed: false,
          isPlacedByRed: false,
        });
      }
    }

    // Initial sensor update
    this.updatePerceptions();

    this.addMessage({
      senderId: 0,
      timestep: 0,
      text: `Simulation initialized with ${droneCount} drones on ${this.map.name} (${this.teamMode.toUpperCase()} mode).`,
      type: 'info',
    });
  }

  public setMap(newMap: MapDefinition) {
    this.map = newMap;
    this.initGrid();
    this.reset(this.drones.length || 10);
  }

  public setTeamMode(mode: TeamMode) {
    this.teamMode = mode;
    this.reset(this.drones.length || 10);
  }

  public addMessage(msg: Omit<DroneMessage, 'id'>) {
    const id = `${Date.now()}-${Math.random().toString(36).substr(2, 5)}`;
    this.messages.unshift({ id, ...msg });
    if (this.messages.length > 50) {
      this.messages.pop();
    }
  }

  // Update sensor readings for all drones
  private updatePerceptions() {
    for (const drone of this.drones) {
      if (drone.health <= 0) continue;

      // 1. Lidar
      drone.lidarRays = computeLidar(
        drone.position,
        drone.angle,
        this.map.walls,
        28,
        320
      );

      // 2. Semantic Sensor
      drone.semanticHits = computeSemanticSensor(
        drone,
        this.bombs,
        this.map.disposalCenter,
        this.map.returnArea,
        this.drones,
        this.map.walls,
        280
      );

      // 3. GPS
      const gpsResult = computeGps(drone.position, this.map.disablerZones);
      drone.gpsPosition = gpsResult.position;
      drone.isGpsDisabled = gpsResult.disabled;

      // 4. Com jamming check
      const prevComDisabled = drone.isComDisabled;
      drone.isComDisabled = isComDisabled(drone.position, this.map.disablerZones);

      if (!prevComDisabled && drone.isComDisabled && this.timestep % 150 === 0) {
        this.addMessage({
          senderId: drone.id,
          timestep: this.timestep,
          text: `Drone #${drone.id} radio jammed in EM interference zone.`,
          type: 'jammed',
          position: drone.position,
        });
      }

      // 5. Compass
      drone.compassAngle = drone.angle;

      // 6. Update Exploration Grid using Lidar
      this.updateExploredCells(drone);
    }
  }

  // Update occupancy grid based on drone position and lidar rays
  private updateExploredCells(drone: DroneState) {
    const dCol = Math.floor(drone.position.x / this.gridCellSize);
    const dRow = Math.floor(drone.position.y / this.gridCellSize);

    // Free space immediately around drone
    for (let r = -2; r <= 2; r++) {
      for (let c = -2; c <= 2; c++) {
        const col = dCol + c;
        const row = dRow + r;
        if (col >= 0 && col < this.gridCols && row >= 0 && row < this.gridRows) {
          const idx = row * this.gridCols + col;
          if (this.explorationGrid[idx] === 0) {
            this.explorationGrid[idx] = 1; // free
          }
        }
      }
    }

    // Trace rays
    for (const ray of drone.lidarRays) {
      const hitCol = Math.floor(ray.hitPoint.x / this.gridCellSize);
      const hitRow = Math.floor(ray.hitPoint.y / this.gridCellSize);

      if (hitCol >= 0 && hitCol < this.gridCols && hitRow >= 0 && hitRow < this.gridRows) {
        const hitIdx = hitRow * this.gridCols + hitCol;
        if (ray.distance < 315) {
          this.explorationGrid[hitIdx] = 2; // wall hit
        }
      }
    }
  }

  // Single simulation step
  public step() {
    this.timestep++;

    // 1. Perception
    this.updatePerceptions();

    // 2. Decision & Control
    for (let i = 0; i < this.drones.length; i++) {
      const drone = this.drones[i];
      const controller = this.controllers[i];

      const inputs = drone.id === this.selectedDroneId ? this.manualInputs : undefined;
      const prevClaim = controller.myClaimedBombId;
      const { command, stateMachine } = controller.step(
        drone,
        this.teamMode,
        this.map.disposalCenter,
        inputs,
        this.drones,
        this.controllers
      );

      // 竞标成功日志
      if (controller.myClaimedBombId !== null && controller.myClaimedBombId !== prevClaim) {
        this.addMessage({
          senderId: drone.id,
          timestep: this.timestep,
          text: `🤝 竞标认领成功：Drone #${drone.id} 赢得 Bomb #${controller.myClaimedBombId}，全速前往抓取！`,
          type: 'info',
          position: drone.position,
        });
      }

      drone.lastCommand = command;
      drone.stateMachine = stateMachine;

      // Handle Red team Bomb Placement
      if (command.placeBomb && drone.carriedBombsCount > 0) {
        drone.carriedBombsCount--;
        const newBombId = this.bombs.length + 1;
        this.bombs.push({
          id: newBombId,
          position: { ...drone.position },
          velocity: { x: 0, y: 0 },
          isGrasped: false,
          graspedByDroneId: null,
          isDisposed: false,
          isPlacedByRed: true,
          blastRadius: Math.min(this.map.width, this.map.height) * 0.12,
        });

        this.addMessage({
          senderId: drone.id,
          timestep: this.timestep,
          text: `Drone #${drone.id} deployed tactical bomb #${newBombId} at (${Math.round(drone.position.x)}, ${Math.round(drone.position.y)}).`,
          type: 'info',
          position: drone.position,
        });
      }
    }

    // 3. Physics & Kinematics Update
    const DT = 0.5;
    const DRAG = 0.92;
    const ANGULAR_DRAG = 0.50;
    const MAX_SPEED = 4.2;

    for (const drone of this.drones) {
      if (drone.health <= 0) continue;

      const cmd = drone.lastCommand;

      // Angular acceleration with responsive damping (prevents overshooting and continuous spinning)
      drone.angularVelocity = drone.angularVelocity * ANGULAR_DRAG + cmd.rotation * 0.14;
      drone.angle = normalizeAngle(drone.angle + drone.angularVelocity * DT);

      // Linear acceleration along drone's local coordinate system
      const forwardX = Math.cos(drone.angle) * cmd.forward * 0.7;
      const forwardY = Math.sin(drone.angle) * cmd.forward * 0.7;
      const lateralX = -Math.sin(drone.angle) * cmd.lateral * 0.4;
      const lateralY = Math.cos(drone.angle) * cmd.lateral * 0.4;

      drone.velocity.x = (drone.velocity.x + forwardX + lateralX) * DRAG;
      drone.velocity.y = (drone.velocity.y + forwardY + lateralY) * DRAG;

      // Speed clamping
      drone.speed = Math.sqrt(drone.velocity.x * drone.velocity.x + drone.velocity.y * drone.velocity.y);
      if (drone.speed > MAX_SPEED) {
        const factor = MAX_SPEED / drone.speed;
        drone.velocity.x *= factor;
        drone.velocity.y *= factor;
        drone.speed = MAX_SPEED;
      }

      // Integrate position
      drone.position.x += drone.velocity.x * DT;
      drone.position.y += drone.velocity.y * DT;

      // Boundary clamp
      drone.position.x = Math.max(15, Math.min(this.map.width - 15, drone.position.x));
      drone.position.y = Math.max(15, Math.min(this.map.height - 15, drone.position.y));

      // Wall collision
      const droneRadius = 14;
      for (const wall of this.map.walls) {
        if (wall.destroyed) continue;
        const colRes = resolveCircleWallCollision(
          drone.position,
          drone.velocity,
          droneRadius,
          wall
        );

        if (colRes.collided) {
          drone.position = colRes.newPos;
          drone.velocity = colRes.newVel;
          // Apply minor collision damage if fast impact
          if (drone.speed > 2.0) {
            drone.health = Math.max(0, drone.health - 1);
          }
        }
      }

      // Trail record every 8 ticks
      if (this.timestep % 8 === 0) {
        drone.trail.push({ ...drone.position });
        if (drone.trail.length > 25) {
          drone.trail.shift();
        }
      }
    }

    // 4. Grasper & Bomb Mechanics
    for (const drone of this.drones) {
      if (drone.health <= 0) continue;

      const wantsGrasp = drone.lastCommand.grasper === 1;

      if (wantsGrasp) {
        if (drone.graspedBombId === null) {
          // Attempt to grasp nearby free bomb
          for (const bomb of this.bombs) {
            if (!bomb.isGrasped && !bomb.isDisposed) {
              const d = distance(drone.position, bomb.position);
              if (d < 36) {
                // Grasp success!
                bomb.isGrasped = true;
                bomb.graspedByDroneId = drone.id;
                drone.graspedBombId = bomb.id;

                this.addMessage({
                  senderId: drone.id,
                  timestep: this.timestep,
                  text: `Drone #${drone.id} grasped bomb #${bomb.id}. En route to Disposal Center.`,
                  type: 'bomb_found',
                  position: drone.position,
                });
                break;
              }
            }
          }
        }
      } else {
        // Release bomb if held
        if (drone.graspedBombId !== null) {
          const bomb = this.bombs.find((b) => b.id === drone.graspedBombId);
          if (bomb) {
            bomb.isGrasped = false;
            bomb.graspedByDroneId = null;
            bomb.velocity = { ...drone.velocity };
          }
          drone.graspedBombId = null;
        }
      }
    }

    // 5. Update Bomb positions
    for (const bomb of this.bombs) {
      if (bomb.isDisposed) continue;

      if (bomb.isGrasped && bomb.graspedByDroneId !== null) {
        const holder = this.drones.find((d) => d.id === bomb.graspedByDroneId);
        if (holder) {
          // Position bomb slightly behind or under drone
          const offsetDist = 18;
          const attachAngle = holder.angle + Math.PI; // behind
          bomb.position.x = holder.position.x + Math.cos(attachAngle) * offsetDist;
          bomb.position.y = holder.position.y + Math.sin(attachAngle) * offsetDist;
          bomb.velocity = { ...holder.velocity };
        }
      } else {
        // Freely floating bomb physics
        bomb.velocity.x *= DRAG;
        bomb.velocity.y *= DRAG;
        bomb.position.x += bomb.velocity.x * DT;
        bomb.position.y += bomb.velocity.y * DT;

        // Check if bomb has reached Disposal Center!
        if (isPointInRect(bomb.position, this.map.disposalCenter)) {
          bomb.isDisposed = true;
          this.addMessage({
            senderId: 0,
            timestep: this.timestep,
            text: `🎯 Bomb #${bomb.id} safely disposed at Disposal Center!`,
            type: 'disposed',
            position: bomb.position,
          });
        }
      }
    }
  }

  // Calculate official competition scores
  public getScores(): SimulationScores {
    const totalBombs = this.bombs.length || 1;
    const disposedBombs = this.bombs.filter((b) => b.isDisposed).length;
    const disposedPercent = (disposedBombs / totalBombs) * 100;

    let totalHealth = 0;
    let dronesInReturn = 0;
    for (const d of this.drones) {
      totalHealth += d.health;
      if (this.map.returnArea && isPointInRect(d.position, this.map.returnArea)) {
        dronesInReturn++;
      }
    }
    const avgHealth = this.drones.length > 0 ? totalHealth / this.drones.length : 100;

    const maxTimestep = this.maxTimesteps;
    const elapsedSteps = this.timestep;
    // Time efficiency: linear from 0 at limit to 100 at <= half limit
    const halfLimit = maxTimestep * 0.5;
    let timeEfficiency = 0;
    if (disposedBombs === totalBombs && elapsedSteps > 0) {
      if (elapsedSteps <= halfLimit) {
        timeEfficiency = 100;
      } else {
        timeEfficiency = Math.max(0, 100 * (1 - (elapsedSteps - halfLimit) / halfLimit));
      }
    }

    // Rescue total score = 60% Disposed + 20% Health + 20% Efficiency
    const rescueScore = Math.min(
      100,
      disposedPercent * 0.6 + avgHealth * 0.2 + timeEfficiency * 0.2
    );

    // Red Team Place mode scoring
    // 1. Map Exploration accuracy (ratio of explored cells vs total non-boundary cells)
    let exploredCellsCount = 0;
    for (let i = 0; i < this.explorationGrid.length; i++) {
      if (this.explorationGrid[i] > 0) exploredCellsCount++;
    }
    const mapExplorationScore = Math.min(
      100,
      Math.round((exploredCellsCount / (this.gridCols * this.gridRows * 0.45)) * 100)
    );

    // 2. Bomb Placement score (spread and count)
    const placedBombs = this.bombs.filter((b) => b.isPlacedByRed).length;
    const expectedBombs = this.drones.length * 2;
    const bombPlacementScore = Math.min(100, Math.round((placedBombs / expectedBombs) * 100));

    // 3. Wall destruction blast coverage
    const wallDestructionScore = Math.min(100, Math.round(placedBombs * 12));

    // 4. Time score
    const timeScore = Math.max(0, Math.round(100 * (1 - elapsedSteps / maxTimestep)));

    // Red place score = 50% map + 20% placement + 20% destruction + 10% time
    const placeTotalScore = Math.min(
      100,
      mapExplorationScore * 0.5 +
        bombPlacementScore * 0.2 +
        wallDestructionScore * 0.2 +
        timeScore * 0.1
    );

    return {
      mode: this.teamMode,
      disposedBombsCount: disposedBombs,
      totalBombsCount: totalBombs,
      disposedPercent: Math.round(disposedPercent),
      averageDroneHealth: Math.round(avgHealth),
      dronesInReturnAreaCount: dronesInReturn,
      timeElapsedTimesteps: elapsedSteps,
      maxTimesteps: maxTimestep,
      timeElapsedSeconds: Math.round(elapsedSteps * 0.05),
      maxSeconds: 180,
      timeEfficiencyScore: Math.round(timeEfficiency),
      rescueTotalScore: Math.round(rescueScore * 10) / 10,
      mapExplorationScore,
      bombPlacementScore,
      wallDestructionScore,
      timeScore,
      placeTotalScore: Math.round(placeTotalScore * 10) / 10,
    };
  }
}
