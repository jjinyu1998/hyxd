export type Vector2 = {
  x: number;
  y: number;
};

export type EntityType = 
  | 'WALL'
  | 'BOMB'
  | 'DISPOSAL_CENTER'
  | 'RETURN_AREA'
  | 'DRONE'
  | 'DISABLER_ZONE'
  | 'OTHER';

export type SemanticHit = {
  distance: number;
  angle: number; // relative to drone heading
  entityType: EntityType;
  id: number;
  position: Vector2;
  grasped: boolean;
};

export type LidarRay = {
  angle: number; // relative angle from drone orientation
  distance: number;
  hitPoint: Vector2;
};

export type DroneStateMachine = 
  | 'idle'
  | 'explore'
  | 'chase'
  | 'carry'
  | 'push'
  | 'spin_release'
  | 'escape'
  | 'manual'
  | 'dead';

export type CommandsDict = {
  forward: number;    // [-1, 1]
  lateral: number;    // [-1, 1]
  rotation: number;   // [-1, 1]
  grasper: number;    // 0 or 1
  placeBomb?: boolean;
};

export type DroneState = {
  id: number;
  position: Vector2;
  velocity: Vector2;
  angle: number; // radians
  angularVelocity: number;
  health: number; // 0-100
  graspedBombId: number | null;
  carriedBombsCount: number; // For red team
  stateMachine: DroneStateMachine;
  lidarRays: LidarRay[];
  semanticHits: SemanticHit[];
  gpsPosition: Vector2 | null;
  compassAngle: number | null;
  speed: number;
  isGpsDisabled: boolean;
  isComDisabled: boolean;
  lastCommand: CommandsDict;
  trail: Vector2[];
  color: string;
};

export type BombState = {
  id: number;
  position: Vector2;
  velocity: Vector2;
  isGrasped: boolean;
  graspedByDroneId: number | null;
  isDisposed: boolean;
  isPlacedByRed: boolean;
  blastRadius?: number;
};

export type WallSegment = {
  id: number;
  x1: number;
  y1: number;
  x2: number;
  y2: number;
  thickness: number;
  isBoundary?: boolean;
  destroyed?: boolean;
};

export type ZoneType = 'disposal_center' | 'start_area' | 'return_area' | 'gps_disabler' | 'com_disabler';

export type ZoneRect = {
  id: number;
  type: ZoneType;
  x: number;
  y: number;
  width: number;
  height: number;
  color?: string;
  name?: string;
};

export type MapDefinition = {
  id: string;
  name: string;
  width: number;
  height: number;
  walls: WallSegment[];
  startArea: ZoneRect;
  disposalCenter: ZoneRect;
  returnArea?: ZoneRect;
  disablerZones: ZoneRect[];
  defaultBombs: Vector2[];
};

export type TeamMode = 'rescue' | 'place' | 'manual';

export type SimulationScores = {
  mode: TeamMode;
  disposedBombsCount: number;
  totalBombsCount: number;
  disposedPercent: number;
  averageDroneHealth: number;
  dronesInReturnAreaCount: number;
  timeElapsedTimesteps: number;
  maxTimesteps: number;
  timeElapsedSeconds: number;
  maxSeconds: number;
  timeEfficiencyScore: number;
  rescueTotalScore: number;
  // Red place mode scores
  mapExplorationScore: number;
  bombPlacementScore: number;
  wallDestructionScore: number;
  timeScore: number;
  placeTotalScore: number;
};

export type DroneMessage = {
  id: string;
  senderId: number;
  timestep: number;
  text: string;
  type: 'info' | 'alert' | 'bomb_found' | 'disposed' | 'jammed';
  position?: Vector2;
};
