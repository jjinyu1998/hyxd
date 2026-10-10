import {
  Vector2,
  DroneState,
  WallSegment,
  BombState,
  ZoneRect,
  LidarRay,
  SemanticHit,
} from '../types/simulation';
import {
  raySegmentIntersection,
  normalizeAngle,
  distance,
  isPointInRect,
} from './physics';

// Generate normal random variable using Box-Muller transform
export function gaussianRandom(mean = 0, stdev = 1): number {
  const u1 = 1 - Math.random();
  const u2 = 1 - Math.random();
  const randStdNormal = Math.sqrt(-2.0 * Math.log(u1)) * Math.sin(2.0 * Math.PI * u2);
  return mean + randStdNormal * stdev;
}

// Compute Lidar rays for drone
export function computeLidar(
  pos: Vector2,
  droneAngle: number,
  walls: WallSegment[],
  rayCount: number = 24,
  maxDistance: number = 320
): LidarRay[] {
  const rays: LidarRay[] = [];
  const angleStep = (2 * Math.PI) / rayCount;

  for (let i = 0; i < rayCount; i++) {
    // Relative angle from -PI to PI
    const relAngle = normalizeAngle(-Math.PI + i * angleStep);
    const globalAngle = normalizeAngle(droneAngle + relAngle);

    let closestDist = maxDistance;
    let hitPt: Vector2 = {
      x: pos.x + Math.cos(globalAngle) * maxDistance,
      y: pos.y + Math.sin(globalAngle) * maxDistance,
    };

    for (const wall of walls) {
      if (wall.destroyed) continue;
      const res = raySegmentIntersection(pos, globalAngle, closestDist, wall);
      if (res.hit && res.dist < closestDist) {
        closestDist = res.dist;
        hitPt = res.point;
      }
    }

    // Add small sensor noise
    const noisyDist = Math.max(5, closestDist + gaussianRandom(0, 0.5));

    rays.push({
      angle: relAngle,
      distance: noisyDist,
      hitPoint: hitPt,
    });
  }

  return rays;
}

// Check line of sight between two points
export function hasLineOfSight(
  p1: Vector2,
  p2: Vector2,
  walls: WallSegment[]
): boolean {
  const d = distance(p1, p2);
  if (d < 1) return true;
  const angle = Math.atan2(p2.y - p1.y, p2.x - p1.x);

  for (const wall of walls) {
    if (wall.destroyed) continue;
    const res = raySegmentIntersection(p1, angle, d - 4, wall);
    if (res.hit) {
      return false;
    }
  }
  return true;
}

// Compute Semantic Sensor hits
export function computeSemanticSensor(
  drone: { id: number; position: Vector2; angle: number },
  bombs: BombState[],
  disposalCenter: ZoneRect,
  returnArea: ZoneRect | undefined,
  otherDrones: DroneState[],
  walls: WallSegment[],
  maxRange: number = 280
): SemanticHit[] {
  const hits: SemanticHit[] = [];

  // 1. Detect Bombs
  for (const bomb of bombs) {
    if (bomb.isDisposed) continue;
    const d = distance(drone.position, bomb.position);
    if (d <= maxRange) {
      if (hasLineOfSight(drone.position, bomb.position, walls)) {
        const globalAngle = Math.atan2(
          bomb.position.y - drone.position.y,
          bomb.position.x - drone.position.x
        );
        const relAngle = normalizeAngle(globalAngle - drone.angle);

        hits.push({
          distance: d,
          angle: relAngle,
          entityType: 'BOMB',
          id: bomb.id,
          position: bomb.position,
          grasped: bomb.isGrasped,
        });
      }
    }
  }

  // 2. Detect Disposal Center
  const disposalCenterPt: Vector2 = {
    x: disposalCenter.x + disposalCenter.width / 2,
    y: disposalCenter.y + disposalCenter.height / 2,
  };
  const distDisposal = distance(drone.position, disposalCenterPt);
  if (distDisposal <= maxRange * 1.5) {
    const globalAngle = Math.atan2(
      disposalCenterPt.y - drone.position.y,
      disposalCenterPt.x - drone.position.x
    );
    hits.push({
      distance: distDisposal,
      angle: normalizeAngle(globalAngle - drone.angle),
      entityType: 'DISPOSAL_CENTER',
      id: disposalCenter.id,
      position: disposalCenterPt,
      grasped: false,
    });
  }

  // 3. Detect other Drones
  for (const other of otherDrones) {
    if (other.id === drone.id) continue;
    const d = distance(drone.position, other.position);
    if (d <= maxRange && hasLineOfSight(drone.position, other.position, walls)) {
      const globalAngle = Math.atan2(
        other.position.y - drone.position.y,
        other.position.x - drone.position.x
      );
      hits.push({
        distance: d,
        angle: normalizeAngle(globalAngle - drone.angle),
        entityType: 'DRONE',
        id: other.id,
        position: other.position,
        grasped: false,
      });
    }
  }

  // Sort by closest distance
  hits.sort((a, b) => a.distance - b.distance);
  return hits;
}

// Compute GPS sensor (noisy, or disabled in GPS jamming zone)
export function computeGps(
  pos: Vector2,
  disablers: ZoneRect[]
): { position: Vector2 | null; disabled: boolean } {
  for (const zone of disablers) {
    if (zone.type === 'gps_disabler' && isPointInRect(pos, zone)) {
      return { position: null, disabled: true };
    }
  }

  return {
    position: {
      x: pos.x + gaussianRandom(0, 1.5),
      y: pos.y + gaussianRandom(0, 1.5),
    },
    disabled: false,
  };
}

// Check Com Disabler zone
export function isComDisabled(pos: Vector2, disablers: ZoneRect[]): boolean {
  for (const zone of disablers) {
    if (zone.type === 'com_disabler' && isPointInRect(pos, zone)) {
      return true;
    }
  }
  return false;
}
