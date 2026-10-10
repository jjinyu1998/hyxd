import { Vector2, WallSegment } from '../types/simulation';

export function normalizeAngle(angle: number): number {
  let a = angle;
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

export function distance(p1: Vector2, p2: Vector2): number {
  const dx = p2.x - p1.x;
  const dy = p2.y - p1.y;
  return Math.sqrt(dx * dx + dy * dy);
}

export function distanceSquared(p1: Vector2, p2: Vector2): number {
  const dx = p2.x - p1.x;
  const dy = p2.y - p1.y;
  return dx * dx + dy * dy;
}

// Ray-segment intersection
export function raySegmentIntersection(
  rayOrigin: Vector2,
  rayAngle: number,
  maxDist: number,
  seg: WallSegment
): { hit: boolean; dist: number; point: Vector2 } {
  const rx = Math.cos(rayAngle);
  const ry = Math.sin(rayAngle);

  const x1 = seg.x1;
  const y1 = seg.y1;
  const x2 = seg.x2;
  const y2 = seg.y2;

  const dx = x2 - x1;
  const dy = y2 - y1;

  const denom = rx * dy - ry * dx;
  if (Math.abs(denom) < 1e-6) {
    return { hit: false, dist: maxDist, point: { x: 0, y: 0 } };
  }

  const ox = x1 - rayOrigin.x;
  const oy = y1 - rayOrigin.y;

  const t = (ox * dy - oy * dx) / denom;
  const u = (ox * ry - oy * rx) / denom;

  if (t >= 0 && t <= maxDist && u >= 0 && u <= 1) {
    return {
      hit: true,
      dist: t,
      point: {
        x: rayOrigin.x + rx * t,
        y: rayOrigin.y + ry * t,
      },
    };
  }

  return { hit: false, dist: maxDist, point: { x: 0, y: 0 } };
}

// Find closest distance from point to line segment
export function pointToSegmentDistance(
  p: Vector2,
  seg: WallSegment
): { dist: number; closestPoint: Vector2; normal: Vector2 } {
  const x1 = seg.x1;
  const y1 = seg.y1;
  const x2 = seg.x2;
  const y2 = seg.y2;

  const dx = x2 - x1;
  const dy = y2 - y1;
  const lenSq = dx * dx + dy * dy;

  if (lenSq === 0) {
    const d = distance(p, { x: x1, y: y1 });
    return {
      dist: d,
      closestPoint: { x: x1, y: y1 },
      normal: { x: 0, y: 1 },
    };
  }

  let t = ((p.x - x1) * dx + (p.y - y1) * dy) / lenSq;
  t = Math.max(0, Math.min(1, t));

  const cx = x1 + t * dx;
  const cy = y1 + t * dy;

  const nx = p.x - cx;
  const ny = p.y - cy;
  const d = Math.sqrt(nx * nx + ny * ny);

  return {
    dist: d,
    closestPoint: { x: cx, y: cy },
    normal: d > 0.001 ? { x: nx / d, y: ny / d } : { x: 0, y: 0 },
  };
}

// Circle to wall segment collision resolution
export function resolveCircleWallCollision(
  pos: Vector2,
  vel: Vector2,
  radius: number,
  seg: WallSegment,
  damping: number = 0.5
): { collided: boolean; newPos: Vector2; newVel: Vector2 } {
  const effectiveRadius = radius + seg.thickness * 0.5;
  const { dist, normal } = pointToSegmentDistance(pos, seg);

  if (dist < effectiveRadius) {
    const penetration = effectiveRadius - dist;
    // Push out along normal
    const newPos = {
      x: pos.x + normal.x * penetration,
      y: pos.y + normal.y * penetration,
    };

    // Reflect or damp velocity along normal
    const dot = vel.x * normal.x + vel.y * normal.y;
    let newVel = vel;
    if (dot < 0) {
      newVel = {
        x: (vel.x - (1 + damping) * dot * normal.x) * 0.7,
        y: (vel.y - (1 + damping) * dot * normal.y) * 0.7,
      };
    }

    return { collided: true, newPos, newVel };
  }

  return { collided: false, newPos: pos, newVel: vel };
}

// Check if point is inside rectangle
export function isPointInRect(
  p: Vector2,
  rect: { x: number; y: number; width: number; height: number }
): boolean {
  return (
    p.x >= rect.x &&
    p.x <= rect.x + rect.width &&
    p.y >= rect.y &&
    p.y <= rect.y + rect.height
  );
}
