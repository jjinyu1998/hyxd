import { MapDefinition, WallSegment, ZoneRect, Vector2 } from '../types/simulation';
import map01Json from './maps/map_01.json';
import map02Json from './maps/map_02.json';
import map03Json from './maps/map_03.json';
import map04Json from './maps/map_04.json';
import map05Json from './maps/map_05.json';

type RawElement = {
  id?: number;
  type: string;
  points?: number[][];
  thickness?: number;
  top_left?: number[];
  bottom_right?: number[];
  color?: string;
};

type RawMapJson = {
  canvas: {
    width: number;
    height: number;
    background: string;
  };
  elements: RawElement[];
};

function parseRawMap(
  id: string,
  name: string,
  raw: RawMapJson,
  defaultBombs: Vector2[],
  extraZones?: ZoneRect[]
): MapDefinition {
  const width = raw.canvas.width;
  const height = raw.canvas.height;
  const walls: WallSegment[] = [];
  let wallId = 1;

  let startArea: ZoneRect = {
    id: 100,
    type: 'start_area',
    x: 50,
    y: 50,
    width: 150,
    height: 150,
    name: 'Drone Spawn Area',
  };

  let disposalCenter: ZoneRect = {
    id: 200,
    type: 'disposal_center',
    x: 50,
    y: 250,
    width: 180,
    height: 150,
    name: 'Bomb Disposal Center',
  };

  for (const el of raw.elements) {
    if (el.type === 'line_wall' && el.points && el.points.length >= 2) {
      const p1 = el.points[0];
      const p2 = el.points[1];
      const isBoundary =
        (Math.abs(p1[0] - p2[0]) < 10 && (p1[0] < 15 || p1[0] > width - 15)) ||
        (Math.abs(p1[1] - p2[1]) < 10 && (p1[1] < 15 || p1[1] > height - 15));

      walls.push({
        id: wallId++,
        x1: p1[0],
        y1: p1[1],
        x2: p2[0],
        y2: p2[1],
        thickness: el.thickness || 6,
        isBoundary,
      });
    } else if (el.type === 'rect_wall' && el.top_left && el.bottom_right) {
      const x1 = el.top_left[0];
      const y1 = el.top_left[1];
      const x2 = el.bottom_right[0];
      const y2 = el.bottom_right[1];
      const thick = el.thickness || 6;

      // 4 boundary lines of rect wall
      walls.push(
        { id: wallId++, x1, y1, x2, y2: y1, thickness: thick },
        { id: wallId++, x1: x2, y1, x2, y2, thickness: thick },
        { id: wallId++, x1: x2, y1: y2, x2: x1, y2, thickness: thick },
        { id: wallId++, x1, y1: y2, x2: x1, y2: y1, thickness: thick }
      );
    } else if (el.type === 'disposal_center' && el.top_left && el.bottom_right) {
      disposalCenter = {
        id: el.id || 200,
        type: 'disposal_center',
        x: el.top_left[0],
        y: el.top_left[1],
        width: Math.abs(el.bottom_right[0] - el.top_left[0]),
        height: Math.abs(el.bottom_right[1] - el.top_left[1]),
        color: el.color || '#16a34a',
        name: 'Disposal Center',
      };
    } else if (el.type === 'start_area' && el.top_left && el.bottom_right) {
      startArea = {
        id: el.id || 100,
        type: 'start_area',
        x: el.top_left[0],
        y: el.top_left[1],
        width: Math.abs(el.bottom_right[0] - el.top_left[0]),
        height: Math.abs(el.bottom_right[1] - el.top_left[1]),
        color: el.color || '#2563eb',
        name: 'Deployment Hub',
      };
    }
  }

  // Define Return Area adjacent to start area for drone recovery
  const returnArea: ZoneRect = {
    id: 300,
    type: 'return_area',
    x: startArea.x,
    y: startArea.y,
    width: startArea.width,
    height: startArea.height,
    name: 'Return & Docking Area',
  };

  const disablerZones: ZoneRect[] = extraZones || [];

  return {
    id,
    name,
    width,
    height,
    walls,
    startArea,
    disposalCenter,
    returnArea,
    disablerZones,
    defaultBombs,
  };
}

// Map 01: Training Facility (1250x800)
export const MAP_01 = parseRawMap(
  'map_01',
  'Map 01: Training Facility',
  map01Json as unknown as RawMapJson,
  [
    { x: 350, y: 250 },
    { x: 420, y: 650 },
    { x: 720, y: 200 },
    { x: 780, y: 550 },
    { x: 1080, y: 350 },
    { x: 1120, y: 680 },
  ],
  [
    {
      id: 501,
      type: 'gps_disabler',
      x: 600,
      y: 100,
      width: 200,
      height: 250,
      name: 'GPS Jamming Zone A',
    },
  ]
);

// Map 02: Research Complex (1200x900)
export const MAP_02 = parseRawMap(
  'map_02',
  'Map 02: Research Complex',
  map02Json as unknown as RawMapJson,
  [
    { x: 300, y: 300 },
    { x: 320, y: 780 },
    { x: 750, y: 150 },
    { x: 800, y: 580 },
    { x: 1050, y: 380 },
    { x: 1020, y: 780 },
    { x: 550, y: 720 },
  ],
  [
    {
      id: 502,
      type: 'com_disabler',
      x: 700,
      y: 400,
      width: 250,
      height: 220,
      name: 'Radio Interference Zone',
    },
  ]
);

// Map 03: Industrial Labyrinth (1113x750)
export const MAP_03 = parseRawMap(
  'map_03',
  'Map 03: Industrial Labyrinth',
  map03Json as unknown as RawMapJson,
  [
    { x: 280, y: 200 },
    { x: 310, y: 600 },
    { x: 620, y: 250 },
    { x: 600, y: 580 },
    { x: 920, y: 220 },
    { x: 950, y: 620 },
  ],
  [
    {
      id: 503,
      type: 'gps_disabler',
      x: 520,
      y: 180,
      width: 220,
      height: 250,
      name: 'Underground GPS Blindspot',
    },
    {
      id: 504,
      type: 'com_disabler',
      x: 820,
      y: 450,
      width: 200,
      height: 200,
      name: 'High EM Noise Field',
    },
  ]
);

// Map 04: Collapsed Basement (1660x1122)
export const MAP_04 = parseRawMap(
  'map_04',
  'Map 04: Collapsed Basement',
  map04Json as unknown as RawMapJson,
  [
    { x: 450, y: 350 },
    { x: 520, y: 880 },
    { x: 950, y: 280 },
    { x: 920, y: 820 },
    { x: 1350, y: 350 },
    { x: 1400, y: 850 },
    { x: 800, y: 550 },
    { x: 1200, y: 600 },
  ],
  [
    {
      id: 505,
      type: 'gps_disabler',
      x: 850,
      y: 200,
      width: 320,
      height: 280,
      name: 'Sub-Level GPS Shielded Zone',
    },
  ]
);

// Map 05: Catacombs & Ruins (1200x1200)
export const MAP_05 = parseRawMap(
  'map_05',
  'Map 05: Catacombs & Ruins',
  map05Json as unknown as RawMapJson,
  [
    { x: 260, y: 300 },
    { x: 350, y: 920 },
    { x: 600, y: 320 },
    { x: 620, y: 880 },
    { x: 950, y: 250 },
    { x: 900, y: 750 },
    { x: 1050, y: 1050 },
    { x: 500, y: 600 },
  ],
  [
    {
      id: 506,
      type: 'com_disabler',
      x: 450,
      y: 450,
      width: 300,
      height: 300,
      name: 'Deep Rock Com Jammer',
    },
  ]
);

export const ALL_MAPS: Record<string, MapDefinition> = {
  map_01: MAP_01,
  map_02: MAP_02,
  map_03: MAP_03,
  map_04: MAP_04,
  map_05: MAP_05,
};
