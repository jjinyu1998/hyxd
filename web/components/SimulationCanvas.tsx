import React, { useRef, useEffect, useState } from 'react';
import { SimulationEngine } from '../simulation/simulationEngine';
import { DroneState, BombState, WallSegment, ZoneRect } from '../types/simulation';

interface SimulationCanvasProps {
  engine: SimulationEngine;
  selectedDroneId: number;
  onSelectDrone: (id: number) => void;
  showLidar: boolean;
  showSemantic: boolean;
  showExploredGrid: boolean;
  showTrails: boolean;
}

export const SimulationCanvas: React.FC<SimulationCanvasProps> = ({
  engine,
  selectedDroneId,
  onSelectDrone,
  showLidar,
  showSemantic,
  showExploredGrid,
  showTrails,
}) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const containerRef = useRef<HTMLDivElement | null>(null);
  const [zoom, setZoom] = useState<number>(0.85);
  const [pan, setPan] = useState<{ x: number; y: number }>({ x: 20, y: 20 });
  const [isDragging, setIsDragging] = useState<boolean>(false);
  const [dragStart, setDragStart] = useState<{ x: number; y: number }>({ x: 0, y: 0 });

  // Handle canvas resize & render loop
  useEffect(() => {
    let animId: number;

    const render = () => {
      const canvas = canvasRef.current;
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      if (!ctx) return;

      const container = containerRef.current;
      if (container) {
        if (canvas.width !== container.clientWidth || canvas.height !== container.clientHeight) {
          canvas.width = container.clientWidth;
          canvas.height = container.clientHeight;
        }
      }

      const map = engine.map;

      // Background
      ctx.fillStyle = '#090d16';
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      ctx.save();
      // Apply pan & zoom
      ctx.translate(pan.x, pan.y);
      ctx.scale(zoom, zoom);

      // Map canvas bounds
      ctx.fillStyle = '#0f172a';
      ctx.fillRect(0, 0, map.width, map.height);

      // Grid lines
      ctx.strokeStyle = 'rgba(51, 65, 85, 0.25)';
      ctx.lineWidth = 1;
      const gridSize = 50;
      for (let x = 0; x <= map.width; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, map.height);
        ctx.stroke();
      }
      for (let y = 0; y <= map.height; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(map.width, y);
        ctx.stroke();
      }

      // 1. Render Explored Grid overlay if enabled
      if (showExploredGrid && engine.explorationGrid.length > 0) {
        const cellSize = engine.gridCellSize;
        const cols = engine.gridCols;
        const rows = engine.gridRows;
        for (let r = 0; r < rows; r++) {
          for (let c = 0; c < cols; c++) {
            const val = engine.explorationGrid[r * cols + c];
            if (val === 1) {
              // Free space explored
              ctx.fillStyle = 'rgba(56, 189, 248, 0.05)';
              ctx.fillRect(c * cellSize, r * cellSize, cellSize, cellSize);
            } else if (val === 2) {
              // Wall sensed
              ctx.fillStyle = 'rgba(239, 68, 68, 0.35)';
              ctx.fillRect(c * cellSize, r * cellSize, cellSize, cellSize);
            }
          }
        }
      }

      // 2. Render Disabler Zones (GPS / Com)
      for (const zone of map.disablerZones) {
        const isGps = zone.type === 'gps_disabler';
        ctx.fillStyle = isGps ? 'rgba(234, 88, 12, 0.12)' : 'rgba(168, 85, 247, 0.12)';
        ctx.strokeStyle = isGps ? '#ea580c' : '#a855f7';
        ctx.lineWidth = 2;
        ctx.setLineDash([6, 4]);
        ctx.fillRect(zone.x, zone.y, zone.width, zone.height);
        ctx.strokeRect(zone.x, zone.y, zone.width, zone.height);
        ctx.setLineDash([]);

        ctx.fillStyle = isGps ? '#fb923c' : '#c084fc';
        ctx.font = '11px monospace';
        ctx.fillText(
          isGps ? '⚡ GPS BLINDSPOT' : '📡 COM INTERFERENCE',
          zone.x + 8,
          zone.y + 18
        );
      }

      // 3. Render Disposal Center
      const dc = map.disposalCenter;
      ctx.fillStyle = 'rgba(22, 163, 74, 0.15)';
      ctx.strokeStyle = '#22c55e';
      ctx.lineWidth = 3;
      ctx.fillRect(dc.x, dc.y, dc.width, dc.height);
      ctx.strokeRect(dc.x, dc.y, dc.width, dc.height);

      // Disposal center stripes/hatch
      ctx.fillStyle = 'rgba(34, 197, 94, 0.8)';
      ctx.font = 'bold 12px sans-serif';
      ctx.fillText('DISPOSAL CENTER', dc.x + 12, dc.y + 24);
      ctx.fillStyle = 'rgba(34, 197, 94, 0.5)';
      ctx.font = '10px monospace';
      ctx.fillText('DROP ZONE', dc.x + 12, dc.y + 40);

      // 4. Render Start Area / Return Area
      const sa = map.startArea;
      ctx.fillStyle = 'rgba(37, 99, 235, 0.12)';
      ctx.strokeStyle = '#3b82f6';
      ctx.lineWidth = 2;
      ctx.fillRect(sa.x, sa.y, sa.width, sa.height);
      ctx.strokeRect(sa.x, sa.y, sa.width, sa.height);
      ctx.fillStyle = '#60a5fa';
      ctx.font = 'bold 11px sans-serif';
      ctx.fillText('START / DOCKING HUB', sa.x + 10, sa.y + 20);

      // 5. Render Walls
      ctx.strokeStyle = '#94a3b8';
      ctx.lineCap = 'round';
      for (const wall of map.walls) {
        if (wall.destroyed) continue;
        ctx.lineWidth = wall.thickness;
        ctx.beginPath();
        ctx.moveTo(wall.x1, wall.y1);
        ctx.lineTo(wall.x2, wall.y2);
        ctx.stroke();
      }

      // 6. Render Drone Trails
      if (showTrails) {
        for (const drone of engine.drones) {
          if (drone.trail.length < 2) continue;
          ctx.strokeStyle = drone.color + '44';
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.moveTo(drone.trail[0].x, drone.trail[0].y);
          for (let i = 1; i < drone.trail.length; i++) {
            ctx.lineTo(drone.trail[i].x, drone.trail[i].y);
          }
          ctx.stroke();
        }
      }

      // 7. Render Bombs
      for (const bomb of engine.bombs) {
        if (bomb.isDisposed) continue;

        const bx = bomb.position.x;
        const by = bomb.position.y;

        // Blast radius indicator for Red placed bombs
        if (bomb.isPlacedByRed && bomb.blastRadius) {
          ctx.strokeStyle = 'rgba(239, 68, 68, 0.2)';
          ctx.fillStyle = 'rgba(239, 68, 68, 0.05)';
          ctx.lineWidth = 1;
          ctx.setLineDash([4, 4]);
          ctx.beginPath();
          ctx.arc(bx, by, bomb.blastRadius, 0, Math.PI * 2);
          ctx.fill();
          ctx.stroke();
          ctx.setLineDash([]);
        }

        // Bomb pulse glow
        ctx.fillStyle = bomb.isGrasped ? '#eab308' : '#ef4444';
        ctx.beginPath();
        ctx.arc(bx, by, 8, 0, Math.PI * 2);
        ctx.fill();

        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Bomb fuse / cross
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 9px sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(bomb.id.toString(), bx, by);
      }

      // 8. Render Drones & Sensors
      for (const drone of engine.drones) {
        if (drone.health <= 0) {
          // Dead drone wreck
          ctx.fillStyle = '#475569';
          ctx.beginPath();
          ctx.arc(drone.position.x, drone.position.y, 10, 0, Math.PI * 2);
          ctx.fill();
          ctx.strokeStyle = '#ef4444';
          ctx.lineWidth = 2;
          ctx.stroke();
          continue;
        }

        const dx = drone.position.x;
        const dy = drone.position.y;
        const isSelected = drone.id === selectedDroneId;

        // Lidar Rays (render for selected drone or all if selected)
        if (showLidar && (isSelected || engine.drones.length <= 4)) {
          for (const ray of drone.lidarRays) {
            ctx.strokeStyle = isSelected ? 'rgba(56, 189, 248, 0.25)' : 'rgba(56, 189, 248, 0.1)';
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.moveTo(dx, dy);
            ctx.lineTo(ray.hitPoint.x, ray.hitPoint.y);
            ctx.stroke();

            // Hit dot
            if (ray.distance < 315) {
              ctx.fillStyle = '#38bdf8';
              ctx.fillRect(ray.hitPoint.x - 1.5, ray.hitPoint.y - 1.5, 3, 3);
            }
          }
        }

        // Semantic Sensor lines
        if (showSemantic && isSelected) {
          for (const hit of drone.semanticHits) {
            ctx.strokeStyle = hit.entityType === 'BOMB' ? 'rgba(239, 68, 68, 0.6)' : 'rgba(34, 197, 94, 0.4)';
            ctx.lineWidth = 1.5;
            ctx.setLineDash([3, 3]);
            ctx.beginPath();
            ctx.moveTo(dx, dy);
            ctx.lineTo(hit.position.x, hit.position.y);
            ctx.stroke();
            ctx.setLineDash([]);
          }
        }

        // Drone Body
        ctx.save();
        ctx.translate(dx, dy);
        ctx.rotate(drone.angle);

        // Selection ring
        if (isSelected) {
          ctx.strokeStyle = '#38bdf8';
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.arc(0, 0, 22, 0, Math.PI * 2);
          ctx.stroke();
        }

        // Rotors
        const rotorOffset = 13;
        ctx.strokeStyle = '#64748b';
        ctx.lineWidth = 2;
        // Arms
        ctx.beginPath();
        ctx.moveTo(-rotorOffset, -rotorOffset);
        ctx.lineTo(rotorOffset, rotorOffset);
        ctx.moveTo(-rotorOffset, rotorOffset);
        ctx.lineTo(rotorOffset, -rotorOffset);
        ctx.stroke();

        // 4 Rotor discs
        ctx.fillStyle = isSelected ? 'rgba(56, 189, 248, 0.3)' : 'rgba(148, 163, 184, 0.25)';
        for (const [rx, ry] of [
          [-rotorOffset, -rotorOffset],
          [rotorOffset, -rotorOffset],
          [-rotorOffset, rotorOffset],
          [rotorOffset, rotorOffset],
        ]) {
          ctx.beginPath();
          ctx.arc(rx, ry, 6, 0, Math.PI * 2);
          ctx.fill();
        }

        // Main Drone Hull
        ctx.fillStyle = drone.color;
        ctx.beginPath();
        ctx.arc(0, 0, 11, 0, Math.PI * 2);
        ctx.fill();

        // Direction Nose (front arrow)
        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.moveTo(13, 0);
        ctx.lineTo(4, -6);
        ctx.lineTo(7, 0);
        ctx.lineTo(4, 6);
        ctx.closePath();
        ctx.fill();

        // Grasper claws
        ctx.strokeStyle = drone.lastCommand.grasper === 1 ? '#22c55e' : '#94a3b8';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(-10, -5);
        ctx.lineTo(-15, -7);
        ctx.moveTo(-10, 5);
        ctx.lineTo(-15, 7);
        ctx.stroke();

        ctx.restore();

        // Drone ID Label & Health Bar
        ctx.textAlign = 'center';
        ctx.font = 'bold 10px sans-serif';
        ctx.fillStyle = '#ffffff';
        ctx.fillText(`#${drone.id}`, dx, dy - 20);

        // Mini health bar
        ctx.fillStyle = 'rgba(0,0,0,0.5)';
        ctx.fillRect(dx - 12, dy + 18, 24, 4);
        const hpWidth = Math.max(0, (drone.health / 100) * 24);
        ctx.fillStyle = drone.health > 40 ? '#22c55e' : '#ef4444';
        ctx.fillRect(dx - 12, dy + 18, hpWidth, 4);
      }

      ctx.restore();

      animId = requestAnimationFrame(render);
    };

    animId = requestAnimationFrame(render);
    return () => cancelAnimationFrame(animId);
  }, [engine, selectedDroneId, zoom, pan, showLidar, showSemantic, showExploredGrid, showTrails]);

  // Handle canvas mouse click to select drone
  const handleCanvasClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (isDragging) return;
    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const mouseX = e.clientX - rect.left;
    const mouseY = e.clientY - rect.top;

    // Convert screen coordinates to world coordinates
    const worldX = (mouseX - pan.x) / zoom;
    const worldY = (mouseY - pan.y) / zoom;

    // Check click on drone
    for (const drone of engine.drones) {
      const d = Math.sqrt(
        (drone.position.x - worldX) ** 2 + (drone.position.y - worldY) ** 2
      );
      if (d < 30) {
        onSelectDrone(drone.id);
        return;
      }
    }
  };

  // Mouse wheel zoom
  const handleWheel = (e: React.WheelEvent<HTMLCanvasElement>) => {
    e.preventDefault();
    const zoomFactor = e.deltaY < 0 ? 1.1 : 0.9;
    setZoom((prev) => Math.max(0.3, Math.min(2.5, prev * zoomFactor)));
  };

  // Pan controls
  const handleMouseDown = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (e.button === 0 && e.shiftKey || e.button === 1 || e.button === 0) {
      setIsDragging(true);
      setDragStart({ x: e.clientX - pan.x, y: e.clientY - pan.y });
    }
  };

  const handleMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (isDragging) {
      setPan({
        x: e.clientX - dragStart.x,
        y: e.clientY - dragStart.y,
      });
    }
  };

  const handleMouseUp = () => {
    setIsDragging(false);
  };

  const handleResetView = () => {
    setZoom(0.85);
    setPan({ x: 20, y: 20 });
  };

  return (
    <div ref={containerRef} className="relative w-full h-full overflow-hidden bg-slate-950">
      <canvas
        ref={canvasRef}
        onClick={handleCanvasClick}
        onWheel={handleWheel}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
        className="w-full h-full block cursor-crosshair"
      />

      {/* Floating Canvas Controls */}
      <div className="absolute bottom-3 left-3 flex items-center gap-2 bg-slate-900/90 backdrop-blur border border-slate-700/60 rounded-lg px-2.5 py-1.5 shadow-lg text-xs text-slate-300">
        <button
          onClick={() => setZoom((z) => Math.min(2.5, z + 0.15))}
          className="px-2 py-0.5 rounded hover:bg-slate-800 text-slate-200 font-bold"
          title="Zoom In"
        >
          +
        </button>
        <span className="w-12 text-center text-slate-400 font-mono">
          {Math.round(zoom * 100)}%
        </span>
        <button
          onClick={() => setZoom((z) => Math.max(0.3, z - 0.15))}
          className="px-2 py-0.5 rounded hover:bg-slate-800 text-slate-200 font-bold"
          title="Zoom Out"
        >
          -
        </button>
        <div className="w-px h-4 bg-slate-700 mx-1" />
        <button
          onClick={handleResetView}
          className="px-2 py-0.5 rounded hover:bg-slate-800 text-sky-400"
          title="Reset View"
        >
          Reset View
        </button>
      </div>

      {/* Hint overlay */}
      <div className="absolute top-3 left-3 pointer-events-none bg-slate-900/80 backdrop-blur border border-slate-800 rounded px-2.5 py-1 text-[11px] text-slate-400 font-mono">
        Drag to Pan • Scroll to Zoom • Click Drone to Select
      </div>
    </div>
  );
};
