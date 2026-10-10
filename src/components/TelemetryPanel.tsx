import React from 'react';
import {
  Compass,
  MapPin,
  Wifi,
  WifiOff,
  BatteryCharging,
  Cpu,
  Radio,
  Package,
  Layers,
  Activity,
  AlertTriangle,
} from 'lucide-react';
import { DroneState, EntityType } from '../types/simulation';

interface TelemetryPanelProps {
  drones: DroneState[];
  selectedDroneId: number;
  onSelectDrone: (id: number) => void;
  teamMode: string;
  isManualMode: boolean;
}

export const TelemetryPanel: React.FC<TelemetryPanelProps> = ({
  drones,
  selectedDroneId,
  onSelectDrone,
  teamMode,
  isManualMode,
}) => {
  const drone = drones.find((d) => d.id === selectedDroneId) || drones[0];
  if (!drone) return null;

  const headingDeg = Math.round(((drone.angle * 180) / Math.PI + 360) % 360);

  const getStatusBadge = (state: string) => {
    switch (state) {
      case 'chase':
        return 'bg-amber-500/20 text-amber-400 border-amber-500/30';
      case 'carry':
      case 'push':
        return 'bg-emerald-500/20 text-emerald-400 border-emerald-500/30';
      case 'spin_release':
        return 'bg-purple-500/20 text-purple-400 border-purple-500/30';
      case 'escape':
        return 'bg-rose-500/20 text-rose-400 border-rose-500/30';
      case 'explore':
        return 'bg-sky-500/20 text-sky-400 border-sky-500/30';
      case 'manual':
        return 'bg-blue-500/20 text-blue-300 border-blue-500/30';
      case 'dead':
        return 'bg-slate-700 text-slate-400 border-slate-600';
      default:
        return 'bg-slate-800 text-slate-300 border-slate-700';
    }
  };

  return (
    <div className="flex flex-col h-full bg-slate-900 border-l border-slate-800 overflow-hidden text-xs">
      {/* Header & Drone Selector Tabs */}
      <div className="p-3 border-b border-slate-800 bg-slate-950/60">
        <div className="flex items-center justify-between mb-2">
          <span className="font-semibold text-slate-200 uppercase tracking-wider flex items-center gap-1.5">
            <Cpu className="w-3.5 h-3.5 text-sky-400" />
            Fleet Telemetry
          </span>
          <span className="text-[10px] font-mono text-slate-400">
            {drones.length} ACTIVE DRONES
          </span>
        </div>

        {/* Scrollable drone selector */}
        <div className="flex items-center gap-1 overflow-x-auto pb-1 scrollbar-none">
          {drones.map((d) => (
            <button
              key={d.id}
              onClick={() => onSelectDrone(d.id)}
              className={`px-2 py-1 rounded text-xs font-mono font-bold flex items-center gap-1 transition ${
                d.id === selectedDroneId
                  ? 'bg-sky-500 text-slate-950 shadow-sm'
                  : 'bg-slate-800/80 text-slate-300 hover:bg-slate-700'
              }`}
            >
              <span
                className="w-2 h-2 rounded-full"
                style={{ backgroundColor: d.color }}
              />
              #{d.id}
            </button>
          ))}
        </div>
      </div>

      {/* Selected Drone Status */}
      <div className="flex-1 overflow-y-auto p-3 space-y-3">
        {/* Drone Header Card */}
        <div className="bg-slate-950/80 border border-slate-800 rounded-lg p-2.5">
          <div className="flex items-center justify-between mb-2">
            <div className="flex items-center gap-2">
              <div
                className="w-3 h-3 rounded-full animate-ping"
                style={{ backgroundColor: drone.color }}
              />
              <span className="font-bold text-sm text-slate-100 font-mono">
                DRONE #{drone.id}
              </span>
            </div>
            <span
              className={`px-2 py-0.5 rounded text-[10px] font-mono font-bold uppercase border ${getStatusBadge(
                drone.stateMachine
              )}`}
            >
              {drone.stateMachine}
            </span>
          </div>

          {/* Health Gauge */}
          <div className="space-y-1 mb-2">
            <div className="flex justify-between text-[11px] text-slate-400">
              <span className="flex items-center gap-1">
                <BatteryCharging className="w-3 h-3 text-emerald-400" /> Hull Health
              </span>
              <span className="font-mono font-bold text-slate-200">
                {drone.health}%
              </span>
            </div>
            <div className="w-full bg-slate-800 h-1.5 rounded-full overflow-hidden">
              <div
                className={`h-full transition-all duration-300 ${
                  drone.health > 50
                    ? 'bg-emerald-500'
                    : drone.health > 25
                    ? 'bg-amber-500'
                    : 'bg-rose-500'
                }`}
                style={{ width: `${drone.health}%` }}
              />
            </div>
          </div>

          {/* Actuator & Cargo status */}
          <div className="grid grid-cols-2 gap-2 text-[11px] pt-1 border-t border-slate-800/60 font-mono">
            <div>
              <span className="text-slate-400 block text-[10px]">GRASPER CLAW</span>
              <span
                className={`font-bold ${
                  drone.graspedBombId !== null
                    ? 'text-emerald-400'
                    : drone.lastCommand.grasper === 1
                    ? 'text-sky-400'
                    : 'text-slate-400'
                }`}
              >
                {drone.graspedBombId !== null
                  ? `HOLDING #${drone.graspedBombId}`
                  : drone.lastCommand.grasper === 1
                  ? 'ACTIVE [EMPTY]'
                  : 'RELEASED'}
              </span>
            </div>

            <div>
              <span className="text-slate-400 block text-[10px]">
                {teamMode === 'place' ? 'CARRIED BOMBS' : 'SPEED'}
              </span>
              <span className="font-bold text-slate-200">
                {teamMode === 'place'
                  ? `${drone.carriedBombsCount} BOMBS`
                  : `${(drone.speed * 10).toFixed(1)} px/s`}
              </span>
            </div>
          </div>
        </div>

        {/* GPS & Sensor Health */}
        <div className="bg-slate-950/80 border border-slate-800 rounded-lg p-2.5 space-y-2">
          <div className="flex items-center justify-between text-slate-300 font-semibold text-[11px]">
            <span className="flex items-center gap-1">
              <MapPin className="w-3.5 h-3.5 text-sky-400" /> Navigation & Signals
            </span>
            <span className="text-[10px] font-mono text-slate-400">
              HDG {headingDeg}°
            </span>
          </div>

          <div className="grid grid-cols-2 gap-2 font-mono text-[11px]">
            <div className="bg-slate-900 p-1.5 rounded border border-slate-800">
              <div className="text-[10px] text-slate-400 flex items-center justify-between">
                <span>GPS POSITION</span>
                {drone.isGpsDisabled ? (
                  <span className="text-rose-400 flex items-center gap-0.5">
                    <AlertTriangle className="w-2.5 h-2.5" /> BLOCKED
                  </span>
                ) : (
                  <span className="text-emerald-400">FIX 3D</span>
                )}
              </div>
              <div className="text-slate-200 font-bold truncate">
                {drone.gpsPosition
                  ? `${Math.round(drone.gpsPosition.x)}, ${Math.round(drone.gpsPosition.y)}`
                  : 'NO GPS FIX'}
              </div>
            </div>

            <div className="bg-slate-900 p-1.5 rounded border border-slate-800">
              <div className="text-[10px] text-slate-400 flex items-center justify-between">
                <span>MESH RADIO</span>
                {drone.isComDisabled ? (
                  <span className="text-rose-400 flex items-center gap-0.5">
                    <WifiOff className="w-2.5 h-2.5" /> JAMMED
                  </span>
                ) : (
                  <span className="text-emerald-400 flex items-center gap-0.5">
                    <Wifi className="w-2.5 h-2.5" /> ONLINE
                  </span>
                )}
              </div>
              <div className="text-slate-200 font-bold truncate">
                {drone.isComDisabled ? 'DISCONNECTED' : 'CH 04 • 2.4GHz'}
              </div>
            </div>
          </div>
        </div>

        {/* Radar / Lidar Scope */}
        <div className="bg-slate-950/80 border border-slate-800 rounded-lg p-2.5">
          <div className="flex items-center justify-between mb-2">
            <span className="font-semibold text-slate-300 flex items-center gap-1 text-[11px]">
              <Radio className="w-3.5 h-3.5 text-sky-400" />
              Lidar 360° Scope
            </span>
            <span className="text-[10px] font-mono text-slate-400">
              {drone.lidarRays.length} RAYS
            </span>
          </div>

          <div className="relative w-full aspect-square max-h-48 mx-auto bg-slate-900/90 rounded-full border border-slate-700/60 overflow-hidden flex items-center justify-center">
            {/* Concentric range rings */}
            <div className="absolute inset-4 rounded-full border border-slate-700/40" />
            <div className="absolute inset-10 rounded-full border border-slate-700/40" />
            <div className="absolute inset-16 rounded-full border border-slate-700/40" />
            <div className="absolute w-full h-px bg-slate-700/40" />
            <div className="absolute h-full w-px bg-slate-700/40" />

            {/* Sweep indicator */}
            <svg className="w-full h-full absolute inset-0">
              {drone.lidarRays.map((ray, i) => {
                const maxRange = 320;
                const rRatio = Math.min(1, ray.distance / maxRange);
                // Canvas radar radius is approx 80px
                const r = rRatio * 75;
                const angle = ray.angle; // relative angle
                const px = 96 + Math.cos(angle) * r;
                const py = 96 + Math.sin(angle) * r;
                const isWallHit = ray.distance < 310;
                return (
                  <circle
                    key={i}
                    cx={px}
                    cy={py}
                    r={isWallHit ? 2 : 1}
                    fill={isWallHit ? '#38bdf8' : 'rgba(56, 189, 248, 0.2)'}
                  />
                );
              })}
            </svg>

            {/* Drone center indicator */}
            <div className="w-3 h-3 rounded-full bg-sky-400 border-2 border-white shadow-sm z-10" />
          </div>
        </div>

        {/* Semantic Sensor Targets */}
        <div className="bg-slate-950/80 border border-slate-800 rounded-lg p-2.5">
          <div className="flex items-center justify-between mb-2">
            <span className="font-semibold text-slate-300 flex items-center gap-1 text-[11px]">
              <Layers className="w-3.5 h-3.5 text-emerald-400" />
              Semantic Targets
            </span>
            <span className="text-[10px] font-mono text-slate-400">
              {drone.semanticHits.length} DETECTED
            </span>
          </div>

          <div className="space-y-1.5 max-h-36 overflow-y-auto">
            {drone.semanticHits.length === 0 ? (
              <div className="text-[11px] text-slate-500 italic text-center py-2">
                No targets in sensory range
              </div>
            ) : (
              drone.semanticHits.slice(0, 5).map((hit, idx) => (
                <div
                  key={idx}
                  className="flex items-center justify-between bg-slate-900 px-2 py-1.5 rounded border border-slate-800 text-[11px] font-mono"
                >
                  <div className="flex items-center gap-1.5">
                    <span
                      className={`w-2 h-2 rounded-full ${
                        hit.entityType === 'BOMB'
                          ? 'bg-rose-500'
                          : hit.entityType === 'DISPOSAL_CENTER'
                          ? 'bg-emerald-500'
                          : 'bg-sky-500'
                      }`}
                    />
                    <span className="font-bold text-slate-200">
                      {hit.entityType === 'BOMB'
                        ? `BOMB #${hit.id}`
                        : hit.entityType === 'DISPOSAL_CENTER'
                        ? 'DISPOSAL ZONE'
                        : `DRONE #${hit.id}`}
                    </span>
                  </div>

                  <div className="text-slate-400 text-[10px]">
                    {Math.round(hit.distance)}px • {Math.round((hit.angle * 180) / Math.PI)}°
                  </div>
                </div>
              ))
            )}
          </div>
        </div>

        {/* Manual Keyboard hints when in manual pilot mode */}
        {isManualMode && (
          <div className="bg-amber-950/30 border border-amber-800/60 rounded-lg p-2.5 text-amber-200 text-[11px]">
            <div className="font-bold flex items-center gap-1 mb-1 text-amber-400">
              <Activity className="w-3.5 h-3.5" /> Manual Keyboard Pilot
            </div>
            <div className="grid grid-cols-2 gap-1 text-[10px] font-mono">
              <div>W / Up: Forward</div>
              <div>S / Down: Reverse</div>
              <div>A / Left: Turn Left</div>
              <div>D / Right: Turn Right</div>
              <div>Q / E: Strafe</div>
              <div>Space: Grasp / Place</div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};
