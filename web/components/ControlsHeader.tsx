import React from 'react';
import {
  Play,
  Pause,
  SkipForward,
  RotateCcw,
  Sliders,
  Radio,
  Eye,
  Crosshair,
  ShieldAlert,
  Flame,
  HelpCircle,
  FolderCheck,
} from 'lucide-react';
import { TeamMode } from '../types/simulation';
import { ALL_MAPS } from '../data/mapsData';

interface ControlsHeaderProps {
  isRunning: boolean;
  onTogglePlay: () => void;
  onStep: () => void;
  onReset: () => void;
  speed: number;
  onChangeSpeed: (s: number) => void;
  selectedMapId: string;
  onSelectMap: (mapId: string) => void;
  teamMode: TeamMode;
  onChangeMode: (mode: TeamMode) => void;
  droneCount: number;
  onChangeDroneCount: (count: number) => void;
  showLidar: boolean;
  onToggleLidar: () => void;
  showSemantic: boolean;
  onToggleSemantic: () => void;
  showExploredGrid: boolean;
  onToggleExploredGrid: () => void;
  showTrails: boolean;
  onToggleTrails: () => void;
  onOpenHelp: () => void;
  onOpenSolutions: () => void;
}

export const ControlsHeader: React.FC<ControlsHeaderProps> = ({
  isRunning,
  onTogglePlay,
  onStep,
  onReset,
  speed,
  onChangeSpeed,
  selectedMapId,
  onSelectMap,
  teamMode,
  onChangeMode,
  droneCount,
  onChangeDroneCount,
  showLidar,
  onToggleLidar,
  showSemantic,
  onToggleSemantic,
  showExploredGrid,
  onToggleExploredGrid,
  showTrails,
  onToggleTrails,
  onOpenHelp,
  onOpenSolutions,
}) => {
  return (
    <header className="bg-slate-900 border-b border-slate-800 px-4 py-2.5 flex flex-wrap items-center justify-between gap-3 shadow-md z-10">
      {/* Brand & Mode selector */}
      <div className="flex items-center gap-3">
        <div className="flex items-center gap-2">
          <div className="w-8 h-8 rounded-lg bg-sky-500/10 border border-sky-500/30 flex items-center justify-center text-sky-400">
            <Radio className="w-4 h-4 animate-pulse" />
          </div>
          <div>
            <div className="flex items-center gap-1.5">
              <h1 className="font-bold text-sm tracking-wide text-slate-100 uppercase">
                Swarm Rescue
              </h1>
              <span className="text-[10px] bg-slate-800 text-sky-400 font-mono px-1.5 py-0.5 rounded border border-slate-700">
                v2.4
              </span>
            </div>
            <p className="text-[11px] text-slate-400 leading-tight">
              Autonomous Drone Fleet Simulator
            </p>
          </div>
        </div>

        {/* Mode Buttons */}
        <div className="flex items-center bg-slate-950 p-1 rounded-lg border border-slate-800 ml-2">
          <button
            onClick={() => onChangeMode('rescue')}
            className={`flex items-center gap-1.5 px-2.5 py-1 rounded text-xs font-medium transition ${
              teamMode === 'rescue'
                ? 'bg-blue-600 text-white shadow-sm'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <ShieldAlert className="w-3.5 h-3.5 text-blue-300" />
            <span>Blue (Rescue)</span>
            <span className="text-[9px] bg-purple-950/80 text-purple-300 px-1 py-0.2 rounded border border-purple-400/40">
              P4-Hybrid
            </span>
          </button>
          <button
            onClick={() => onChangeMode('place')}
            className={`flex items-center gap-1.5 px-2.5 py-1 rounded text-xs font-medium transition ${
              teamMode === 'place'
                ? 'bg-rose-600 text-white shadow-sm'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <Flame className="w-3.5 h-3.5 text-rose-300" />
            <span>Red (Place)</span>
          </button>
          <button
            onClick={() => onChangeMode('manual')}
            className={`flex items-center gap-1.5 px-2.5 py-1 rounded text-xs font-medium transition ${
              teamMode === 'manual'
                ? 'bg-amber-600 text-white shadow-sm'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <Crosshair className="w-3.5 h-3.5 text-amber-300" />
            <span>Manual Pilot</span>
          </button>
        </div>
      </div>

      {/* Map & Drone Count Selectors */}
      <div className="flex items-center gap-2">
        <select
          value={selectedMapId}
          onChange={(e) => onSelectMap(e.target.value)}
          className="bg-slate-950 border border-slate-700 rounded-lg px-2.5 py-1 text-xs text-slate-200 font-medium focus:outline-none focus:border-sky-500"
        >
          {Object.entries(ALL_MAPS).map(([id, mapDef]) => (
            <option key={id} value={id}>
              {mapDef.name} ({mapDef.width}x{mapDef.height})
            </option>
          ))}
        </select>

        <div className="flex items-center gap-1.5 bg-slate-950 border border-slate-800 rounded-lg px-2 py-1 text-xs text-slate-300">
          <span className="text-slate-400">Drones:</span>
          <select
            value={droneCount}
            onChange={(e) => onChangeDroneCount(Number(e.target.value))}
            className="bg-transparent text-sky-400 font-mono font-bold focus:outline-none"
          >
            <option value={3}>3</option>
            <option value={5}>5</option>
            <option value={8}>8</option>
            <option value={10}>10</option>
            <option value={12}>12</option>
          </select>
        </div>
      </div>

      {/* Playback Controls */}
      <div className="flex items-center gap-2">
        <div className="flex items-center bg-slate-950 border border-slate-800 rounded-lg p-0.5">
          <button
            onClick={onTogglePlay}
            className={`p-1.5 rounded transition ${
              isRunning
                ? 'bg-amber-500/20 text-amber-400 hover:bg-amber-500/30'
                : 'bg-emerald-500/20 text-emerald-400 hover:bg-emerald-500/30'
            }`}
            title={isRunning ? 'Pause (Space)' : 'Play (Space)'}
          >
            {isRunning ? <Pause className="w-4 h-4" /> : <Play className="w-4 h-4" />}
          </button>
          <button
            onClick={onStep}
            className="p-1.5 rounded text-slate-300 hover:text-white hover:bg-slate-800 transition"
            title="Step Forward"
          >
            <SkipForward className="w-4 h-4" />
          </button>
          <button
            onClick={onReset}
            className="p-1.5 rounded text-slate-400 hover:text-rose-400 hover:bg-slate-800 transition"
            title="Reset Simulation"
          >
            <RotateCcw className="w-4 h-4" />
          </button>
        </div>

        {/* Speed presets */}
        <div className="flex items-center bg-slate-950 border border-slate-800 rounded-lg p-0.5 text-xs font-mono">
          {[1, 2, 4].map((s) => (
            <button
              key={s}
              onClick={() => onChangeSpeed(s)}
              className={`px-2 py-1 rounded transition ${
                speed === s
                  ? 'bg-sky-600 text-white font-bold'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              {s}x
            </button>
          ))}
        </div>

        {/* Sensor Visualization Toggles */}
        <div className="flex items-center gap-1 bg-slate-950 border border-slate-800 rounded-lg p-0.5 text-xs">
          <button
            onClick={onToggleLidar}
            className={`px-2 py-1 rounded transition flex items-center gap-1 ${
              showLidar ? 'bg-sky-500/20 text-sky-400' : 'text-slate-400 hover:text-slate-200'
            }`}
            title="Toggle Lidar Beam Visuals"
          >
            <Eye className="w-3 h-3" />
            <span>Lidar</span>
          </button>
          <button
            onClick={onToggleSemantic}
            className={`px-2 py-1 rounded transition flex items-center gap-1 ${
              showSemantic ? 'bg-emerald-500/20 text-emerald-400' : 'text-slate-400 hover:text-slate-200'
            }`}
            title="Toggle Semantic Sensor Lines"
          >
            <span>Semantic</span>
          </button>
          <button
            onClick={onToggleExploredGrid}
            className={`px-2 py-1 rounded transition flex items-center gap-1 ${
              showExploredGrid ? 'bg-purple-500/20 text-purple-400' : 'text-slate-400 hover:text-slate-200'
            }`}
            title="Toggle Explored SLAM Map Grid"
          >
            <span>SLAM</span>
          </button>
        </div>

        {/* Solutions Code export modal button */}
        <button
          onClick={onOpenSolutions}
          className="flex items-center gap-1.5 px-2.5 py-1 rounded-lg border border-emerald-500/40 bg-emerald-950/40 hover:bg-emerald-900/60 text-emerald-300 font-medium text-xs transition shadow-sm"
          title="查看与导出已保存的 Python 方案代码"
        >
          <FolderCheck className="w-3.5 h-3.5 text-emerald-400" />
          <span>代码方案 (.py)</span>
        </button>

        {/* Manual Help modal button */}
        <button
          onClick={onOpenHelp}
          className="p-1.5 rounded-lg border border-slate-700 bg-slate-850 text-slate-400 hover:text-sky-400 transition"
          title="Guide & Keyboard Controls"
        >
          <HelpCircle className="w-4 h-4" />
        </button>
      </div>
    </header>
  );
};
