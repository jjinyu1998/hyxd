import React from 'react';
import { X, ShieldAlert, Flame, Crosshair, Cpu, Check, BookOpen } from 'lucide-react';

interface ManualHelpModalProps {
  isOpen: boolean;
  onClose: () => void;
}

export const ManualHelpModal: React.FC<ManualHelpModalProps> = ({ isOpen, onClose }) => {
  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/70 backdrop-blur-sm p-4">
      <div className="bg-slate-900 border border-slate-700 rounded-xl shadow-2xl max-w-2xl w-full max-h-[85vh] flex flex-col overflow-hidden text-slate-200">
        {/* Modal Header */}
        <div className="flex items-center justify-between p-4 border-b border-slate-800 bg-slate-950">
          <div className="flex items-center gap-2">
            <BookOpen className="w-5 h-5 text-sky-400" />
            <h2 className="font-bold text-sm tracking-wide text-white uppercase">
              Swarm Rescue Mission Guide & Flight Manual
            </h2>
          </div>
          <button
            onClick={onClose}
            className="p-1 rounded text-slate-400 hover:text-white hover:bg-slate-800 transition"
          >
            <X className="w-5 h-5" />
          </button>
        </div>

        {/* Modal Content */}
        <div className="p-5 overflow-y-auto space-y-5 text-xs">
          {/* Mission Overview */}
          <div>
            <h3 className="font-bold text-slate-100 uppercase text-xs tracking-wider mb-1 flex items-center gap-1.5 text-sky-400">
              <Cpu className="w-4 h-4" /> 1. Mission Objectives
            </h3>
            <p className="text-slate-300 leading-relaxed">
              In <strong>Swarm-Rescue</strong>, a coordinated fleet of autonomous drones is deployed into unknown indoor disaster environments (such as collapsed buildings or underground labyrinths) to locate hazard bombs, neutralize threats, and deliver them to designated disposal zones.
            </p>
          </div>

          {/* Red & Blue Team Rules */}
          <div className="grid grid-cols-1 md:grid-cols-2 gap-3">
            <div className="bg-slate-950 p-3 rounded-lg border border-blue-900/40 space-y-1.5">
              <div className="font-bold text-blue-400 flex items-center gap-1.5">
                <ShieldAlert className="w-4 h-4" /> Blue Team (Rescue Mode)
              </div>
              <ul className="space-y-1 text-slate-300 list-disc list-inside">
                <li>Explore the map and detect scattered bombs.</li>
                <li>Grasp bombs with onboard claw actuators.</li>
                <li>Transport and drop them inside the Disposal Center.</li>
                <li><strong>Scoring (100 pts)</strong>: 60% Disposed bombs, 20% Fleet health, 20% Time efficiency.</li>
              </ul>
            </div>

            <div className="bg-slate-950 p-3 rounded-lg border border-rose-900/40 space-y-1.5">
              <div className="font-bold text-rose-400 flex items-center gap-1.5">
                <Flame className="w-4 h-4" /> Red Team (Place Mode)
              </div>
              <ul className="space-y-1 text-slate-300 list-disc list-inside">
                <li>Deploy tactical bombs deep inside rooms.</li>
                <li>Build SLAM occupancy grid of map walls.</li>
                <li>Maximize blast destruction against internal structures.</li>
                <li><strong>Scoring (100 pts)</strong>: 50% Mapping accuracy, 20% Placement difficulty, 20% Wall blast, 10% Time.</li>
              </ul>
            </div>
          </div>

          {/* Sensors & Interference Zones */}
          <div>
            <h3 className="font-bold text-slate-100 uppercase text-xs tracking-wider mb-2 text-sky-400">
              2. Onboard Sensors & Environmental Interference
            </h3>
            <div className="grid grid-cols-2 gap-2 text-slate-300">
              <div className="bg-slate-950 p-2.5 rounded border border-slate-800">
                <span className="font-bold text-sky-300 block mb-0.5">Lidar Raycaster (360°)</span>
                Measures exact distances to walls with 28 laser beams.
              </div>
              <div className="bg-slate-950 p-2.5 rounded border border-slate-800">
                <span className="font-bold text-emerald-300 block mb-0.5">Semantic Sensor</span>
                Classifies detected entities (Wall, Bomb, Disposal Zone, Drone).
              </div>
              <div className="bg-slate-950 p-2.5 rounded border border-slate-800">
                <span className="font-bold text-amber-300 block mb-0.5">⚡ GPS Jamming Zones</span>
                Orange dashed areas block GPS satellite position fixes.
              </div>
              <div className="bg-slate-950 p-2.5 rounded border border-slate-800">
                <span className="font-bold text-purple-300 block mb-0.5">📡 EM Interference Zones</span>
                Purple dashed areas disable inter-drone radio communications.
              </div>
            </div>
          </div>

          {/* Flight Controls (Manual Pilot) */}
          <div>
            <h3 className="font-bold text-slate-100 uppercase text-xs tracking-wider mb-2 flex items-center gap-1.5 text-amber-400">
              <Crosshair className="w-4 h-4" /> 3. Keyboard Flight Controls (Manual Mode)
            </h3>
            <div className="bg-slate-950 p-3 rounded-lg border border-slate-800 grid grid-cols-2 gap-2 font-mono text-[11px]">
              <div>
                <strong className="text-amber-300">W / ↑</strong>: Forward Thrust
              </div>
              <div>
                <strong className="text-amber-300">S / ↓</strong>: Reverse Thrust
              </div>
              <div>
                <strong className="text-amber-300">A / ←</strong>: Rotate Left
              </div>
              <div>
                <strong className="text-amber-300">D / →</strong>: Rotate Right
              </div>
              <div>
                <strong className="text-amber-300">Q / E</strong>: Lateral Strafe
              </div>
              <div>
                <strong className="text-amber-300">Spacebar</strong>: Toggle Grasper / Deploy Bomb
              </div>
              <div>
                <strong className="text-amber-300">0 - 9</strong>: Select Active Drone
              </div>
              <div>
                <strong className="text-amber-300">P / Space</strong>: Play / Pause Sim (in auto)
              </div>
            </div>
          </div>
        </div>

        {/* Modal Footer */}
        <div className="p-3 border-t border-slate-800 bg-slate-950 flex justify-end">
          <button
            onClick={onClose}
            className="px-4 py-1.5 rounded-lg bg-sky-600 hover:bg-sky-500 text-white font-medium text-xs transition"
          >
            Understood, Back to Flight Deck
          </button>
        </div>
      </div>
    </div>
  );
};
