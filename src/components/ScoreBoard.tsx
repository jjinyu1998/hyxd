import React, { useEffect } from 'react';
import confetti from 'canvas-confetti';
import {
  Trophy,
  CheckCircle2,
  Clock,
  Heart,
  Target,
  Sparkles,
  Map,
  Flame,
} from 'lucide-react';
import { SimulationScores, TeamMode } from '../types/simulation';

interface ScoreBoardProps {
  scores: SimulationScores;
  teamMode: TeamMode;
}

export const ScoreBoard: React.FC<ScoreBoardProps> = ({ scores, teamMode }) => {
  // Fire confetti if mission completed (all bombs disposed in rescue)
  useEffect(() => {
    if (
      teamMode === 'rescue' &&
      scores.disposedBombsCount > 0 &&
      scores.disposedBombsCount === scores.totalBombsCount
    ) {
      confetti({
        particleCount: 60,
        spread: 60,
        origin: { y: 0.8 },
      });
    }
  }, [scores.disposedBombsCount, scores.totalBombsCount, teamMode]);

  const formatTime = (secs: number) => {
    const m = Math.floor(secs / 60);
    const s = secs % 60;
    return `${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
  };

  const isRescue = teamMode === 'rescue' || teamMode === 'manual';

  return (
    <div className="bg-slate-900 border-t border-slate-800 px-4 py-2 flex flex-wrap items-center justify-between gap-4 text-xs">
      {/* Timestep and Walltime clock */}
      <div className="flex items-center gap-4">
        <div className="flex items-center gap-2">
          <Clock className="w-4 h-4 text-sky-400" />
          <div>
            <div className="text-[10px] text-slate-400 uppercase font-mono">
              Simulation Clock
            </div>
            <div className="font-mono font-bold text-slate-200">
              {scores.timeElapsedTimesteps} / {scores.maxTimesteps} steps ({formatTime(scores.timeElapsedSeconds)})
            </div>
          </div>
        </div>

        <div className="h-6 w-px bg-slate-800" />

        {/* Primary metric depending on mode */}
        {isRescue ? (
          <div className="flex items-center gap-3">
            <div className="flex items-center gap-1.5">
              <CheckCircle2 className="w-4 h-4 text-emerald-400" />
              <div>
                <div className="text-[10px] text-slate-400 uppercase">
                  Bombs Disposed (60%)
                </div>
                <div className="font-mono font-bold text-emerald-400">
                  {scores.disposedBombsCount} / {scores.totalBombsCount} ({scores.disposedPercent}%)
                </div>
              </div>
            </div>

            <div className="flex items-center gap-1.5">
              <Heart className="w-4 h-4 text-rose-400" />
              <div>
                <div className="text-[10px] text-slate-400 uppercase">
                  Fleet Health (20%)
                </div>
                <div className="font-mono font-bold text-slate-200">
                  {scores.averageDroneHealth}%
                </div>
              </div>
            </div>

            <div className="flex items-center gap-1.5">
              <Target className="w-4 h-4 text-blue-400" />
              <div>
                <div className="text-[10px] text-slate-400 uppercase">
                  Time Bonus (20%)
                </div>
                <div className="font-mono font-bold text-slate-200">
                  {scores.timeEfficiencyScore} pts
                </div>
              </div>
            </div>
          </div>
        ) : (
          <div className="flex items-center gap-3">
            <div className="flex items-center gap-1.5">
              <Map className="w-4 h-4 text-purple-400" />
              <div>
                <div className="text-[10px] text-slate-400 uppercase">
                  Mapping (50%)
                </div>
                <div className="font-mono font-bold text-purple-400">
                  {scores.mapExplorationScore}%
                </div>
              </div>
            </div>

            <div className="flex items-center gap-1.5">
              <Flame className="w-4 h-4 text-rose-400" />
              <div>
                <div className="text-[10px] text-slate-400 uppercase">
                  Placement (20%)
                </div>
                <div className="font-mono font-bold text-rose-400">
                  {scores.bombPlacementScore}%
                </div>
              </div>
            </div>

            <div className="flex items-center gap-1.5">
              <Target className="w-4 h-4 text-amber-400" />
              <div>
                <div className="text-[10px] text-slate-400 uppercase">
                  Blast Coverage (20%)
                </div>
                <div className="font-mono font-bold text-amber-400">
                  {scores.wallDestructionScore}%
                </div>
              </div>
            </div>
          </div>
        )}
      </div>

      {/* Grand Score Display */}
      <div className="flex items-center gap-3 bg-slate-950 px-3 py-1.5 rounded-lg border border-slate-800">
        <Trophy className="w-5 h-5 text-amber-400" />
        <div>
          <div className="text-[10px] text-slate-400 uppercase tracking-wider font-semibold">
            {isRescue ? 'Rescue Evaluation Score' : 'Place Mission Score'}
          </div>
          <div className="text-base font-mono font-extrabold text-amber-400">
            {isRescue ? scores.rescueTotalScore : scores.placeTotalScore}
            <span className="text-xs text-slate-500 font-normal"> / 100</span>
          </div>
        </div>

        {/* Status Pill */}
        <div className="ml-2">
          {isRescue && scores.disposedBombsCount === scores.totalBombsCount ? (
            <span className="flex items-center gap-1 bg-emerald-500/20 text-emerald-400 font-bold px-2 py-0.5 rounded text-[10px] border border-emerald-500/40">
              <Sparkles className="w-3 h-3" /> ALL BOMBS CLEARED
            </span>
          ) : (
            <span className="bg-slate-800 text-slate-400 px-2 py-0.5 rounded text-[10px] font-mono">
              IN PROGRESS
            </span>
          )}
        </div>
      </div>
    </div>
  );
};
