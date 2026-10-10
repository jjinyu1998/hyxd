import React, { useState } from 'react';
import {
  MessageSquare,
  AlertTriangle,
  Radio,
  CheckCircle,
  Info,
  Trash2,
} from 'lucide-react';
import { DroneMessage } from '../types/simulation';

interface CommLogProps {
  messages: DroneMessage[];
  onClear: () => void;
}

export const CommLog: React.FC<CommLogProps> = ({ messages, onClear }) => {
  const [filter, setFilter] = useState<'all' | 'bomb' | 'alert'>('all');

  const filteredMessages = messages.filter((m) => {
    if (filter === 'bomb') return m.type === 'bomb_found' || m.type === 'disposed';
    if (filter === 'alert') return m.type === 'jammed' || m.type === 'alert';
    return true;
  });

  const getIcon = (type: DroneMessage['type']) => {
    switch (type) {
      case 'bomb_found':
        return <Info className="w-3.5 h-3.5 text-amber-400 shrink-0" />;
      case 'disposed':
        return <CheckCircle className="w-3.5 h-3.5 text-emerald-400 shrink-0" />;
      case 'jammed':
      case 'alert':
        return <AlertTriangle className="w-3.5 h-3.5 text-rose-400 shrink-0" />;
      default:
        return <Radio className="w-3.5 h-3.5 text-sky-400 shrink-0" />;
    }
  };

  return (
    <div className="bg-slate-900 border-t border-slate-800 p-2.5 flex flex-col h-40 text-xs">
      {/* Log Header */}
      <div className="flex items-center justify-between mb-2">
        <div className="flex items-center gap-2">
          <MessageSquare className="w-3.5 h-3.5 text-sky-400" />
          <span className="font-semibold text-slate-200 uppercase tracking-wider text-[11px]">
            Drone Mesh Network & Dispatch Logs
          </span>
          <span className="text-[10px] text-slate-500 font-mono">
            ({messages.length} events)
          </span>
        </div>

        <div className="flex items-center gap-2">
          {/* Filter pills */}
          <div className="flex items-center bg-slate-950 p-0.5 rounded border border-slate-800 text-[10px]">
            <button
              onClick={() => setFilter('all')}
              className={`px-1.5 py-0.5 rounded transition ${
                filter === 'all' ? 'bg-slate-800 text-white' : 'text-slate-400'
              }`}
            >
              All
            </button>
            <button
              onClick={() => setFilter('bomb')}
              className={`px-1.5 py-0.5 rounded transition ${
                filter === 'bomb' ? 'bg-slate-800 text-amber-400' : 'text-slate-400'
              }`}
            >
              Bombs
            </button>
            <button
              onClick={() => setFilter('alert')}
              className={`px-1.5 py-0.5 rounded transition ${
                filter === 'alert' ? 'bg-slate-800 text-rose-400' : 'text-slate-400'
              }`}
            >
              Alerts
            </button>
          </div>

          <button
            onClick={onClear}
            className="text-slate-500 hover:text-slate-300 p-1"
            title="Clear Log"
          >
            <Trash2 className="w-3 h-3" />
          </button>
        </div>
      </div>

      {/* Messages stream */}
      <div className="flex-1 overflow-y-auto space-y-1 font-mono text-[11px]">
        {filteredMessages.length === 0 ? (
          <div className="text-slate-500 italic text-center py-4 text-xs">
            No radio messages recorded yet
          </div>
        ) : (
          filteredMessages.map((msg) => (
            <div
              key={msg.id}
              className="flex items-start gap-2 bg-slate-950/70 px-2 py-1 rounded border border-slate-850 hover:border-slate-700/60 transition"
            >
              <div className="mt-0.5">{getIcon(msg.type)}</div>
              <span className="text-slate-500 text-[10px] whitespace-nowrap">
                T+{msg.timestep}
              </span>
              <span className="text-sky-400 font-bold whitespace-nowrap">
                [DRONE #{msg.senderId}]
              </span>
              <span className="text-slate-300">{msg.text}</span>
            </div>
          ))
        )}
      </div>
    </div>
  );
};
