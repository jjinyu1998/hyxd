import React, { useState, useEffect, useRef, useCallback } from 'react';
import { SimulationEngine } from './simulation/simulationEngine';
import { ALL_MAPS } from './data/mapsData';
import { TeamMode, CommandsDict } from './types/simulation';
import { SimulationCanvas } from './components/SimulationCanvas';
import { ControlsHeader } from './components/ControlsHeader';
import { TelemetryPanel } from './components/TelemetryPanel';
import { ScoreBoard } from './components/ScoreBoard';
import { CommLog } from './components/CommLog';
import { ManualHelpModal } from './components/ManualHelpModal';
import { ExportSolutionsModal } from './components/ExportSolutionsModal';

export function App() {
  const [selectedMapId, setSelectedMapId] = useState<string>('map_01');
  const [teamMode, setTeamMode] = useState<TeamMode>('rescue');
  const [droneCount, setDroneCount] = useState<number>(10);
  const [isRunning, setIsRunning] = useState<boolean>(true);
  const [speed, setSpeed] = useState<number>(1);
  const [selectedDroneId, setSelectedDroneId] = useState<number>(0);

  // Visualization layer toggles
  const [showLidar, setShowLidar] = useState<boolean>(true);
  const [showSemantic, setShowSemantic] = useState<boolean>(true);
  const [showExploredGrid, setShowExploredGrid] = useState<boolean>(false);
  const [showTrails, setShowTrails] = useState<boolean>(true);
  const [isHelpOpen, setIsHelpOpen] = useState<boolean>(false);
  const [isSolutionsOpen, setIsSolutionsOpen] = useState<boolean>(false);

  // Keyboard inputs for manual piloting
  const keysPressed = useRef<{ [key: string]: boolean }>({});

  // Simulation Engine instance
  const engineRef = useRef<SimulationEngine | null>(null);
  if (!engineRef.current) {
    engineRef.current = new SimulationEngine(
      ALL_MAPS[selectedMapId],
      teamMode,
      droneCount
    );
  }
  const engine = engineRef.current;

  // State tick bumper to update React components
  const [, setTickCounter] = useState<number>(0);

  // Handle map change
  const handleSelectMap = (mapId: string) => {
    setSelectedMapId(mapId);
    if (ALL_MAPS[mapId]) {
      engine.setMap(ALL_MAPS[mapId]);
      setSelectedDroneId(0);
      setTickCounter((t) => t + 1);
    }
  };

  // Handle mode change
  const handleChangeMode = (mode: TeamMode) => {
    setTeamMode(mode);
    engine.setTeamMode(mode);
    setTickCounter((t) => t + 1);
  };

  // Handle drone count change
  const handleChangeDroneCount = (count: number) => {
    setDroneCount(count);
    engine.reset(count);
    setSelectedDroneId(0);
    setTickCounter((t) => t + 1);
  };

  // Play / Pause toggle
  const handleTogglePlay = () => {
    setIsRunning((prev) => {
      engine.isRunning = !prev;
      return !prev;
    });
  };

  // Single step forward
  const handleStep = () => {
    engine.step();
    setTickCounter((t) => t + 1);
  };

  // Reset
  const handleReset = () => {
    engine.reset(droneCount);
    setIsRunning(false);
    engine.isRunning = false;
    setTickCounter((t) => t + 1);
  };

  // Key listeners for manual flying
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Ignore if inside an input or modal
      if ((e.target as HTMLElement).tagName === 'INPUT' || (e.target as HTMLElement).tagName === 'SELECT') {
        return;
      }

      keysPressed.current[e.key.toLowerCase()] = true;

      // 0-9 to select drone
      if (e.key >= '0' && e.key <= '9') {
        const id = parseInt(e.key, 10);
        if (id < engine.drones.length) {
          setSelectedDroneId(id);
          engine.selectedDroneId = id;
        }
      }

      // Spacebar toggling play/pause when in auto modes
      if (e.key === ' ' && teamMode !== 'manual') {
        e.preventDefault();
        handleTogglePlay();
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      keysPressed.current[e.key.toLowerCase()] = false;
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [engine, teamMode]);

  // Main simulation tick loop
  useEffect(() => {
    if (!isRunning) return;

    const intervalTime = speed === 1 ? 30 : speed === 2 ? 16 : 8;

    const interval = setInterval(() => {
      // Update manual inputs from keys if manual mode
      if (teamMode === 'manual') {
        const k = keysPressed.current;
        let forward = 0;
        let lateral = 0;
        let rotation = 0;
        let grasper = 0;
        let placeBomb = false;

        if (k['w'] || k['arrowup']) forward += 0.85;
        if (k['s'] || k['arrowdown']) forward -= 0.65;
        if (k['a'] || k['arrowleft']) rotation -= 0.85;
        if (k['d'] || k['arrowright']) rotation += 0.85;
        if (k['q']) lateral -= 0.65;
        if (k['e']) lateral += 0.65;
        if (k[' ']) {
          grasper = 1;
          placeBomb = true;
        }

        engine.manualInputs = { forward, lateral, rotation, grasper, placeBomb };
      }

      engine.step();
      setTickCounter((t) => (t + 1) % 100000);
    }, intervalTime);

    return () => clearInterval(interval);
  }, [isRunning, speed, engine, teamMode]);

  const scores = engine.getScores();

  return (
    <div className="flex flex-col w-screen h-screen overflow-hidden bg-slate-950 font-sans text-slate-100 select-none">
      {/* Top Header Controls */}
      <ControlsHeader
        isRunning={isRunning}
        onTogglePlay={handleTogglePlay}
        onStep={handleStep}
        onReset={handleReset}
        speed={speed}
        onChangeSpeed={setSpeed}
        selectedMapId={selectedMapId}
        onSelectMap={handleSelectMap}
        teamMode={teamMode}
        onChangeMode={handleChangeMode}
        droneCount={droneCount}
        onChangeDroneCount={handleChangeDroneCount}
        showLidar={showLidar}
        onToggleLidar={() => setShowLidar(!showLidar)}
        showSemantic={showSemantic}
        onToggleSemantic={() => setShowSemantic(!showSemantic)}
        showExploredGrid={showExploredGrid}
        onToggleExploredGrid={() => setShowExploredGrid(!showExploredGrid)}
        showTrails={showTrails}
        onToggleTrails={() => setShowTrails(!showTrails)}
        onOpenHelp={() => setIsHelpOpen(true)}
        onOpenSolutions={() => setIsSolutionsOpen(true)}
      />

      {/* Main Workspace */}
      <div className="flex flex-1 overflow-hidden">
        {/* Left Column: Canvas + Comm Log */}
        <div className="flex flex-col flex-1 h-full min-w-0">
          <div className="flex-1 min-h-0 relative">
            <SimulationCanvas
              engine={engine}
              selectedDroneId={selectedDroneId}
              onSelectDrone={(id) => {
                setSelectedDroneId(id);
                engine.selectedDroneId = id;
              }}
              showLidar={showLidar}
              showSemantic={showSemantic}
              showExploredGrid={showExploredGrid}
              showTrails={showTrails}
            />
          </div>

          {/* Comm & Dispatch Log */}
          <CommLog
            messages={engine.messages}
            onClear={() => {
              engine.messages = [];
              setTickCounter((t) => t + 1);
            }}
          />
        </div>

        {/* Right Column: Fleet Telemetry & Sensors */}
        <div className="w-80 h-full shrink-0">
          <TelemetryPanel
            drones={engine.drones}
            selectedDroneId={selectedDroneId}
            onSelectDrone={(id) => {
              setSelectedDroneId(id);
              engine.selectedDroneId = id;
            }}
            teamMode={teamMode}
            isManualMode={teamMode === 'manual'}
          />
        </div>
      </div>

      {/* Bottom Competition Scoreboard */}
      <ScoreBoard scores={scores} teamMode={teamMode} />

      {/* Guide Modal */}
      <ManualHelpModal isOpen={isHelpOpen} onClose={() => setIsHelpOpen(false)} />

      {/* Solutions Export Modal */}
      <ExportSolutionsModal
        isOpen={isSolutionsOpen}
        onClose={() => setIsSolutionsOpen(false)}
      />
    </div>
  );
}

export default App;
