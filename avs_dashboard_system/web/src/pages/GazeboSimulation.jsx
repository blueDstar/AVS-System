import React from 'react';
import { wsService } from '../services/websocket';
import { useGazebo } from '../services/store';
import { Box, Play, Square, RotateCcw, Monitor } from 'lucide-react';

export default function GazeboSimulation() {
  const gazebo = useGazebo();
  
  const status = gazebo.gazebo_status || 'stopped';
  const currentWorld = gazebo.current_world;
  const targetRuntime = gazebo.target_runtime;
  const worlds = gazebo.available_worlds || {};

  const startGazebo = (worldName) => {
    wsService.send('start_gazebo', { world: worldName });
  };

  const stopGazebo = () => {
    wsService.send('stop_gazebo', {});
  };

  const resetGazebo = () => {
    wsService.send('reset_gazebo', {});
  };

  const isSim = targetRuntime === 'gazebo';

  return (
    <div className="page-container max-w-4xl mx-auto">
      <div className="flex justify-between items-end mb-6">
        <div>
          <h1 className="page-title">Gazebo Simulation</h1>
          <p className="page-subtitle mb-0">Manage simulation environments for algorithm benchmarking.</p>
        </div>
        
        <div className="flex gap-2">
          {status === 'running' && (
            <button onClick={resetGazebo} className="btn btn-secondary flex items-center gap-2">
              <RotateCcw size={16} /> RESET WORLD
            </button>
          )}
          {status !== 'stopped' && (
            <button onClick={stopGazebo} className="btn btn-danger flex items-center gap-2">
              <Square size={16} fill="currentColor" /> STOP GAZEBO
            </button>
          )}
        </div>
      </div>

      <div className="card mb-6 bg-[rgba(255,255,255,0.02)] border-[rgba(255,255,255,0.05)]">
        <h3 className="font-semibold flex items-center gap-2 mb-4 border-b border-[rgba(255,255,255,0.1)] pb-2">
          <Monitor size={18} /> Runtime Status
        </h3>
        <div className="grid-3">
          <div>
            <div className="text-xs text-muted uppercase mb-1">Gazebo Status</div>
            <div className={`font-bold uppercase ${status === 'running' ? 'text-success' : status === 'error' ? 'text-danger' : 'text-text'}`}>
              {status}
            </div>
          </div>
          <div>
            <div className="text-xs text-muted uppercase mb-1">Target Runtime</div>
            <div className={`font-bold ${isSim ? 'text-warning' : 'text-info'}`}>
              {isSim ? 'SIMULATION' : 'REAL ROBOT'}
            </div>
          </div>
          <div>
            <div className="text-xs text-muted uppercase mb-1">Active World</div>
            <div className="font-bold text-text">
              {currentWorld ? (worlds[currentWorld]?.display_name || currentWorld) : 'None'}
            </div>
          </div>
        </div>
        
        {isSim && status === 'running' && (
           <div className="mt-4 p-3 bg-warning/10 border border-warning/30 rounded text-sm text-warning font-semibold flex items-center justify-center">
             Simulation mode active. Commands to the real robot are blocked.
           </div>
        )}
      </div>

      <h3 className="font-semibold mb-4">Available Worlds</h3>
      <div className="grid-2">
        {Object.entries(worlds).map(([worldId, world]) => (
          <div key={worldId} className="card flex flex-col hover:border-accent transition-colors">
            <div className="flex items-center gap-3 mb-2">
              <div className="p-2 bg-[rgba(255,255,255,0.05)] rounded text-accent">
                <Box size={24} />
              </div>
              <h4 className="font-bold text-lg">{world.display_name}</h4>
            </div>
            
            <p className="text-sm text-muted flex-1 mb-4">
              Recommended Speed: <span className="font-mono text-accent">{world.recommended_speed} m/s</span>
            </p>
            
            <button 
              onClick={() => startGazebo(worldId)}
              disabled={status !== 'stopped'}
              className="btn btn-primary flex justify-center items-center gap-2 w-full"
            >
              <Play size={16} fill="currentColor" /> LAUNCH WORLD
            </button>
          </div>
        ))}
      </div>
    </div>
  );
}
