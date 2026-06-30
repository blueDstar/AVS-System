import React, { useState } from 'react';
import { wsService } from '../services/websocket';
import { useActiveCtrl, useControllers, useProcesses, useRosIntrospection } from '../services/store';
import { CONTROLLERS } from '../utils/constants';
import ConfirmDialog from '../components/Common/ConfirmDialog';
import { Cpu, Power, Settings2, Play, Square, AlertTriangle } from 'lucide-react';

// Map Mux controller names to Process Manager process names
const CTRL_TO_PROCESS_MAP = {
  'main_pd': 'main_following_pd',
  'cascade_pd': 'cascade_pd',
  'manual': 'pure_control_monitor'
};

export default function Controller() {
  const activeCtrl = useActiveCtrl();
  const rawControllers = useControllers().controllers;
  const ctrlList = rawControllers?.length > 0 
    ? rawControllers 
    : Object.keys(CONTROLLERS).map(k => ({ name: k }));
    
  const processesStatus = useProcesses()?.processes || [];
  const rosIntrospection = useRosIntrospection() || {};
  
  // Check for dangerous /cmd_vel publishers
  const cmdVelPubs = rosIntrospection.cmd_vel_publishers || [];
  const unauthorizedPubs = cmdVelPubs.filter(p => !p.node.includes('cmd_vel_mux'));
  const hasUnauthorizedPubs = unauthorizedPubs.length > 0;
  
  const [confirmOpen, setConfirmOpen] = useState(false);
  const [pendingCtrl, setPendingCtrl] = useState(null);

  const handleSwitchRequest = (ctrlName) => {
    if (ctrlName === activeCtrl) return;
    setPendingCtrl(ctrlName);
    setConfirmOpen(true);
  };

  const executeSwitch = () => {
    if (pendingCtrl) {
      wsService.send('switch_controller', { name: pendingCtrl });
    }
    setConfirmOpen(false);
    setPendingCtrl(null);
  };

  const stopAll = () => {
    wsService.send('stop_controller', {});
  };
  
  const handleProcessCmd = (action, name) => {
    wsService.send(action + '_process', { name });
  };

  return (
    <div className="page-container max-w-6xl mx-auto">
      <div className="flex justify-between items-end mb-6">
        <div>
          <h1 className="page-title">Controllers</h1>
          <p className="page-subtitle mb-0">Multi-layer Process & Mux Management</p>
        </div>
        
        {activeCtrl !== 'off' && (
          <button onClick={stopAll} className="btn btn-danger flex items-center gap-2">
            <Square size={16} fill="currentColor" /> STOP MUX
          </button>
        )}
      </div>

      {hasUnauthorizedPubs && (
        <div className="mb-6 p-4 bg-red-900/40 border border-red-500 rounded text-red-200 flex gap-4 items-center">
          <AlertTriangle size={32} className="text-red-500 shrink-0" />
          <div>
            <h4 className="font-bold text-red-400">WARNING: Multiple /cmd_vel publishers detected!</h4>
            <p className="text-sm opacity-90">
              The following nodes are bypassing the safety mux and publishing directly to /cmd_vel:
              <strong className="ml-2 bg-black/30 px-2 py-1 rounded">{unauthorizedPubs.map(p => p.node).join(', ')}</strong>
            </p>
            <p className="text-xs mt-1 text-red-300">
              Please stop these processes via Node Manager before activating a controller to prevent hardware conflict.
            </p>
          </div>
        </div>
      )}

      <div className="grid grid-cols-1 md:grid-cols-2 xl:grid-cols-3 gap-6">
        {ctrlList.filter(c => c.name !== 'off' && c.name !== 'simulation').map((ctrlObj) => {
          const ctrlId = ctrlObj.name;
          const info = CONTROLLERS[ctrlId] || { label: ctrlId, color: 'info' };
          const isActive = activeCtrl === ctrlId;
          
          const procName = CTRL_TO_PROCESS_MAP[ctrlId];
          const procObj = processesStatus.find(p => p.name === procName) || {};
          const isProcRunning = procObj.running;
          
          return (
            <div 
              key={ctrlId} 
              className={`card flex flex-col relative overflow-hidden transition-all duration-300 ${isActive ? 'ring-2 ring-accent border-accent' : 'hover:border-[rgba(255,255,255,0.2)]'}`}
            >
              {isActive && (
                <div className="absolute top-0 right-0 left-0 h-1 bg-accent shadow-[0_0_10px_var(--color-accent)] animate-pulse" />
              )}
              
              <div className="flex items-start justify-between mb-4">
                <div className="flex items-center gap-3">
                  <div className={`p-2 rounded bg-[rgba(255,255,255,0.05)] ${isActive ? 'text-accent' : 'text-muted'}`}>
                    <Cpu size={24} />
                  </div>
                  <div>
                    <h3 className="font-bold text-base text-text leading-tight">{info.label}</h3>
                    <div className="text-xs text-dim font-mono">{ctrlId}</div>
                  </div>
                </div>
                {isActive && (
                  <span className="badge badge-ok">ACTIVE IN MUX</span>
                )}
              </div>

              <div className="text-sm text-muted mb-4 flex-1">
                {ctrlId === 'main_pd' && 'Standard PD controller for lane following using single lookahead point.'}
                {ctrlId === 'cascade_pd' && 'Dual-loop PD controller. Outer loop for heading, inner loop for angular velocity.'}
                {ctrlId === 'pd_lidar' && 'PD controller augmented with LiDAR obstacle avoidance.'}
                {ctrlId === 'backstepping_pd' && 'Non-linear backstepping approach for tight cornering.'}
                {ctrlId === 'manual' && 'Direct teleoperation monitor.'}
              </div>
              
              <div className="bg-[rgba(0,0,0,0.2)] p-3 rounded mb-4">
                <div className="flex justify-between items-center mb-2">
                  <span className="text-xs text-dim">Layer 1: Process</span>
                  <span className={`text-xs font-bold ${isProcRunning ? 'text-success' : 'text-danger'}`}>
                    {isProcRunning ? 'RUNNING' : 'STOPPED'}
                  </span>
                </div>
                <div className="flex gap-2">
                  {isProcRunning ? (
                    <button onClick={() => handleProcessCmd('stop', procName)} className="btn btn-sm btn-danger flex-1">Stop Process</button>
                  ) : (
                    <button onClick={() => handleProcessCmd('start', procName)} className="btn btn-sm btn-secondary flex-1" disabled={!procName}>Start Process</button>
                  )}
                </div>
                {procName && !isProcRunning && isActive && (
                  <div className="mt-2 text-xs text-danger flex items-center gap-1">
                    <AlertTriangle size={12} /> Active in Mux but Process is STOPPED!
                  </div>
                )}
              </div>
              
              <div className="flex items-center gap-2 mt-auto border-t border-[rgba(255,255,255,0.05)] pt-4">
                {isActive ? (
                  <button disabled className="btn flex-1 bg-[rgba(255,255,255,0.05)] text-muted cursor-default border border-[rgba(255,255,255,0.05)]">
                    Currently Selected
                  </button>
                ) : (
                  <button 
                    onClick={() => handleSwitchRequest(ctrlId)} 
                    disabled={hasUnauthorizedPubs || (!isProcRunning && procName)}
                    className="btn btn-primary flex-1 flex justify-center items-center gap-2 font-bold"
                  >
                    <Play size={16} fill="currentColor" /> SET ACTIVE
                  </button>
                )}
                <button className="btn btn-icon btn-secondary" title="Parameters">
                  <Settings2 size={16} />
                </button>
              </div>
            </div>
          );
        })}
      </div>

      <ConfirmDialog 
        isOpen={confirmOpen}
        title="Switch Controller?"
        message={`Are you sure you want to route [${pendingCtrl}] to /cmd_vel? The current controller will be stopped.`}
        onConfirm={executeSwitch}
        onCancel={() => { setConfirmOpen(false); setPendingCtrl(null); }}
        confirmText="Activate"
        isDanger={false}
      />
    </div>
  );
}
