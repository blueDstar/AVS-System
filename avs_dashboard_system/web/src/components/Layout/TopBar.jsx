import React from 'react';
import { useConnected, useTelemetry, useEmergency, useRuntime, useActiveCtrl } from '../../services/store';
import { CONTROLLERS, RUNTIME_MODES } from '../../utils/constants';
import EmergencyStop from '../Common/EmergencyStop';
import { Wifi, WifiOff, Activity, ShieldAlert, Cpu, Box } from 'lucide-react';

export default function TopBar() {
  const connected = useConnected();
  const telemetry = useTelemetry();
  const isEmergency = useEmergency();
  const runtime = useRuntime();
  const activeCtrl = useActiveCtrl();
  
  const ctrlInfo = CONTROLLERS[activeCtrl] || { label: activeCtrl, color: 'neutral' };
  const runtimeInfo = RUNTIME_MODES[runtime] || RUNTIME_MODES.real_robot;

  return (
    <header className="app-topbar flex items-center justify-between px-4 border-b" style={{ background: 'var(--color-surface)', borderColor: 'var(--color-border)' }}>
      
      {/* Left: Status Badges */}
      <div className="flex items-center gap-4">
        {/* Connection */}
        <div className={`flex items-center gap-2 text-xs font-semibold px-3 py-1.5 rounded-full border ${connected ? 'border-success/30 text-success bg-success/10' : 'border-danger/30 text-danger bg-danger/10'}`}>
          {connected ? <Wifi size={14} /> : <WifiOff size={14} />}
          {connected ? 'CONNECTED' : 'DISCONNECTED'}
        </div>

        {/* Runtime Mode */}
        {runtime === 'gazebo' && (
          <div className="sim-banner flex items-center gap-2">
            <Box size={14} /> SIMULATION MODE
          </div>
        )}
        {runtime === 'real_robot' && (
          <div className="flex items-center gap-2 text-xs font-bold px-3 py-1 text-info bg-info/10 rounded">
            REAL ROBOT (ID: {telemetry?.system?.ros_domain_id || 20})
          </div>
        )}

        {/* Active Controller */}
        <div className="flex items-center gap-2 px-3 py-1 rounded bg-[rgba(255,255,255,0.05)] border border-[rgba(255,255,255,0.1)]">
          <Cpu size={14} className="text-muted" />
          <span className="text-xs text-muted uppercase tracking-wider">CTRL:</span>
          <span className={`text-xs font-bold text-${ctrlInfo.color}`}>{ctrlInfo.label}</span>
        </div>
      </div>

      {/* Right: Emergency Stop */}
      <div className="flex items-center gap-4">
        <div className="flex items-center gap-3 text-xs text-muted">
          <span className="flex items-center gap-1" title="Telemetry Rate">
            <Activity size={14} /> {telemetry?.system?.cmd_vel_hz?.toFixed(1) || 0} Hz
          </span>
        </div>
        <EmergencyStop />
      </div>

    </header>
  );
}
