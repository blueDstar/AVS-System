import React from 'react';
import { useEmergency } from '../../services/store';
import { wsService } from '../../services/websocket';
import { AlertTriangle, Power } from 'lucide-react';

export default function EmergencyStop() {
  const isEmergency = useEmergency();

  const handleStop = () => {
    wsService.send('emergency_stop', { active: true });
  };

  const handleReset = () => {
    if (window.confirm("Are you sure you want to RESET the Emergency Stop? Make sure the environment is safe.")) {
      wsService.send('emergency_reset', {});
    }
  };

  if (isEmergency) {
    return (
      <div className="flex items-center gap-2">
        <div className="flex items-center gap-2 px-4 py-2 bg-danger text-white rounded font-bold animate-pulse shadow-[0_0_15px_rgba(239,68,68,0.5)]">
          <AlertTriangle size={18} />
          E-STOP ACTIVE
        </div>
        <button onClick={handleReset} className="btn bg-[rgba(255,255,255,0.1)] hover:bg-[rgba(255,255,255,0.2)] text-white border border-[rgba(255,255,255,0.2)]">
          RESET
        </button>
      </div>
    );
  }

  return (
    <button 
      onClick={handleStop}
      className="flex items-center gap-2 px-6 py-2 bg-danger hover:bg-[#dc2626] text-white rounded font-bold text-sm tracking-wider transition-all hover:scale-105 shadow-[0_4px_10px_rgba(239,68,68,0.3)] hover:shadow-[0_0_20px_rgba(239,68,68,0.6)]"
    >
      <Power size={18} />
      EMERGENCY STOP
    </button>
  );
}
