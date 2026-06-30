import React from 'react';
import { useProcesses, useLogs } from '../services/store';
import { wsService } from '../services/websocket';
import { Settings as SettingsIcon, Terminal, Server } from 'lucide-react';
import { formatNumber } from '../utils/formatters';

export default function Settings() {
  const processesStatus = useProcesses().processes || [];
  const logs = useLogs();

  const handleProcessCmd = (action, name) => {
    wsService.send(action + '_process', { name });
  };

  return (
    <div className="page-container flex flex-col h-full max-w-6xl mx-auto">
      <div className="shrink-0 mb-6">
        <h1 className="page-title">System Settings & Infrastructure</h1>
        <p className="page-subtitle mb-0">Manage background processes and view raw logs.</p>
      </div>

      <div className="grid-2 flex-1 min-h-0 gap-6 pb-6">
        
        {/* Process Manager */}
        <div className="card flex flex-col">
          <h3 className="font-semibold flex items-center gap-2 mb-4 border-b border-[rgba(255,255,255,0.1)] pb-2">
            <Server size={18} /> System Processes
          </h3>
          
          <div className="flex-1 overflow-y-auto">
            {processesStatus.map(proc => (
              <React.Fragment key={proc.name}>
              <div className="p-3 mb-2 mx-2 mt-2 bg-[rgba(255,255,255,0.02)] border border-[rgba(255,255,255,0.05)] rounded flex justify-between items-center hover:border-[rgba(255,255,255,0.1)] transition-colors">
                <div>
                  <div className="font-semibold text-sm">{proc.description} <span className="text-xs font-normal text-dim ml-2 font-mono">({proc.name})</span></div>
                  <div className="flex gap-4 mt-1 text-xs">
                    <span className={proc.running ? 'text-success font-bold' : (proc.status === 'error' ? 'text-danger font-bold' : 'text-muted font-bold')}>
                      {proc.status.toUpperCase()}
                    </span>
                    {proc.running && (
                      <>
                        <span className="text-dim">PID: {proc.pid}</span>
                        <span className="text-dim">CPU: {proc.cpu_percent}%</span>
                        <span className="text-dim">RAM: {proc.ram_mb} MB</span>
                      </>
                    )}
                  </div>
                </div>
                
                <div className="flex gap-2">
                  {proc.running ? (
                    <>
                      <button onClick={() => handleProcessCmd('restart', proc.name)} className="btn btn-sm btn-secondary">Restart</button>
                      <button onClick={() => handleProcessCmd('stop', proc.name)} className="btn btn-sm btn-danger">Stop</button>
                    </>
                  ) : (
                    <button onClick={() => handleProcessCmd('start', proc.name)} className="btn btn-sm btn-primary">Start</button>
                  )}
                </div>
              </div>
              {/* Process Logs */}
              {proc.recent_log && proc.recent_log.length > 0 && (
                <div className="bg-black text-[10px] text-dim font-mono p-2 rounded mx-3 mb-4 -mt-2 border border-[rgba(255,255,255,0.05)] break-all max-h-32 overflow-y-auto">
                  {proc.recent_log.map((line, i) => (
                    <div key={i}>{line}</div>
                  ))}
                </div>
              )}
            </React.Fragment>
            ))}
          </div>
        </div>

        {/* Live Logs */}
        <div className="card flex flex-col bg-[#0a0e17]">
          <h3 className="font-semibold flex items-center gap-2 mb-2 border-b border-[rgba(255,255,255,0.1)] pb-2">
            <Terminal size={18} /> Application Logs
          </h3>
          
          <div className="flex-1 overflow-y-auto font-mono text-xs flex flex-col gap-1 p-2 bg-black rounded border border-[rgba(255,255,255,0.05)]">
            {logs.length === 0 && <div className="text-dim text-center py-4">No logs yet...</div>}
            {logs.map((log, i) => {
              const timeStr = new Date(log.time * 1000).toLocaleTimeString([], { hour12: false });
              let colorClass = 'text-muted';
              if (log.level === 'warn') colorClass = 'text-warning';
              if (log.level === 'error') colorClass = 'text-danger';
              if (log.level === 'info') colorClass = 'text-info';
              
              return (
                <div key={i} className="flex gap-3 hover:bg-[rgba(255,255,255,0.05)] px-1 rounded">
                  <span className="text-dim w-16 shrink-0">{timeStr}</span>
                  <span className={`${colorClass} uppercase w-10 shrink-0`}>[{log.level}]</span>
                  <span className="text-[var(--color-text)] break-all">{log.msg}</span>
                </div>
              );
            })}
          </div>
        </div>

      </div>
    </div>
  );
}
