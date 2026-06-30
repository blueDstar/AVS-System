import React, { useState } from 'react';
import { useStore } from '../services/store';
import { FileText, Download, Play, Square, Tag } from 'lucide-react';
import { wsService } from '../services/websocket';
import { formatNumber, formatTime } from '../utils/formatters';

export default function LogsExport() {
  const store = useStore();
  const expStatus = store.experimentStatus || {};
  const isRecording = expStatus.recording;
  
  const [meta, setMeta] = useState({
    controller_name: 'main_pd',
    scenario_name: 'test_run',
    notes: ''
  });

  const [eventLabel, setEventLabel] = useState('');

  const startRecord = () => {
    wsService.send('start_experiment', meta);
  };

  const stopRecord = () => {
    wsService.send('stop_experiment', {});
  };

  const markEvent = () => {
    if (eventLabel.trim()) {
      wsService.send('mark_event', { label: eventLabel });
      setEventLabel('');
    }
  };

  return (
    <div className="page-container max-w-4xl mx-auto">
      <h1 className="page-title">Experiment Recorder & Logs</h1>
      <p className="page-subtitle">Record comprehensive CSV datasets for analysis.</p>

      <div className="grid-2 gap-6 mb-6">
        
        {/* Record Control */}
        <div className="card">
          <h3 className="font-semibold border-b border-[rgba(255,255,255,0.1)] pb-2 mb-4">
            Session Recording
          </h3>
          
          <div className="flex flex-col gap-4">
            <div>
              <label className="input-label">Controller Name</label>
              <input 
                className="input" type="text" 
                value={meta.controller_name} onChange={e=>setMeta({...meta, controller_name: e.target.value})} 
                disabled={isRecording}
              />
            </div>
            <div>
              <label className="input-label">Scenario Name</label>
              <input 
                className="input" type="text" 
                value={meta.scenario_name} onChange={e=>setMeta({...meta, scenario_name: e.target.value})} 
                disabled={isRecording}
              />
            </div>
            <div>
              <label className="input-label">Notes (Optional)</label>
              <input 
                className="input" type="text" placeholder="e.g. wet floor, max speed 0.3"
                value={meta.notes} onChange={e=>setMeta({...meta, notes: e.target.value})} 
                disabled={isRecording}
              />
            </div>
            
            <div className="mt-2 pt-4 border-t border-[rgba(255,255,255,0.05)] flex items-center justify-between">
              {isRecording ? (
                <button onClick={stopRecord} className="btn btn-danger-solid flex-1 flex justify-center gap-2">
                  <Square size={16} fill="currentColor" /> STOP RECORDING
                </button>
              ) : (
                <button onClick={startRecord} className="btn btn-primary flex-1 flex justify-center gap-2">
                  <Play size={16} fill="currentColor" /> START RECORDING
                </button>
              )}
            </div>
          </div>
        </div>

        {/* Live Status & Events */}
        <div className="card flex flex-col">
          <h3 className="font-semibold border-b border-[rgba(255,255,255,0.1)] pb-2 mb-4">
            Live Status
          </h3>
          
          <div className="bg-[rgba(0,0,0,0.2)] p-4 rounded mb-4">
            <div className="flex justify-between mb-2">
              <span className="text-muted text-sm">Status</span>
              {isRecording ? <span className="text-danger font-bold animate-pulse">RECORDING</span> : <span className="text-muted font-bold">READY</span>}
            </div>
            <div className="flex justify-between mb-2">
              <span className="text-muted text-sm">Duration</span>
              <span className="font-mono text-text">{formatTime(expStatus.duration_s)}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted text-sm">Data Rows</span>
              <span className="font-mono text-text">{formatNumber(expStatus.row_count, 0)}</span>
            </div>
          </div>
          
          <div className="mt-auto pt-4 border-t border-[rgba(255,255,255,0.05)]">
            <label className="input-label">Mark Event in Data</label>
            <div className="flex gap-2">
              <input 
                className="input flex-1" type="text" placeholder="Event label"
                value={eventLabel} onChange={e=>setEventLabel(e.target.value)}
                onKeyDown={e => e.key === 'Enter' && markEvent()}
                disabled={!isRecording}
              />
              <button onClick={markEvent} disabled={!isRecording || !eventLabel} className="btn btn-secondary">
                <Tag size={16} /> Mark
              </button>
            </div>
          </div>
        </div>

      </div>

    </div>
  );
}
