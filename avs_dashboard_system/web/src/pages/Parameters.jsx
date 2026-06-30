import React, { useState } from 'react';
import { wsService } from '../services/websocket';
import { Sliders, Save, RotateCcw, ChevronDown, ChevronRight } from 'lucide-react';

/** Parameter group data */
const PARAM_GROUPS = [
  {
    id: 'cmd_vel_mux',
    label: 'cmd_vel Mux (Safety)',
    icon: '🛡️',
    node: 'cmd_vel_mux_control',
    params: [
      { key: 'v_max',                type: 'float', default: 0.3,  min: 0.01, max: 1.0,  step: 0.01, unit: 'm/s',   label: 'Max Linear Vel (v_max)' },
      { key: 'v_min',                type: 'float', default: -0.1, min: -0.5, max: 0.0,  step: 0.01, unit: 'm/s',   label: 'Min Linear Vel (v_min)' },
      { key: 'omega_max',            type: 'float', default: 2.0,  min: 0.1,  max: 5.0,  step: 0.1,  unit: 'rad/s', label: 'Max Angular Vel (ω_max)' },
      { key: 'max_accel',            type: 'float', default: 0.5,  min: 0.01, max: 2.0,  step: 0.05, unit: 'm/s²',  label: 'Max Acceleration' },
      { key: 'max_alpha',            type: 'float', default: 1.0,  min: 0.1,  max: 5.0,  step: 0.1,  unit: 'rad/s²',label: 'Max Angular Accel' },
      { key: 'cmd_source_timeout_s', type: 'float', default: 0.5,  min: 0.1,  max: 5.0,  step: 0.1,  unit: 's',     label: 'Command Timeout' },
    ],
  },
  {
    id: 'telemetry',
    label: 'Telemetry Aggregator',
    icon: '📡',
    node: 'telemetry_aggregator_control',
    params: [
      { key: 'publish_hz',          type: 'float', default: 10.0, min: 1.0, max: 50.0, step: 1.0, unit: 'Hz', label: 'Publish Rate' },
      { key: 'timeout_odom_s',      type: 'float', default: 1.0,  min: 0.5, max: 10.0, step: 0.5, unit: 's',  label: 'Odom Timeout' },
      { key: 'timeout_imu_s',       type: 'float', default: 2.0,  min: 0.5, max: 10.0, step: 0.5, unit: 's',  label: 'IMU Timeout' },
      { key: 'timeout_scan_s',      type: 'float', default: 2.0,  min: 0.5, max: 10.0, step: 0.5, unit: 's',  label: 'LiDAR Timeout' },
    ],
  },
  {
    id: 'recorder',
    label: 'Experiment Recorder',
    icon: '📊',
    node: 'experiment_recorder_control',
    params: [
      { key: 'record_hz',           type: 'float', default: 10.0, min: 1.0, max: 50.0, step: 1.0, unit: 'Hz', label: 'Recording Rate' },
      { key: 'csv_flush_interval_s',type: 'float', default: 1.0,  min: 0.5, max: 10.0, step: 0.5, unit: 's',  label: 'CSV Flush Interval' },
      { key: 'auto_plot',           type: 'bool',  default: true,  label: 'Auto-generate plots on stop' },
    ],
  },
];

function ParamGroup({ group }) {
  const [expanded, setExpanded] = useState(true);
  const [values, setValues]     = useState(
    Object.fromEntries(group.params.map(p => [p.key, p.default]))
  );
  const [dirty, setDirty]       = useState(false);

  const handleChange = (key, val) => {
    setValues(prev => ({ ...prev, [key]: val }));
    setDirty(true);
  };

  const handleApply = () => {
    // Send each param via WS (ROS set_parameters would need a service call)
    // For now we notify the user (future: implement via ros2 param set over WS)
    console.log('[params] Apply:', group.node, values);
    wsService.send('set_parameters', { node: group.node, params: values });
    setDirty(false);
  };

  const handleReset = () => {
    setValues(Object.fromEntries(group.params.map(p => [p.key, p.default])));
    setDirty(false);
  };

  return (
    <div className="card mb-4">
      <div
        className="flex items-center justify-between cursor-pointer"
        onClick={() => setExpanded(e => !e)}
      >
        <div className="flex items-center gap-3">
          <span className="text-xl">{group.icon}</span>
          <div>
            <div className="font-semibold">{group.label}</div>
            <div className="text-xs text-dim font-mono">{group.node}</div>
          </div>
        </div>
        <div className="flex items-center gap-3">
          {dirty && <span className="text-xs text-warning font-semibold">● UNSAVED</span>}
          {expanded ? <ChevronDown size={18} className="text-muted" /> : <ChevronRight size={18} className="text-muted" />}
        </div>
      </div>

      {expanded && (
        <div className="mt-4 pt-4 border-t border-[rgba(255,255,255,0.06)]">
          <div className="grid-2 gap-4 mb-6">
            {group.params.map(param => (
              <div key={param.key}>
                <label className="input-label">
                  {param.label}
                  {param.unit && <span className="text-dim ml-1 normal-case">({param.unit})</span>}
                </label>

                {param.type === 'bool' ? (
                  <label className="toggle flex items-center gap-3 mt-1">
                    <input
                      type="checkbox"
                      checked={values[param.key]}
                      onChange={e => handleChange(param.key, e.target.checked)}
                    />
                    <div className="toggle-track"><div className="toggle-thumb" /></div>
                    <span className="text-sm">{values[param.key] ? 'Enabled' : 'Disabled'}</span>
                  </label>
                ) : (
                  <div className="flex items-center gap-3 mt-1">
                    <input
                      type="range"
                      className="slider flex-1"
                      min={param.min} max={param.max} step={param.step}
                      value={values[param.key]}
                      onChange={e => handleChange(param.key, parseFloat(e.target.value))}
                    />
                    <div className="flex flex-col items-center">
                      <input
                        type="number"
                        className="input text-right font-mono"
                        style={{ width: 80 }}
                        min={param.min} max={param.max} step={param.step}
                        value={values[param.key]}
                        onChange={e => handleChange(param.key, parseFloat(e.target.value))}
                      />
                    </div>
                  </div>
                )}
              </div>
            ))}
          </div>

          <div className="flex gap-3 justify-end border-t border-[rgba(255,255,255,0.06)] pt-4">
            <button onClick={handleReset} className="btn btn-ghost flex items-center gap-2">
              <RotateCcw size={14} /> Reset Defaults
            </button>
            <button
              onClick={handleApply}
              disabled={!dirty}
              className="btn btn-primary flex items-center gap-2"
            >
              <Save size={14} /> Apply Changes
            </button>
          </div>
        </div>
      )}
    </div>
  );
}

export default function Parameters() {
  return (
    <div className="page-container max-w-4xl mx-auto">
      <div className="flex items-end justify-between mb-6">
        <div>
          <h1 className="page-title">Dynamic Parameters</h1>
          <p className="page-subtitle mb-0">Adjust ROS 2 node parameters without restarting.</p>
        </div>
        <div className="text-xs text-muted bg-[rgba(255,255,255,0.03)] px-3 py-2 rounded border border-[rgba(255,255,255,0.08)]">
          <Sliders size={14} className="inline mr-1" />
          Changes are sent to nodes via WebSocket → ros2 param set
        </div>
      </div>

      {PARAM_GROUPS.map(group => (
        <ParamGroup key={group.id} group={group} />
      ))}
    </div>
  );
}
