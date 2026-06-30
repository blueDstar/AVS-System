import React, { useState, useMemo } from 'react';
import { useHistory, useStore } from '../services/store';
import RealtimeChart from '../components/Charts/RealtimeChart';
import { Play, Pause, RotateCcw } from 'lucide-react';
import { useDispatch } from '../services/store';

const CHART_HEIGHT = 220;

export default function RealtimePlots() {
  const history  = useHistory();
  const dispatch = useDispatch();
  const [paused, setPaused]   = useState(false);
  const [windowS, setWindowS] = useState(30);

  const times = history.timestamps;

  const reset = () => {
    dispatch({ type: 'RESET_HISTORY' });
  };

  return (
    <div className="page-container h-full flex flex-col" style={{ minHeight: 0 }}>
      {/* Toolbar */}
      <div className="flex justify-between items-end mb-4 shrink-0">
        <div>
          <h1 className="page-title">Realtime Telemetry Plots</h1>
          <p className="page-subtitle mb-0">
            {times.length} samples • ECharts rendering at 10 Hz
          </p>
        </div>

        <div className="flex items-center gap-3">
          <div className="flex items-center gap-2 text-xs text-muted">
            <span>Window:</span>
            <select
              className="select py-1 pl-2 pr-6 text-xs"
              value={windowS}
              onChange={e => setWindowS(Number(e.target.value))}
            >
              <option value={10}>10s</option>
              <option value={30}>30s</option>
              <option value={60}>60s</option>
              <option value={120}>120s</option>
            </select>
          </div>

          <button
            onClick={() => setPaused(p => !p)}
            className={`btn btn-sm flex items-center gap-1 ${paused ? 'btn-primary' : 'btn-secondary'}`}
          >
            {paused ? <Play size={14}/> : <Pause size={14}/>}
            {paused ? 'RESUME' : 'PAUSE'}
          </button>

          <button onClick={reset} className="btn btn-sm btn-secondary flex items-center gap-1">
            <RotateCcw size={14}/> RESET
          </button>
        </div>
      </div>

      {/* Charts grid */}
      <div className="flex-1 overflow-y-auto pr-1">
        <div className="grid-2 gap-4 pb-4">

          {/* Linear Velocity */}
          <div className="card p-3">
            <div className="card-header mb-2">
              <span className="card-title">Linear Velocity — v</span>
            </div>
            <RealtimeChart
              times={times}
              windowSizeS={windowS}
              paused={paused}
              height={CHART_HEIGHT}
              yAxis={[{ type: 'value', name: 'm/s', min: -0.15, max: 0.5 }]}
              series={[
                { name: 'Cmd v',  data: history.cmd_v,  color: 'rgba(129,140,248,0.9)' },
                { name: 'Odom v', data: history.odom_v, color: 'rgba(0,212,170,0.9)' },
              ]}
            />
          </div>

          {/* Angular Velocity */}
          <div className="card p-3">
            <div className="card-header mb-2">
              <span className="card-title">Angular Velocity — ω</span>
            </div>
            <RealtimeChart
              times={times}
              windowSizeS={windowS}
              paused={paused}
              height={CHART_HEIGHT}
              yAxis={[{ type: 'value', name: 'rad/s', min: -2.5, max: 2.5 }]}
              series={[
                { name: 'Cmd ω',  data: history.cmd_omega,  color: 'rgba(129,140,248,0.9)' },
                { name: 'Odom ω', data: history.odom_omega, color: 'rgba(0,212,170,0.9)' },
                { name: 'IMU wz', data: history.imu_wz,     color: 'rgba(245,158,11,0.9)' },
              ]}
            />
          </div>

          {/* Lateral Error */}
          <div className="card p-3">
            <div className="card-header mb-2">
              <span className="card-title">Lateral Error — ε<sub>x</sub></span>
            </div>
            <RealtimeChart
              times={times}
              windowSizeS={windowS}
              paused={paused}
              height={CHART_HEIGHT}
              yAxis={[{ type: 'value', name: 'mm', min: -350, max: 350 }]}
              series={[
                { name: 'ε_x (mm)', data: history.epsilon_x_mm, color: 'rgba(239,68,68,0.9)' },
              ]}
            />
          </div>

          {/* Heading Error */}
          <div className="card p-3">
            <div className="card-header mb-2">
              <span className="card-title">Heading Error — θ</span>
            </div>
            <RealtimeChart
              times={times}
              windowSizeS={windowS}
              paused={paused}
              height={CHART_HEIGHT}
              yAxis={[{ type: 'value', name: 'rad', min: -1.2, max: 1.2 }]}
              series={[
                { name: 'θ (rad)', data: history.theta_rad, color: 'rgba(16,185,129,0.9)' },
              ]}
            />
          </div>

          {/* Wheel Reference vs Measured */}
          <div className="card p-3">
            <div className="card-header mb-2">
              <span className="card-title">Wheel Speed — Reference vs Measured</span>
            </div>
            <RealtimeChart
              times={times}
              windowSizeS={windowS}
              paused={paused}
              height={CHART_HEIGHT}
              yAxis={[{ type: 'value', name: 'rad/s', min: -4, max: 4 }]}
              series={[
                { name: 'Left Ref',  data: history.v_left_ref,  color: 'rgba(0,212,170,0.5)',  type: 'line' },
                { name: 'Left Meas', data: history.v_left_meas, color: 'rgba(0,212,170,0.95)', type: 'line' },
                { name: 'Right Ref',  data: history.v_right_ref,  color: 'rgba(245,158,11,0.5)',  type: 'line' },
                { name: 'Right Meas', data: history.v_right_meas, color: 'rgba(245,158,11,0.95)', type: 'line' },
              ]}
            />
          </div>

          {/* LiDAR Distances */}
          <div className="card p-3">
            <div className="card-header mb-2">
              <span className="card-title">LiDAR — Obstacle Distances</span>
            </div>
            <RealtimeChart
              times={times}
              windowSizeS={windowS}
              paused={paused}
              height={CHART_HEIGHT}
              yAxis={[{ type: 'value', name: 'm', min: 0, max: 5 }]}
              series={[
                { name: 'Front', data: history.front_min, color: 'rgba(239,68,68,0.9)' },
                { name: 'Left',  data: history.left_min,  color: 'rgba(16,185,129,0.9)' },
                { name: 'Right', data: history.right_min, color: 'rgba(129,140,248,0.9)' },
              ]}
            />
          </div>

        </div>
      </div>
    </div>
  );
}
