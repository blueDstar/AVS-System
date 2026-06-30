import React from 'react';
import OdomCanvas from '../components/Charts/OdomCanvas';
import { useOdomPath, useDispatch, useTelemetry } from '../services/store';
import { MapPin, Trash2, Download } from 'lucide-react';
import { formatNumber } from '../utils/formatters';

export default function MapViewer() {
  const odomPath = useOdomPath();
  const dispatch = useDispatch();
  const telemetry = useTelemetry();
  const odom = telemetry?.odom || {};

  const clearPath = () => dispatch({ type: 'RESET_ODOM_PATH' });

  return (
    <div className="page-container flex flex-col h-full" style={{ minHeight: 0 }}>
      <div className="flex justify-between items-end mb-4 shrink-0">
        <div>
          <h1 className="page-title">Map Viewer</h1>
          <p className="page-subtitle mb-0">HTML5 Canvas 2D — Robot odometry path visualization</p>
        </div>
        <div className="flex gap-2">
          <button onClick={clearPath} className="btn btn-secondary flex items-center gap-2">
            <Trash2 size={14} /> Clear Path
          </button>
        </div>
      </div>

      <div className="flex gap-4 flex-1 min-h-0">
        {/* Map Canvas */}
        <div className="card flex-1 p-0 overflow-hidden" style={{ minHeight: 400 }}>
          <OdomCanvas style={{ width: '100%', height: '100%' }} />
        </div>

        {/* Right panel: Pose info */}
        <div className="card shrink-0" style={{ width: 200 }}>
          <div className="card-header">
            <MapPin size={16} className="text-muted" />
            <span className="card-title">Current Pose</span>
          </div>

          <div className="flex flex-col gap-4">
            {[
              { label: 'X',   value: formatNumber(odom.x, 4),   unit: 'm' },
              { label: 'Y',   value: formatNumber(odom.y, 4),   unit: 'm' },
              { label: 'Yaw', value: formatNumber(odom.yaw, 4), unit: 'rad' },
              { label: 'V',   value: formatNumber(odom.v, 3),   unit: 'm/s' },
              { label: 'ω',   value: formatNumber(odom.omega, 3), unit: 'rad/s' },
            ].map(({ label, value, unit }) => (
              <div key={label}>
                <div className="text-xs text-dim uppercase mb-1">{label}</div>
                <div className="font-mono font-bold text-base text-text">
                  {value} <span className="text-xs text-muted font-normal">{unit}</span>
                </div>
              </div>
            ))}

            <div className="mt-4 pt-3 border-t border-[rgba(255,255,255,0.06)]">
              <div className="text-xs text-dim uppercase mb-1">Path Points</div>
              <div className="font-mono font-bold text-accent">{odomPath.length}</div>
            </div>

            {odom.timeout && (
              <div className="text-xs text-danger font-semibold">
                ⚠ Odom Timeout
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
