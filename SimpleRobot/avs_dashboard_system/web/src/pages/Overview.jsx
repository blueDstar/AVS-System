import React from 'react';
import { useTelemetry, useConnected, useEmergency, useActiveCtrl, useHistory } from '../services/store';
import StatusCard from '../components/Common/StatusCard';
import StatusBadge from '../components/Common/StatusBadge';
import { 
  Activity, Cpu, ShieldCheck, 
  MapPin, Eye, RotateCw, Navigation,
  AlertTriangle
} from 'lucide-react';
import { formatNumber } from '../utils/formatters';

/** Inline sparkline SVG component */
function Sparkline({ data = [], color = '#00d4aa', height = 32 }) {
  const validData = data.filter(v => typeof v === 'number' && isFinite(v));
  if (validData.length < 2) {
    return <div className="opacity-20 text-xs text-center text-dim" style={{ height }}>—</div>;
  }
  const W = 120, H = height;
  const minV = Math.min(...validData);
  const maxV = Math.max(...validData);
  const range = maxV - minV || 1;
  const points = validData.map((val, i) => {
    const x = (i / (validData.length - 1)) * W;
    const y = H - ((val - minV) / range) * H;
    return `${x.toFixed(1)},${y.toFixed(1)}`;
  }).join(' ');
  return (
    <svg width="100%" height={H} viewBox={`0 0 ${W} ${H}`} preserveAspectRatio="none">
      <polyline points={points} fill="none" stroke={color} strokeWidth="1.5" strokeLinejoin="round" opacity="0.8" />
    </svg>
  );
}

export default function Overview() {
  const t = useTelemetry();
  const connected = useConnected();
  const eStop = useEmergency();
  const activeCtrl = useActiveCtrl();
  const history = useHistory();

  const odom  = t?.odom  || {};
  const cmd   = t?.cmd_vel || {};
  const lane  = t?.lane  || {};
  const lidar = t?.lidar || {};
  const imu   = t?.imu   || {};
  const sys   = t?.system || {};

  if (!connected) {
    return (
      <div className="page-container flex items-center justify-center" style={{ minHeight: '60vh' }}>
        <div className="text-center text-muted">
          <div style={{ fontSize: 64, marginBottom: 16 }}>🔌</div>
          <h2 className="text-xl font-bold mb-2">Waiting for Connection...</h2>
          <p className="text-sm">Ensure the ROS 2 dashboard backend is running.</p>
          <p className="text-xs text-dim mt-2">Expected at <span className="mono">ws://localhost:8080/ws</span></p>
        </div>
      </div>
    );
  }

  return (
    <div className="page-container">
      <h1 className="page-title">System Overview</h1>
      <p className="page-subtitle">Real-time status — AVS Robot, ROS_DOMAIN_ID={sys.ros_domain_id || 20}</p>

      {/* === Row 1: 4 core status cards === */}
      <div className="grid-4 mb-4">

        {/* Active Controller */}
        <div className="card">
          <div className="card-header">
            <Cpu size={16} className="text-muted" />
            <span className="card-title">Active Controller</span>
          </div>
          <div className={`metric-value ${activeCtrl !== 'off' ? 'text-accent' : 'text-dim'} mb-1`}>
            {activeCtrl.toUpperCase()}
          </div>
          <div className="text-xs text-dim mb-3">
            Publish: {formatNumber(sys.cmd_vel_hz, 1)} Hz
          </div>
          <div className="flex justify-between items-center text-xs">
            <span className="text-muted">E-Stop:</span>
            {eStop
              ? <StatusBadge status="error" label="ACTIVE" />
              : <StatusBadge status="ok" label="CLEAR" />}
          </div>
        </div>

        {/* Linear Velocity */}
        <div className="card">
          <div className="card-header">
            <Navigation size={16} className="text-muted" />
            <span className="card-title">Linear Speed</span>
          </div>
          <div className={`metric-value ${odom.timeout ? 'text-danger' : 'text-info'}`}>
            {formatNumber(odom.v, 3)}
            <span className="metric-unit">m/s</span>
          </div>
          <div className="text-xs text-dim mt-1 mb-2">
            Cmd: {formatNumber(cmd.v, 3)} m/s &nbsp;|&nbsp; Odom: {formatNumber(odom.hz, 1)} Hz
          </div>
          <Sparkline data={history.odom_v.slice(-80)} color="var(--color-info)" />
        </div>

        {/* Angular Velocity */}
        <div className="card">
          <div className="card-header">
            <RotateCw size={16} className="text-muted" />
            <span className="card-title">Angular Speed</span>
          </div>
          <div className={`metric-value ${odom.timeout ? 'text-danger' : 'text-warning'}`}>
            {formatNumber(odom.omega, 3)}
            <span className="metric-unit">rad/s</span>
          </div>
          <div className="text-xs text-dim mt-1 mb-2">
            Cmd: {formatNumber(cmd.omega, 3)} rad/s &nbsp;|&nbsp; IMU wz: {formatNumber(imu.wz, 3)}
          </div>
          <Sparkline data={history.odom_omega.slice(-80)} color="var(--color-warning)" />
        </div>

        {/* Lane Tracking */}
        <div className="card">
          <div className="card-header">
            <Eye size={16} className="text-muted" />
            <span className="card-title">Lane Tracking</span>
          </div>
          <div className={`metric-value mb-1 ${lane.valid ? 'text-success' : 'text-danger'}`}>
            {lane.valid ? 'VALID' : 'LOST'}
          </div>
          <div className="text-xs text-dim mb-3">{lane.state || '—'} | {formatNumber(lane.fps_est, 1)} fps</div>
          <div className="grid-2 text-xs gap-2">
            <div>
              <div className="text-muted">Lat Err</div>
              <div className={`font-mono font-bold ${Math.abs(lane.epsilon_x_mm) > 100 ? 'text-warning' : 'text-text'}`}>
                {formatNumber(lane.epsilon_x_mm, 1)} mm
              </div>
            </div>
            <div>
              <div className="text-muted">Head Err</div>
              <div className={`font-mono font-bold ${Math.abs(lane.theta_rad) > 0.2 ? 'text-warning' : 'text-text'}`}>
                {formatNumber(lane.theta_rad, 3)} rad
              </div>
            </div>
          </div>
        </div>

      </div>

      {/* === Row 2: 3 supporting cards === */}
      <div className="grid-3">

        {/* Obstacle Clearance */}
        <div className="card">
          <div className="card-header">
            <ShieldCheck size={16} className="text-muted" />
            <span className="card-title">Obstacle Clearance</span>
          </div>
          <div className={`metric-value mb-2 ${
            lidar.front_min < 0.35 ? 'text-danger' :
            lidar.front_min < 0.6  ? 'text-warning' : 'text-success'
          }`}>
            {formatNumber(lidar.front_min, 2)}
            <span className="metric-unit">m (front)</span>
          </div>
          <div className="progress-bar mb-1">
            <div
              className={`progress-fill ${
                lidar.front_min < 0.35 ? 'danger' :
                lidar.front_min < 0.6  ? 'warn' : ''
              }`}
              style={{ width: `${Math.min(100, (lidar.front_min / 3.0) * 100)}%` }}
            />
          </div>
          <div className="flex justify-between text-xs text-dim">
            <span>0m</span>
            <span className="text-muted">Lidar: {formatNumber(lidar.hz, 1)} Hz</span>
            <span>3m+</span>
          </div>
          <div className="grid-3 mt-3 text-xs gap-2">
            {[
              { label: 'Left', v: lidar.left_min },
              { label: 'Front', v: lidar.front_min },
              { label: 'Right', v: lidar.right_min },
            ].map(({ label, v }) => (
              <div key={label} className="text-center bg-[rgba(255,255,255,0.03)] rounded p-1">
                <div className="text-dim">{label}</div>
                <div className={`font-mono font-bold ${v < 0.5 ? 'text-danger' : 'text-text'}`}>
                  {formatNumber(v, 2)}m
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* Odometry Pose */}
        <div className="card">
          <div className="card-header">
            <MapPin size={16} className="text-muted" />
            <span className="card-title">Odometry Pose</span>
            {odom.timeout && <AlertTriangle size={14} className="text-danger ml-auto" />}
          </div>
          <div className={`metric-value text-xl mb-2 ${odom.timeout ? 'text-dim' : 'text-text'}`}>
            ({formatNumber(odom.x, 2)}, {formatNumber(odom.y, 2)})
          </div>
          <div className="text-xs text-dim mb-4">
            Yaw: {formatNumber(odom.yaw, 3)} rad
          </div>
          <div className="grid-3 text-xs gap-2">
            {[
              { label: 'X', v: formatNumber(odom.x, 3), unit: 'm' },
              { label: 'Y', v: formatNumber(odom.y, 3), unit: 'm' },
              { label: 'ψ', v: formatNumber(odom.yaw, 3), unit: 'rad' },
            ].map(({ label, v, unit }) => (
              <div key={label} className="text-center bg-[rgba(255,255,255,0.03)] rounded p-2">
                <div className="text-dim text-[10px] uppercase">{label}</div>
                <div className="font-mono font-semibold text-sm">{v}</div>
                <div className="text-dim text-[10px]">{unit}</div>
              </div>
            ))}
          </div>
        </div>

        {/* Topic Health */}
        <div className="card">
          <div className="card-header">
            <Activity size={16} className="text-muted" />
            <span className="card-title">Topic Health</span>
          </div>
          <div className="flex flex-col gap-2">
            {[
              { label: 'Odometry',    hz: odom.hz,  timeout: odom.timeout,  topic: '/odom_raw' },
              { label: 'IMU',         hz: imu.hz,   timeout: imu.timeout,   topic: '/imu' },
              { label: 'LiDAR',       hz: lidar.hz, timeout: lidar.timeout, topic: '/scan' },
              { label: 'Lane (YOLO)', hz: lane.hz,  timeout: lane.timeout,  topic: '/avs/lane_state' },
            ].map(({ label, hz, timeout, topic }) => (
              <div key={label} className="flex items-center justify-between text-xs py-1 border-b border-[rgba(255,255,255,0.04)]">
                <div>
                  <div className={`font-semibold ${timeout ? 'text-danger' : 'text-text'}`}>{label}</div>
                  <div className="text-dim font-mono">{topic}</div>
                </div>
                <div className="text-right">
                  {timeout
                    ? <StatusBadge status="error" label="TIMEOUT" dot={false} />
                    : <StatusBadge status="ok" label={`${formatNumber(hz, 1)} Hz`} dot={false} />}
                </div>
              </div>
            ))}
          </div>
        </div>

      </div>
    </div>
  );
}
