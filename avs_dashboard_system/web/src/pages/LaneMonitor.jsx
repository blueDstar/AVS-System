import React, { useEffect, useRef, useState } from 'react';
import { useTelemetry } from '../services/store';
import { Eye, AlertTriangle, Radio } from 'lucide-react';
import { formatNumber } from '../utils/formatters';

const CLASS_COLORS = {
  'main-lane': 'rgba(16, 185, 129, 0.8)', // green
  'other-lane': 'rgba(59, 130, 246, 0.8)', // blue
  'turn-lane': 'rgba(168, 85, 247, 0.8)', // purple
  'parking-zone': 'rgba(249, 115, 22, 0.8)', // orange
  'solid-white': 'rgba(255, 255, 255, 0.8)', // white
  'solid-yellow': 'rgba(234, 179, 8, 0.8)', // yellow
  'dashed-white': 'rgba(255, 255, 255, 0.8)',
  'dashed-yellow': 'rgba(234, 179, 8, 0.8)',
  'double-solid-white': 'rgba(255, 255, 255, 1)',
  'stop-line': 'rgba(239, 68, 68, 0.8)', // red
  'start': 'rgba(6, 182, 212, 0.8)', // cyan
  'vehicle': 'rgba(220, 38, 38, 0.8)' // red
};

/** Perception Canvas Overlay */
function PerceptionCanvas({ polygons, bboxes, width = 640, height = 480 }) {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const cw = canvas.width;
    const ch = canvas.height;
    ctx.clearRect(0, 0, cw, ch);
    
    // Scale from image coords to canvas coords
    const scaleX = cw / width;
    const scaleY = ch / height;
    
    // Draw grid
    ctx.strokeStyle = 'rgba(255,255,255,0.05)';
    ctx.lineWidth = 1;
    for (let i=0; i<width; i+=100) { ctx.beginPath(); ctx.moveTo(i*scaleX, 0); ctx.lineTo(i*scaleX, ch); ctx.stroke(); }
    for (let j=0; j<height; j+=100) { ctx.beginPath(); ctx.moveTo(0, j*scaleY); ctx.lineTo(cw, j*scaleY); ctx.stroke(); }

    if (polygons) {
      polygons.forEach((poly) => {
        ctx.beginPath();
        poly.points.forEach((pt, idx) => {
          if (idx === 0) ctx.moveTo(pt.x * scaleX, pt.y * scaleY);
          else ctx.lineTo(pt.x * scaleX, pt.y * scaleY);
        });
        ctx.closePath();
        
        const baseColor = poly.class_name ? CLASS_COLORS[poly.class_name] : (poly.color || 'rgba(0, 212, 170, 0.8)');
        ctx.fillStyle = baseColor ? baseColor.replace('0.8', '0.2').replace(', 1)', ', 0.2)') : 'rgba(0, 212, 170, 0.2)';
        ctx.fill();
        ctx.strokeStyle = baseColor || 'rgba(0, 212, 170, 0.8)';
        if (poly.class_name && poly.class_name.includes('dashed')) {
          ctx.setLineDash([5, 5]);
        } else {
          ctx.setLineDash([]);
        }
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.setLineDash([]);
      });
    }

    if (bboxes) {
      bboxes.forEach((box) => {
        const x = box.xmin * scaleX;
        const y = box.ymin * scaleY;
        const w = (box.xmax - box.xmin) * scaleX;
        const h = (box.ymax - box.ymin) * scaleY;
        
        const baseColor = box.class_name ? CLASS_COLORS[box.class_name] : (box.color || '#ef4444');
        ctx.strokeStyle = baseColor || '#ef4444';
        ctx.lineWidth = 2;
        ctx.strokeRect(x, y, w, h);
        
        const label = box.label || box.class_name;
        if (label) {
          ctx.fillStyle = baseColor || '#ef4444';
          ctx.fillRect(x, y - 20, ctx.measureText(label).width + 8, 20);
          ctx.fillStyle = 'white';
          ctx.font = '12px sans-serif';
          ctx.fillText(box.label, x + 4, y - 5);
        }
      });
    }
  }, [polygons, bboxes, width, height]);

  return (
    <canvas 
      ref={canvasRef} 
      width={640} height={480} 
      className="w-full h-full object-contain bg-black rounded"
    />
  );
}

/** Canvas-based lateral error gauge */
function LateralGauge({ value_mm, max_mm = 400 }) {
  const ref = useRef(null);

  useEffect(() => {
    const canvas = ref.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    ctx.clearRect(0, 0, W, H);

    const norm = Math.max(-1, Math.min(1, value_mm / max_mm));
    const cx   = W / 2;
    const barH = 20;
    const barY = H / 2 - barH / 2;
    const barW = W - 32;

    // Track background
    ctx.fillStyle = 'rgba(255,255,255,0.05)';
    ctx.beginPath();
    ctx.roundRect(16, barY, barW, barH, 10);
    ctx.fill();

    // Warning zones
    [-0.5, 0.25].forEach((start) => {
      ctx.fillStyle = 'rgba(245,158,11,0.12)';
      ctx.fillRect(cx + start * barW, barY, 0.25 * barW, barH);
    });
    // Danger zones
    ctx.fillStyle = 'rgba(239,68,68,0.12)';
    ctx.fillRect(16, barY, 0.15 * barW, barH);
    ctx.fillRect(16 + 0.85 * barW, barY, 0.15 * barW, barH);

    // Indicator
    const indicatorX = cx + norm * (barW / 2);
    const indColor = Math.abs(norm) > 0.8 ? '#ef4444' : Math.abs(norm) > 0.5 ? '#f59e0b' : '#00d4aa';
    ctx.fillStyle = indColor;
    ctx.beginPath();
    ctx.arc(indicatorX, H / 2, 12, 0, Math.PI * 2);
    ctx.fill();
    // Glow
    const grd = ctx.createRadialGradient(indicatorX, H / 2, 0, indicatorX, H / 2, 20);
    grd.addColorStop(0, indColor + 'aa');
    grd.addColorStop(1, 'transparent');
    ctx.fillStyle = grd;
    ctx.beginPath();
    ctx.arc(indicatorX, H / 2, 20, 0, Math.PI * 2);
    ctx.fill();

    // Center line
    ctx.strokeStyle = 'rgba(255,255,255,0.3)';
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(cx, barY - 4);
    ctx.lineTo(cx, barY + barH + 4);
    ctx.stroke();
    ctx.setLineDash([]);
  }, [value_mm, max_mm]);

  return (
    <canvas ref={ref} width={320} height={60} style={{ width: '100%', height: 60 }} />
  );
}

/** Heading error visual arc */
function HeadingArc({ theta_rad }) {
  const MAX = 1.2;
  const norm = Math.max(-MAX, Math.min(MAX, theta_rad)) / MAX;
  const angleDeg = norm * 60; // ±60° display range

  return (
    <div className="flex flex-col items-center">
      <svg width="120" height="70" viewBox="0 0 120 70">
        {/* Arc background */}
        <path d="M 10 65 A 55 55 0 0 1 110 65" fill="none" stroke="rgba(255,255,255,0.08)" strokeWidth="8" strokeLinecap="round" />
        {/* Zones */}
        <path d="M 10 65 A 55 55 0 0 1 60 10" fill="none" stroke="rgba(16,185,129,0.3)" strokeWidth="8" strokeLinecap="round" />
        <path d="M 60 10 A 55 55 0 0 1 110 65" fill="none" stroke="rgba(16,185,129,0.3)" strokeWidth="8" strokeLinecap="round" />
        {/* Indicator needle */}
        <line
          x1="60" y1="65"
          x2={60 + 50 * Math.sin(theta_rad)}
          y2={65 - 50 * Math.cos(Math.abs(theta_rad) < 1.5 ? theta_rad : Math.sign(theta_rad) * 1.5)}
          stroke={Math.abs(theta_rad) > 0.3 ? '#f59e0b' : '#00d4aa'}
          strokeWidth="3"
          strokeLinecap="round"
        />
        <circle cx="60" cy="65" r="5" fill={Math.abs(theta_rad) > 0.3 ? '#f59e0b' : '#00d4aa'} />
      </svg>
      <div className={`font-mono font-bold text-lg ${Math.abs(theta_rad) > 0.3 ? 'text-warning' : 'text-success'}`}>
        {formatNumber(theta_rad, 3)} rad
      </div>
      <div className="text-xs text-dim">({formatNumber(theta_rad * 180 / Math.PI, 1)}°)</div>
    </div>
  );
}

export default function LaneMonitor() {
  const telemetry = useTelemetry();
  const lane  = telemetry?.lane  || {};
  const lidar = telemetry?.lidar || {};

  const isValid   = lane.valid;
  const isTimeout = lane.timeout;

  // Lateral error history for mini chart
  const errHistory = useRef([]);
  useEffect(() => {
    errHistory.current.push(lane.epsilon_x_mm || 0);
    if (errHistory.current.length > 100) errHistory.current.shift();
  }, [lane.epsilon_x_mm]);

  return (
    <div className="page-container flex flex-col h-full" style={{ minHeight: 0 }}>
      <h1 className="page-title">Lane Monitor</h1>
      <p className="page-subtitle mb-4">Perception system status and tracking error visualization.</p>

      <div className="flex gap-4 flex-1 min-h-0">

        {/* Left column: Status */}
        <div className="flex flex-col gap-4" style={{ width: 260 }}>

          <div className="card">
            <div className="card-header">
              <Radio size={16} className="text-muted" />
              <span className="card-title">Perception Status</span>
            </div>
            <div className={`text-2xl font-bold mb-2 ${isTimeout ? 'text-dim' : isValid ? 'text-success' : 'text-danger'}`}>
              {isTimeout ? '⏸ TIMEOUT' : isValid ? '✓ TRACKING' : '✕ LANE LOST'}
            </div>
            <div className="text-xs text-muted mb-4 font-mono">{lane.state || 'UNKNOWN'}</div>

            <div className="flex flex-col gap-3 text-sm">
              <div className="flex justify-between">
                <span className="text-muted">Vision FPS</span>
                <span className="font-mono font-bold text-info">{formatNumber(lane.fps_est, 1)}</span>
              </div>
              {lane.classes && lane.classes.length > 0 && (
                <div className="flex justify-between items-start mt-1 border-t border-[rgba(255,255,255,0.05)] pt-2">
                  <span className="text-muted">Detected</span>
                  <div className="flex flex-col gap-1 items-end">
                    {lane.classes.map((cls, idx) => (
                      <span key={idx} className="badge badge-accent text-[10px] py-0">{cls}</span>
                    ))}
                  </div>
                </div>
              )}
              <div className="flex justify-between">
                <span className="text-muted">Topic Hz</span>
                <span className="font-mono font-bold">{formatNumber(lane.hz, 1)}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted">ε_y (mm)</span>
                <span className="font-mono font-bold">{formatNumber(lane.epsilon_y_mm, 1)}</span>
              </div>
            </div>
            
            {/* Status Badges */}
            <div className="flex flex-col gap-2 mt-4 pt-4 border-t border-[rgba(255,255,255,0.05)]">
              {lane.special_zones?.stop_line_detected && (
                <div className="bg-red-900/50 text-red-200 border border-red-500 rounded px-2 py-1 text-center font-bold text-xs animate-pulse">STOP LINE DETECTED</div>
              )}
              {lane.special_zones?.parking_zone_detected && (
                <div className="bg-orange-900/50 text-orange-200 border border-orange-500 rounded px-2 py-1 text-center font-bold text-xs animate-pulse">PARKING ZONE DETECTED</div>
              )}
              {lane.special_zones?.turn_lane_detected && (
                <div className="bg-purple-900/50 text-purple-200 border border-purple-500 rounded px-2 py-1 text-center font-bold text-xs">TURN LANE DETECTED</div>
              )}
              {lane.special_zones?.vehicle_detected && (
                <div className="bg-red-900/50 text-red-200 border border-red-500 rounded px-2 py-1 text-center font-bold text-xs animate-pulse">VEHICLE DETECTED</div>
              )}
              {isValid && lane.classes_detected?.some(c => c.class_name === 'main-lane') && (
                <div className="bg-emerald-900/50 text-emerald-200 border border-emerald-500 rounded px-2 py-1 text-center font-bold text-xs">MAIN LANE DETECTED</div>
              )}
              {!isValid && !isTimeout && (
                <div className="bg-red-900/50 text-red-200 border border-red-500 rounded px-2 py-1 text-center font-bold text-xs animate-pulse">LANE LOST</div>
              )}
            </div>
          </div>

          {/* Camera/Perception Overlay */}
          <div className="card flex-1 flex flex-col">
            <div className="card-header">
              <span className="card-title">Perception View (Polygons & BBoxes)</span>
            </div>
            <div className="flex-1 relative bg-black/40 rounded overflow-hidden flex items-center justify-center p-2">
              <PerceptionCanvas 
                polygons={lane.polygons || []} 
                bboxes={lane.bboxes || []} 
              />
              {!isValid && (
                <div className="absolute inset-0 z-10 flex flex-col items-center justify-center bg-black/50 backdrop-blur-sm">
                  <AlertTriangle size={40} className="text-danger mb-2 animate-pulse" />
                  <h2 className="font-bold text-danger">{isTimeout ? 'TIMEOUT' : 'NO DETECTIONS'}</h2>
                </div>
              )}
            </div>
          </div>

          {/* Class Detection Panel */}
          <div className="card">
            <div className="card-header">
              <span className="card-title">Class Detections</span>
            </div>
            <div className="overflow-x-auto">
              <table className="w-full text-left text-sm whitespace-nowrap">
                <thead className="bg-[rgba(255,255,255,0.05)]">
                  <tr>
                    <th className="p-2 text-muted font-semibold">Class Name</th>
                    <th className="p-2 text-muted font-semibold text-right">Count</th>
                    <th className="p-2 text-muted font-semibold text-right">Max Conf</th>
                    <th className="p-2 text-muted font-semibold text-right">Mean Conf</th>
                    <th className="p-2 text-muted font-semibold text-right">Last Seen</th>
                  </tr>
                </thead>
                <tbody>
                  {(!lane.classes_detected || lane.classes_detected.length === 0) && (
                    <tr><td colSpan={5} className="p-4 text-center text-muted">No classes detected</td></tr>
                  )}
                  {(lane.classes_detected || []).map((c, idx) => (
                    <tr key={idx} className="border-t border-[rgba(255,255,255,0.05)]">
                      <td className="p-2 font-bold" style={{ color: CLASS_COLORS[c.class_name] || 'white' }}>{c.class_name}</td>
                      <td className="p-2 text-right font-mono">{c.count}</td>
                      <td className={`p-2 text-right font-mono ${c.max_confidence > 0.8 ? 'text-success' : 'text-warning'}`}>{formatNumber(c.max_confidence, 2)}</td>
                      <td className="p-2 text-right font-mono">{formatNumber(c.mean_confidence, 2)}</td>
                      <td className="p-2 text-right font-mono text-dim">{c.last_seen_age_s < 0.1 ? 'now' : `${formatNumber(c.last_seen_age_s, 1)}s ago`}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </div>

          {/* LiDAR Quick View */}
          <div className="card flex-1">
            <div className="card-header">
              <span className="card-title">LiDAR Safety</span>
            </div>
            <div className="flex flex-col gap-2">
              {[
                { label: 'Front', v: lidar.front_min, danger: 0.4, warn: 0.7 },
                { label: 'Left',  v: lidar.left_min,  danger: 0.2, warn: 0.4 },
                { label: 'Right', v: lidar.right_min, danger: 0.2, warn: 0.4 },
              ].map(({ label, v, danger, warn }) => (
                <div key={label}>
                  <div className="flex justify-between text-xs mb-1">
                    <span className="text-muted">{label}</span>
                    <span className={`font-mono font-bold ${v < danger ? 'text-danger' : v < warn ? 'text-warning' : 'text-success'}`}>
                      {formatNumber(v, 2)} m
                    </span>
                  </div>
                  <div className="progress-bar">
                    <div
                      className={`progress-fill ${v < danger ? 'danger' : v < warn ? 'warn' : ''}`}
                      style={{ width: `${Math.min(100, (v / 3.0) * 100)}%`, transition: 'width 0.1s linear' }}
                    />
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>

        {/* Right: Visualization */}
        <div className="flex flex-col gap-4 flex-1 min-w-0">

          {/* Lateral Error Gauge */}
          <div className="card">
            <div className="card-header">
              <span className="card-title">Lateral Error — ε_x</span>
              <span className={`ml-auto text-lg font-mono font-bold ${Math.abs(lane.epsilon_x_mm) > 150 ? 'text-danger' : Math.abs(lane.epsilon_x_mm) > 80 ? 'text-warning' : 'text-success'}`}>
                {lane.epsilon_x_mm > 0 ? '+' : ''}{formatNumber(lane.epsilon_x_mm, 1)} mm
              </span>
            </div>
            <LateralGauge value_mm={lane.epsilon_x_mm || 0} max_mm={400} />
            <div className="flex justify-between text-xs text-dim mt-2">
              <span>← Left 400mm</span>
              <span className="text-muted">Center</span>
              <span>Right 400mm →</span>
            </div>
          </div>

          {/* Heading Error */}
          <div className="card flex items-center justify-around">
            <div className="text-center">
              <div className="card-title mb-4">Heading Error — θ</div>
              <HeadingArc theta_rad={lane.theta_rad || 0} />
            </div>
            <div className="text-center">
              <div className="card-title mb-4">Lateral Error History</div>
              <svg width="200" height="80" viewBox="0 0 200 80">
                <line x1="0" y1="40" x2="200" y2="40" stroke="rgba(255,255,255,0.1)" strokeDasharray="4 4" />
                {errHistory.current.length > 1 && (
                  <polyline
                    points={errHistory.current.map((v, i) => {
                      const x = (i / (errHistory.current.length - 1)) * 200;
                      const y = 40 - (v / 400) * 36;
                      return `${x},${Math.max(2, Math.min(78, y))}`;
                    }).join(' ')}
                    fill="none" stroke="#00d4aa" strokeWidth="1.5" strokeLinejoin="round"
                  />
                )}
                <text x="4" y="12" fill="#64748b" fontSize="9">+400mm</text>
                <text x="4" y="76" fill="#64748b" fontSize="9">-400mm</text>
              </svg>
            </div>
          </div>

          {/* Robot in lane visual */}
          <div className="card relative overflow-hidden flex items-center justify-center"
               style={{ minHeight: 180, background: 'rgba(0,0,0,0.3)' }}>
            {!isValid && (
              <div className="absolute inset-0 z-10 flex flex-col items-center justify-center bg-black/50 backdrop-blur-sm">
                <AlertTriangle size={40} className="text-danger mb-2 animate-pulse" />
                <h2 className="font-bold text-danger">{isTimeout ? 'PERCEPTION TIMEOUT' : 'LANE LOST'}</h2>
              </div>
            )}
            {/* Lane visualization */}
            <svg width="300" height="160" viewBox="0 0 300 160">
              {/* Lane boundaries based on detection */}
              {lane.classes_detected?.some(c => c.class_name === 'other-lane') ? (
                <>
                  <line x1="20" y1="0" x2="20" y2="160" stroke="rgba(59,130,246,0.5)" strokeWidth="3" />
                  <line x1="280" y1="0" x2="280" y2="160" stroke="rgba(59,130,246,0.5)" strokeWidth="3" />
                  <rect x="20" y="0" width="40" height="160" fill="rgba(59,130,246,0.05)" />
                  <rect x="240" y="0" width="40" height="160" fill="rgba(59,130,246,0.05)" />
                </>
              ) : null}
              
              <line x1="60" y1="0" x2="60" y2="160" stroke="rgba(255,255,255,0.6)" strokeWidth="3" />
              <line x1="240" y1="0" x2="240" y2="160" stroke="rgba(255,255,255,0.6)" strokeWidth="3" />
              <rect x="60" y="0" width="180" height="160" fill="rgba(16,185,129,0.05)" />
              
              {/* Special Lines */}
              {lane.classes_detected?.some(c => c.class_name === 'solid-yellow') && (
                <line x1="60" y1="0" x2="60" y2="160" stroke="rgba(234,179,8,0.8)" strokeWidth="4" />
              )}
              {lane.classes_detected?.some(c => c.class_name === 'dashed-white') ? (
                <line x1="150" y1="0" x2="150" y2="160" stroke="rgba(255,255,255,0.6)" strokeWidth="2" strokeDasharray="10 10" />
              ) : (
                <line x1="150" y1="0" x2="150" y2="160" stroke="rgba(0,212,170,0.2)" strokeWidth="1.5" strokeDasharray="10 8" />
              )}
              
              {lane.special_zones?.stop_line_detected && (
                <line x1="60" y1="40" x2="240" y2="40" stroke="rgba(239,68,68,0.8)" strokeWidth="8" />
              )}
              {lane.special_zones?.parking_zone_detected && (
                <rect x="250" y="20" width="40" height="80" fill="rgba(249,115,22,0.3)" stroke="rgba(249,115,22,0.8)" strokeWidth="2" />
              )}

              {/* Lookahead target point */}
              {isValid && (
                <>
                  <circle cx="150" cy="20" r="4" fill="#f59e0b" />
                  <line x1="150" y1="80" x2="150" y2="20" stroke="rgba(245,158,11,0.3)" strokeWidth="1" strokeDasharray="4 4" />
                </>
              )}

              {/* Robot */}
              {(() => {
                const MAX_ERR = 80;
                const normLat = Math.max(-1, Math.min(1, (lane.epsilon_x_mm || 0) / 400));
                const normTheta = Math.max(-1, Math.min(1, (lane.theta_rad || 0) / 1.2));
                const robotX = 150 + normLat * MAX_ERR;
                const robotY = 80;
                const rotDeg = normTheta * 20;
                return (
                  <g>
                    {/* Error line */}
                    {Math.abs(normLat) > 0.05 && (
                      <line x1="150" y1="80" x2={robotX} y2="80" stroke="#ef4444" strokeWidth="2" />
                    )}
                    <g transform={`translate(${robotX},${robotY}) rotate(${rotDeg})`}>
                      <rect x="-15" y="-25" width="30" height="50" rx="4" fill="#00d4aa" opacity="0.9" />
                      <polygon points="0,-32 -8,-22 8,-22" fill="white" opacity="0.9" />
                      <rect x="-10" y="-20" width="20" height="12" rx="2" fill="rgba(0,0,0,0.3)" />
                      {/* Heading error arrow */}
                      {Math.abs(normTheta) > 0.1 && (
                        <line x1="0" y1="-30" x2="0" y2="-60" stroke="#f59e0b" strokeWidth="2" markerEnd="url(#arrow)" />
                      )}
                    </g>
                  </g>
                );
              })()}
              <defs>
                <marker id="arrow" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
                  <path d="M 0 0 L 10 5 L 0 10 z" fill="#f59e0b" />
                </marker>
              </defs>
            </svg>
          </div>

        </div>
      </div>
    </div>
  );
}
