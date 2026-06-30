import React, { useEffect, useRef, useCallback } from 'react';
import { useOdomPath, useTelemetry, useDispatch } from '../../services/store';

/**
 * OdomCanvas
 * Real-time 2D Canvas that draws:
 *  - Grid (1m cells)
 *  - Odometry path trace
 *  - Robot pose indicator (triangle)
 *  - Coordinate axes at origin
 */
export default function OdomCanvas({ className = '', style = {} }) {
  const canvasRef  = useRef(null);
  const rafRef     = useRef(null);
  const odomPath   = useOdomPath();
  const telemetry  = useTelemetry();
  const dispatch   = useDispatch();

  // Transform: ROS coords → canvas pixel coords
  // We auto-fit the path inside the canvas with 10% margin.
  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const W   = canvas.width;
    const H   = canvas.height;

    ctx.clearRect(0, 0, W, H);

    // ---- Background ----
    ctx.fillStyle = '#0a0e17';
    ctx.fillRect(0, 0, W, H);

    // ---- Compute view transform ----
    let minX = -3, maxX = 3, minY = -3, maxY = 3;
    if (odomPath.length > 1) {
      minX = Math.min(...odomPath.map(p => p.x)) - 1.5;
      maxX = Math.max(...odomPath.map(p => p.x)) + 1.5;
      minY = Math.min(...odomPath.map(p => p.y)) - 1.5;
      maxY = Math.max(...odomPath.map(p => p.y)) + 1.5;
      // Maintain square aspect
      const rangeX = maxX - minX;
      const rangeY = maxY - minY;
      const range  = Math.max(rangeX, rangeY);
      const cx     = (minX + maxX) / 2;
      const cy     = (minY + maxY) / 2;
      minX = cx - range / 2; maxX = cx + range / 2;
      minY = cy - range / 2; maxY = cy + range / 2;
    }
    const rangeX = maxX - minX;
    const rangeY = maxY - minY;
    const scaleX = W / rangeX;
    const scaleY = H / rangeY;
    const scale  = Math.min(scaleX, scaleY);

    const toCanvas = (rx, ry) => ({
      x: (rx - minX) * scaleX,
      y: H - (ry - minY) * scaleY,   // flip Y (ROS Y points left/up)
    });

    // ---- Grid (1m cells) ----
    ctx.strokeStyle = 'rgba(255,255,255,0.05)';
    ctx.lineWidth   = 1;
    const gridStart = { x: Math.ceil(minX), y: Math.ceil(minY) };
    for (let gx = gridStart.x; gx <= maxX; gx++) {
      const p = toCanvas(gx, minY);
      const q = toCanvas(gx, maxY);
      ctx.beginPath();
      ctx.moveTo(p.x, p.y);
      ctx.lineTo(q.x, q.y);
      ctx.stroke();
    }
    for (let gy = gridStart.y; gy <= maxY; gy++) {
      const p = toCanvas(minX, gy);
      const q = toCanvas(maxX, gy);
      ctx.beginPath();
      ctx.moveTo(p.x, p.y);
      ctx.lineTo(q.x, q.y);
      ctx.stroke();
    }

    // ---- Origin axes ----
    if (minX < 0 && maxX > 0 && minY < 0 && maxY > 0) {
      const O = toCanvas(0, 0);
      // X axis
      ctx.strokeStyle = 'rgba(239,68,68,0.5)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(O.x, O.y);
      ctx.lineTo(toCanvas(1, 0).x, toCanvas(1, 0).y);
      ctx.stroke();
      // Y axis
      ctx.strokeStyle = 'rgba(16,185,129,0.5)';
      ctx.beginPath();
      ctx.moveTo(O.x, O.y);
      ctx.lineTo(toCanvas(0, 1).x, toCanvas(0, 1).y);
      ctx.stroke();
      // Origin dot
      ctx.fillStyle = 'rgba(255,255,255,0.5)';
      ctx.beginPath();
      ctx.arc(O.x, O.y, 4, 0, Math.PI * 2);
      ctx.fill();
    }

    // ---- Path trace ----
    if (odomPath.length > 1) {
      // Fade from old (dim) to recent (bright)
      const grad = ctx.createLinearGradient(0, 0, W, 0);
      grad.addColorStop(0, 'rgba(0,212,170,0.2)');
      grad.addColorStop(1, 'rgba(0,212,170,1.0)');

      ctx.strokeStyle = '#00d4aa';
      ctx.lineWidth   = 2;
      ctx.lineJoin    = 'round';
      ctx.beginPath();
      for (let i = 0; i < odomPath.length; i++) {
        const { x, y } = toCanvas(odomPath[i].x, odomPath[i].y);
        if (i === 0) ctx.moveTo(x, y);
        else         ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    // ---- Robot pose ----
    const odom = telemetry?.odom || {};
    const yaw  = odom.yaw || 0;
    const rp   = toCanvas(odom.x || 0, odom.y || 0);

    const robotW = 12;
    const robotH = 20;
    ctx.save();
    ctx.translate(rp.x, rp.y);
    ctx.rotate(-yaw + Math.PI / 2); // canvas angle: forward = up
    // Robot body (rectangle)
    ctx.fillStyle   = 'rgba(0,212,170,0.9)';
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth   = 1.5;
    ctx.beginPath();
    ctx.roundRect(-robotW / 2, -robotH / 2, robotW, robotH, 3);
    ctx.fill();
    ctx.stroke();
    // Direction arrow
    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    ctx.moveTo(0, -robotH / 2 - 6);
    ctx.lineTo(-5, -robotH / 2 + 2);
    ctx.lineTo(5,  -robotH / 2 + 2);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    // ---- Scale indicator ----
    const scaleBarM  = 1.0;           // 1 meter
    const scaleBarPx = scale * scaleBarM;
    const barY = H - 18;
    ctx.strokeStyle = 'rgba(255,255,255,0.6)';
    ctx.lineWidth   = 2;
    ctx.beginPath();
    ctx.moveTo(20, barY);
    ctx.lineTo(20 + scaleBarPx, barY);
    ctx.moveTo(20, barY - 4);
    ctx.lineTo(20, barY + 4);
    ctx.moveTo(20 + scaleBarPx, barY - 4);
    ctx.lineTo(20 + scaleBarPx, barY + 4);
    ctx.stroke();
    ctx.fillStyle   = 'rgba(255,255,255,0.7)';
    ctx.font        = '11px JetBrains Mono, monospace';
    ctx.fillText('1 m', 24 + scaleBarPx, barY + 4);

  }, [odomPath, telemetry]);

  useEffect(() => {
    const animate = () => {
      draw();
      rafRef.current = requestAnimationFrame(animate);
    };
    rafRef.current = requestAnimationFrame(animate);
    return () => cancelAnimationFrame(rafRef.current);
  }, [draw]);

  // Resize observer
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ro = new ResizeObserver(entries => {
      for (const e of entries) {
        canvas.width  = e.contentRect.width;
        canvas.height = e.contentRect.height;
      }
    });
    ro.observe(canvas);
    return () => ro.disconnect();
  }, []);

  return (
    <canvas
      ref={canvasRef}
      className={className}
      style={{ display: 'block', width: '100%', height: '100%', cursor: 'crosshair', ...style }}
    />
  );
}
