import React, { useRef, useEffect, useState } from 'react';

/**
 * VirtualJoystick - SVG based joystick for sending manual velocity commands.
 * 
 * @param {function} onMove - Callback (v, omega) normalized to [-1, 1]
 * @param {function} onRelease - Callback () when joystick is released
 */
export default function VirtualJoystick({ onMove, onRelease }) {
  const containerRef = useRef(null);
  const [active, setActive] = useState(false);
  const [pos, setPos] = useState({ x: 0, y: 0 }); // -1 to 1

  const size = 200;
  const radius = size / 2;
  const knobRadius = 30;
  const maxDist = radius - knobRadius;

  const handlePointerDown = (e) => {
    setActive(true);
    updatePos(e.clientX, e.clientY);
  };

  const handlePointerMove = (e) => {
    if (!active) return;
    updatePos(e.clientX, e.clientY);
  };

  const handlePointerUp = () => {
    setActive(false);
    setPos({ x: 0, y: 0 });
    if (onRelease) onRelease();
  };

  const updatePos = (clientX, clientY) => {
    if (!containerRef.current) return;
    const rect = containerRef.current.getBoundingClientRect();
    const centerX = rect.left + radius;
    const centerY = rect.top + radius;

    let dx = clientX - centerX;
    let dy = clientY - centerY;

    const dist = Math.sqrt(dx * dx + dy * dy);
    if (dist > maxDist) {
      dx = (dx / dist) * maxDist;
      dy = (dy / dist) * maxDist;
    }

    // Normalize to [-1, 1]
    // x axis (left/right) -> corresponds to omega (inverted, left is positive omega)
    // y axis (up/down) -> corresponds to v (inverted, up is positive v)
    const normX = dx / maxDist;
    const normY = dy / maxDist;

    setPos({ x: dx, y: dy });

    if (onMove) {
      // Note: Up = -normY. We want Up = positive v
      // Note: Right = +normX. We want Right = negative omega (turn right)
      onMove(-normY, -normX);
    }
  };

  useEffect(() => {
    const handleUp = () => {
      if (active) handlePointerUp();
    };
    window.addEventListener('pointerup', handleUp);
    return () => window.removeEventListener('pointerup', handleUp);
  }, [active]);

  return (
    <div 
      ref={containerRef}
      className="relative touch-none select-none"
      style={{ width: size, height: size, margin: '0 auto' }}
      onPointerDown={handlePointerDown}
      onPointerMove={handlePointerMove}
    >
      <svg width={size} height={size}>
        {/* Base circle */}
        <circle 
          cx={radius} cy={radius} r={radius - 2} 
          fill="rgba(255,255,255,0.05)" 
          stroke="var(--color-border)" strokeWidth="2" 
        />
        
        {/* Crosshairs */}
        <line x1={radius} y1="10" x2={radius} y2={size-10} stroke="rgba(255,255,255,0.1)" strokeWidth="1" strokeDasharray="4 4" />
        <line x1="10" y1={radius} x2={size-10} y2={radius} stroke="rgba(255,255,255,0.1)" strokeWidth="1" strokeDasharray="4 4" />
        
        {/* Connection line */}
        {active && (
          <line 
            x1={radius} y1={radius} 
            x2={radius + pos.x} y2={radius + pos.y} 
            stroke="var(--color-accent)" strokeWidth="4" opacity="0.5"
          />
        )}
        
        {/* Knob */}
        <circle 
          cx={radius + pos.x} cy={radius + pos.y} r={knobRadius}
          fill="var(--color-accent)"
          style={{ transition: active ? 'none' : 'all 0.2s ease-out' }}
          filter="drop-shadow(0 0 8px rgba(0,212,170,0.5))"
        />
      </svg>
    </div>
  );
}
