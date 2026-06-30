import React, { useState, useEffect, useRef } from 'react';
import { wsService } from '../services/websocket';
import { useActiveCtrl, useTelemetry } from '../services/store';
import VirtualJoystick from '../components/Control/VirtualJoystick';
import { Gamepad2, AlertTriangle } from 'lucide-react';

export default function ManualControl() {
  const activeCtrl = useActiveCtrl();
  const telemetry = useTelemetry();
  
  const [maxV, setMaxV] = useState(0.2);
  const [maxOmega, setMaxOmega] = useState(1.0);
  const [keyboardActive, setKeyboardActive] = useState(false);
  
  // WS send loop for continuous joystick sending
  const cmdRef = useRef({ v: 0, omega: 0 });
  const intervalRef = useRef(null);

  // Send manual command
  const sendCmd = (v, omega) => {
    wsService.send('manual_cmd', { v, omega });
  };

  // Joystick handlers
  const handleJoyMove = (vNorm, omegaNorm) => {
    cmdRef.current = { v: vNorm * maxV, omega: omegaNorm * maxOmega };
  };

  const handleJoyRelease = () => {
    cmdRef.current = { v: 0, omega: 0 };
    sendCmd(0, 0); // Send stop immediately
  };

  // Keyboard handlers (WASD)
  useEffect(() => {
    const keys = { w: false, a: false, s: false, d: false };
    
    const updateFromKeys = () => {
      if (!keyboardActive) return;
      let v = 0; let omega = 0;
      if (keys.w) v += maxV;
      if (keys.s) v -= maxV;
      if (keys.a) omega += maxOmega;
      if (keys.d) omega -= maxOmega;
      cmdRef.current = { v, omega };
    };

    const onKeyDown = (e) => {
      const key = e.key.toLowerCase();
      if (keys.hasOwnProperty(key)) {
        keys[key] = true;
        updateFromKeys();
      }
    };
    const onKeyUp = (e) => {
      const key = e.key.toLowerCase();
      if (keys.hasOwnProperty(key)) {
        keys[key] = false;
        updateFromKeys();
      }
    };

    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
    };
  }, [keyboardActive, maxV, maxOmega]);

  // Send loop (10Hz)
  useEffect(() => {
    intervalRef.current = setInterval(() => {
      if (cmdRef.current.v !== 0 || cmdRef.current.omega !== 0) {
        sendCmd(cmdRef.current.v, cmdRef.current.omega);
      }
    }, 100);
    return () => clearInterval(intervalRef.current);
  }, []);

  const requestManualControl = () => {
    wsService.send('switch_controller', { name: 'manual' });
  };

  const isManualActive = activeCtrl === 'manual';

  return (
    <div className="page-container flex flex-col items-center">
      <h1 className="page-title w-full">Manual Control</h1>
      <p className="page-subtitle w-full">Directly teleoperate the robot using joystick or WASD.</p>

      {!isManualActive && (
        <div className="w-full max-w-xl mb-6 p-4 bg-warning/10 border border-warning/30 rounded flex items-start gap-4">
          <AlertTriangle className="text-warning shrink-0" />
          <div>
            <h3 className="font-bold text-warning mb-1">Manual Control is not active</h3>
            <p className="text-sm text-muted mb-3">
              Current controller is: <strong>{activeCtrl}</strong>. Commands sent here will be ignored by the cmd_vel multiplexer until you switch to manual mode.
            </p>
            <button onClick={requestManualControl} className="btn btn-primary">
              Switch to Manual Control
            </button>
          </div>
        </div>
      )}

      <div className="grid-2 w-full max-w-4xl gap-6">
        
        {/* Left: Joystick */}
        <div className="card flex flex-col items-center justify-center p-8">
          <VirtualJoystick onMove={handleJoyMove} onRelease={handleJoyRelease} />
          
          <div className="mt-8 text-center text-sm text-muted">
            Drag the knob to drive.<br/>
            Release to stop.
          </div>
        </div>

        {/* Right: Settings & Status */}
        <div className="card flex flex-col gap-6">
          <div>
            <h3 className="font-semibold border-b border-[rgba(255,255,255,0.1)] pb-2 mb-4 flex items-center gap-2">
              <Gamepad2 size={18}/> Settings
            </h3>
            
            <div className="mb-4">
              <div className="flex justify-between mb-1">
                <label className="text-xs text-muted uppercase">Max Speed (v)</label>
                <span className="text-accent font-mono">{maxV.toFixed(2)} m/s</span>
              </div>
              <input 
                type="range" className="slider" 
                min="0.05" max="0.5" step="0.01" 
                value={maxV} onChange={e => setMaxV(parseFloat(e.target.value))} 
              />
            </div>
            
            <div className="mb-6">
              <div className="flex justify-between mb-1">
                <label className="text-xs text-muted uppercase">Max Turn Rate (ω)</label>
                <span className="text-accent font-mono">{maxOmega.toFixed(2)} rad/s</span>
              </div>
              <input 
                type="range" className="slider" 
                min="0.1" max="2.0" step="0.1" 
                value={maxOmega} onChange={e => setMaxOmega(parseFloat(e.target.value))} 
              />
            </div>

            <label className="flex items-center gap-3 cursor-pointer p-3 bg-[rgba(255,255,255,0.02)] border border-[rgba(255,255,255,0.05)] rounded">
              <div className="toggle">
                <input type="checkbox" checked={keyboardActive} onChange={e => setKeyboardActive(e.target.checked)} />
                <div className="toggle-track"><div className="toggle-thumb" /></div>
              </div>
              <div>
                <div className="text-sm font-semibold">Enable Keyboard (WASD)</div>
                <div className="text-xs text-muted">Use W, A, S, D keys to drive</div>
              </div>
            </label>
          </div>

          <div className="mt-auto">
             <h3 className="text-xs text-muted uppercase mb-2">Live Telemetry</h3>
             <div className="grid grid-cols-2 gap-4 bg-[rgba(0,0,0,0.2)] p-4 rounded">
                <div>
                  <div className="text-xs text-dim">Current v</div>
                  <div className="font-mono text-lg text-info">{telemetry?.cmd_vel?.v?.toFixed(3)} m/s</div>
                </div>
                <div>
                  <div className="text-xs text-dim">Current ω</div>
                  <div className="font-mono text-lg text-info">{telemetry?.cmd_vel?.omega?.toFixed(3)} rad/s</div>
                </div>
             </div>
          </div>
        </div>

      </div>
    </div>
  );
}
