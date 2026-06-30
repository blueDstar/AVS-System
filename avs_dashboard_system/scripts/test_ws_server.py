#!/usr/bin/env python3
"""
AVS Dashboard — Standalone WebSocket Test Server
File: scripts/test_ws_server.py

Runs a minimal aiohttp WebSocket server that simulates the dashboard_api_control
node WITHOUT needing ROS 2 installed. Use this to:
  1. Test the React frontend connectivity
  2. Verify WebSocket protocol is working
  3. Develop/debug UI without a robot

Usage:
  python3 scripts/test_ws_server.py [port]

Then open: http://localhost:8080 (after building the frontend)
Or in dev:  npm run dev (in web/) — proxy will forward to port 8080
"""

import asyncio
import json
import math
import os
import sys
import time
from pathlib import Path

try:
    from aiohttp import web
    import aiohttp
except ImportError:
    print("ERROR: aiohttp not installed. Run: pip install aiohttp")
    sys.exit(1)

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080


# ============================================================================
# Simulated telemetry generator
# ============================================================================

def make_telemetry(t: float) -> dict:
    """Generate realistic-looking fake telemetry for UI testing."""
    v_ref    = 0.05 + 0.02 * math.sin(t * 0.5)
    v_meas   = v_ref + 0.002 * math.sin(t * 10.7)
    omega    = 0.3 * math.sin(t * 0.3)
    eps_x    = 30 * math.sin(t * 0.8) + 5 * math.cos(t * 3.1)
    theta    = 0.1 * math.sin(t * 0.7)
    
    # Integrate simple position
    x = 0.5 * math.cos(t * 0.2)
    y = 0.5 * math.sin(t * 0.2)
    yaw = t * 0.2

    return {
        "time": time.time(),
        "system": {
            "ros_domain_id": 20,
            "mode": "real_robot",
            "active_controller": "main_pd" if t > 5 else "off",
            "emergency_stop": False,
            "cmd_vel_hz": round(20.0 + 0.5 * math.sin(t), 2),
        },
        "cmd_vel": {
            "v": round(v_ref, 4),
            "omega": round(omega, 4),
            "hz": round(20.0, 2),
        },
        "odom": {
            "x": round(x, 4),
            "y": round(y, 4),
            "yaw": round(yaw % (2 * math.pi), 4),
            "v": round(v_meas, 4),
            "omega": round(omega + 0.01 * math.sin(t * 5), 4),
            "hz": round(10.0, 2),
            "timeout": False,
        },
        "imu": {
            "yaw": round(yaw, 4),
            "wz":  round(omega + 0.01, 4),
            "hz":  round(100.0, 2),
            "timeout": False,
        },
        "lidar": {
            "front_min": round(max(0.3, 1.5 + 1.0 * math.sin(t * 0.15)), 3),
            "left_min":  round(max(0.2, 0.8 + 0.5 * math.cos(t * 0.25)), 3),
            "right_min": round(max(0.2, 0.9 + 0.4 * math.sin(t * 0.3)),  3),
            "hz":  round(10.0, 2),
            "timeout": False,
        },
        "lane": {
            "valid": True,
            "state": "TRACKING",
            "epsilon_x_mm": round(eps_x, 2),
            "epsilon_y_mm": round(eps_x * 0.3, 2),
            "theta_rad":    round(theta, 4),
            "fps_est":      round(30.0 + math.sin(t), 2),
            "hz":           round(30.0, 2),
            "timeout": False,
        },
        "controller_debug": {
            "v_left_ref":   round(v_ref + 0.5 * omega * 0.15, 4),
            "v_right_ref":  round(v_ref - 0.5 * omega * 0.15, 4),
            "v_left_meas":  round(v_meas + 0.5 * omega * 0.15 + 0.002 * math.sin(t * 11), 4),
            "v_right_meas": round(v_meas - 0.5 * omega * 0.15 + 0.002 * math.cos(t * 13), 4),
        },
    }


# ============================================================================
# WebSocket handler
# ============================================================================

connected_clients = set()
start_time = time.time()


async def ws_handler(request):
    ws = web.WebSocketResponse(heartbeat=30.0)
    await ws.prepare(request)
    connected_clients.add(ws)

    print(f"[WS] Client connected: {request.remote} (total: {len(connected_clients)})")

    # Send initial fake data
    await ws.send_str(json.dumps({
        "type": "controller_list",
        "data": {
            "active_controller": "off",
            "emergency_stop": False,
            "controllers": ["off", "manual", "main_pd", "cascade_pd", "backstepping_pd", "pd_lidar"],
        }
    }))
    await ws.send_str(json.dumps({
        "type": "gazebo_status",
        "data": {
            "gazebo_status": "stopped",
            "current_world": None,
            "target_runtime": "real_robot",
            "available_worlds": {
                "city_lane":   {"display_name": "City Lane",   "recommended_speed": 0.06},
                "figure8_lane":{"display_name": "Figure-8",    "recommended_speed": 0.05},
            },
        }
    }))
    await ws.send_str(json.dumps({
        "type": "process_status",
        "data": {
            "processes": [
                {"name": "micro_ros_agent", "description": "micro-ROS Agent", "group": "infrastructure",
                 "status": "stopped", "pid": None, "running": False, "uptime_s": 0, "cpu_percent": 0, "ram_mb": 0, "recent_log": []},
                {"name": "perception",      "description": "AVS Perception Stack", "group": "perception",
                 "status": "running", "pid": 12345, "running": True, "uptime_s": 120, "cpu_percent": 12.5, "ram_mb": 450, "recent_log": ["[INFO] lane detected"]},
            ]
        }
    }))
    await ws.send_str(json.dumps({"type": "log", "level": "info",  "msg": "Test server connected", "time": time.time()}))
    await ws.send_str(json.dumps({"type": "log", "level": "warn",  "msg": "This is a SIMULATION test server (no real ROS)", "time": time.time()}))

    try:
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                try:
                    data = json.loads(msg.data)
                    action = data.get("action", "")
                    print(f"[WS] Command: {action} | {data.get('data', {})}")
                    # Echo back an ack
                    await ws.send_str(json.dumps({"type": "ack", "action": action, "data": {}, "time": time.time()}))
                    await ws.send_str(json.dumps({"type": "log", "level": "info", "msg": f"Action received: {action}", "time": time.time()}))
                except Exception as e:
                    print(f"[WS] Parse error: {e}")
            elif msg.type == aiohttp.WSMsgType.ERROR:
                break
    finally:
        connected_clients.discard(ws)
        print(f"[WS] Client disconnected (remaining: {len(connected_clients)})")

    return ws


# ============================================================================
# Telemetry push loop
# ============================================================================

async def push_telemetry():
    global connected_clients
    while True:
        await asyncio.sleep(0.1)  # 10 Hz
        if not connected_clients:
            continue
        t = time.time() - start_time
        payload = json.dumps({"type": "dashboard_state", "data": make_telemetry(t)})
        dead = set()
        for ws in list(connected_clients):
            try:
                await ws.send_str(payload)
            except Exception:
                dead.add(ws)
        connected_clients -= dead


# ============================================================================
# HTTP health endpoint
# ============================================================================

async def health(request):
    return web.json_response({"status": "ok", "server": "avs_test_server", "time": time.time()})


# ============================================================================
# Serve frontend (if built)
# ============================================================================

async def serve_spa(request):
    static_dir = Path(__file__).parent.parent / "web" / "dist"
    index = static_dir / "index.html"
    if index.exists():
        return web.FileResponse(index)
    return web.Response(
        text="<h1>AVS Test Server Running</h1>"
             "<p>Connect WebSocket to ws://localhost:{PORT}/ws</p>"
             "<p>Build frontend first: <code>cd web && npm install && npm run build</code></p>",
        content_type="text/html"
    )


# ============================================================================
# Main
# ============================================================================

async def main():
    app = web.Application()
    app.router.add_get("/ws",      ws_handler)
    app.router.add_get("/health",  health)
    app.router.add_get("/{path:.*}", serve_spa)

    # Serve /assets if dist exists
    dist = Path(__file__).parent.parent / "web" / "dist"
    assets = dist / "assets"
    if assets.exists():
        app.router.add_static("/assets", str(assets))

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", PORT)
    await site.start()

    print(f"""
╔══════════════════════════════════════════════════╗
║     AVS Dashboard — Test WS Server              ║
╠══════════════════════════════════════════════════╣
║  WebSocket: ws://localhost:{PORT}/ws               ║
║  Dashboard: http://localhost:{PORT}                ║
║  Health:    http://localhost:{PORT}/health          ║
║                                                  ║
║  Generating FAKE telemetry at 10 Hz              ║
║  Press Ctrl+C to stop                           ║
╚══════════════════════════════════════════════════╝
""")

    await asyncio.gather(
        push_telemetry(),
        asyncio.get_event_loop().create_future(),  # keep running
    )


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[INFO] Server stopped.")
