"""
AVS Dashboard System — Dashboard API Control Node
File: avs_dashboard_system/dashboard_api_control_node.py

Provides the WebSocket + HTTP API server for the web dashboard.
Uses aiohttp (lightweight, WebSocket-native) running in asyncio loop
in a background thread, while ROS 2 spins in the main thread.

Architecture:
  Main thread:  rclpy.spin() → ROS callbacks → update shared state
  Background:   asyncio event loop → aiohttp server → WebSocket + REST

WebSocket Protocol (ws://host:port/ws):
  Server → Client (push, 10 Hz):
    {"type": "telemetry", "data": <dashboard_state JSON>}
    {"type": "process_status", "data": {...}}
    {"type": "controller_list", "data": {...}}
    {"type": "gazebo_status", "data": {...}}
    {"type": "experiment_status", "data": {...}}
    {"type": "log", "level": "info|warn|error", "msg": "..."}

  Client → Server (commands):
    {"type": "cmd", "action": "switch_controller", "data": {"name": "main_pd"}}
    {"type": "cmd", "action": "emergency_stop",    "data": {"active": true}}
    {"type": "cmd", "action": "emergency_reset",   "data": {}}
    {"type": "cmd", "action": "manual_cmd",        "data": {"v": 0.05, "omega": 0.1}}
    {"type": "cmd", "action": "start_experiment",  "data": {metadata}}
    {"type": "cmd", "action": "stop_experiment",   "data": {}}
    {"type": "cmd", "action": "mark_event",        "data": {"label": "..."}}
    {"type": "cmd", "action": "start_process",     "data": {"name": "..."}}
    {"type": "cmd", "action": "stop_process",      "data": {"name": "..."}}
    {"type": "cmd", "action": "start_gazebo",      "data": {"world": "city_lane"}}
    {"type": "cmd", "action": "stop_gazebo",       "data": {}}
    {"type": "cmd", "action": "compare_experiments","data": {"ids": [], "scenario": ""}}
    {"type": "cmd", "action": "list_experiments",  "data": {}}
    {"type": "cmd", "action": "get_status",        "data": {}}

HTTP REST endpoints (for file downloads, non-realtime):
  GET  /api/status              → current dashboard state JSON
  GET  /api/experiments         → list of experiment directories
  GET  /api/experiments/{id}/summary → experiment summary JSON
  GET  /api/experiments/{id}/csv     → download data.csv
  GET  /api/experiments/{id}/plot/{name} → download plot PNG
  POST /api/experiments/compare      → trigger comparison
  GET  /health                       → health check

All other operations go through WebSocket for speed.
"""

import asyncio
import json
import logging
import os
import threading
import time
from pathlib import Path
from typing import Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

try:
    from aiohttp import web
    import aiohttp
    AIOHTTP_AVAILABLE = True
except ImportError:
    AIOHTTP_AVAILABLE = False

from avs_dashboard_system.utils import parse_json_string

RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE
)

# Suppress aiohttp access logs in production
logging.getLogger('aiohttp.access').setLevel(logging.WARNING)


class SharedState:
    """Thread-safe shared state between ROS callbacks and asyncio handlers."""

    def __init__(self):
        self._lock = threading.Lock()
        self._data = {
            'dashboard_state': {},
            'process_status': {},
            'controller_list': {},
            'gazebo_status': {},
            'experiment_status': {},
            'experiment_list': {},
            'experiment_summary': {},
            'controller_comparison': {},
        }
        self._log_buffer: list = []
        self._max_log = 500

    def update(self, key: str, value: dict):
        with self._lock:
            self._data[key] = value

    def get(self, key: str) -> dict:
        with self._lock:
            return dict(self._data.get(key, {}))

    def get_all(self) -> dict:
        with self._lock:
            return dict(self._data)

    def add_log(self, level: str, msg: str):
        with self._lock:
            entry = {
                'time': time.time(),
                'level': level,
                'msg': msg,
            }
            self._log_buffer.append(entry)
            if len(self._log_buffer) > self._max_log:
                self._log_buffer.pop(0)

    def get_recent_logs(self, n: int = 50) -> list:
        with self._lock:
            return list(self._log_buffer[-n:])


class DashboardApiControlNode(Node):
    """
    ROS 2 node that runs an aiohttp WebSocket+HTTP server.
    ROS spins in main thread; aiohttp runs in background asyncio thread.
    """

    def __init__(self):
        super().__init__('dashboard_api_control')

        # Parameters
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 8080)
        self.declare_parameter('telemetry_push_hz', 10.0)
        self.declare_parameter('base_dir', '~/avs_experiments')
        self.declare_parameter('static_dir', '')

        # Topics (configurable)
        self.declare_parameter('topic_dashboard_state', '/avs/dashboard_state')
        self.declare_parameter('topic_process_status', '/avs/process_status')
        self.declare_parameter('topic_controller_list', '/avs/controller_list')
        self.declare_parameter('topic_gazebo_status', '/avs/gazebo_status')
        self.declare_parameter('topic_experiment_status', '/avs/experiment_status')
        self.declare_parameter('topic_experiment_list', '/avs/experiment_list')
        self.declare_parameter('topic_experiment_summary', '/avs/experiment_summary')
        self.declare_parameter('topic_controller_comparison', '/avs/controller_comparison')
        self.declare_parameter('topic_cmd_manual', '/avs/cmd_vel/manual')
        self.declare_parameter('topic_supervisor_cmd', '/avs/supervisor_cmd')
        self.declare_parameter('topic_process_cmd', '/avs/process_cmd')
        self.declare_parameter('topic_gazebo_cmd', '/avs/gazebo_cmd')
        self.declare_parameter('topic_experiment_cmd', '/avs/experiment/cmd')
        self.declare_parameter('topic_analyzer_cmd', '/avs/analyzer_cmd')
        self.declare_parameter('topic_emergency_stop', '/avs/emergency_stop')

        self._host = self.get_parameter('web_host').value
        self._port = self.get_parameter('web_port').value
        self._push_hz = self.get_parameter('telemetry_push_hz').value
        self._base_dir = Path(
            self.get_parameter('base_dir').value).expanduser()

        # Shared state
        self._state = SharedState()

        # Connected WebSocket clients
        self._ws_clients: Set[aiohttp.web.WebSocketResponse] = set()
        self._ws_lock = asyncio.Lock() if False else threading.Lock()  # use asyncio lock in async context

        # asyncio event loop (background thread)
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._server_task = None

        # Publishers
        r10 = RELIABLE_QOS
        self._pub_manual = self.create_publisher(
            Twist, self.get_parameter('topic_cmd_manual').value, r10)
        self._pub_supervisor = self.create_publisher(
            String, self.get_parameter('topic_supervisor_cmd').value, r10)
        self._pub_process_cmd = self.create_publisher(
            String, self.get_parameter('topic_process_cmd').value, r10)
        self._pub_gazebo_cmd = self.create_publisher(
            String, self.get_parameter('topic_gazebo_cmd').value, r10)
        self._pub_exp_cmd = self.create_publisher(
            String, self.get_parameter('topic_experiment_cmd').value, r10)
        self._pub_analyzer = self.create_publisher(
            String, self.get_parameter('topic_analyzer_cmd').value, r10)
        self._pub_estop = self.create_publisher(
            Bool, self.get_parameter('topic_emergency_stop').value, r10)

        # Subscribers
        def make_state_cb(key):
            def cb(msg: String):
                data = parse_json_string(msg.data, default={})
                self._state.update(key, data)
                # Schedule push to all WS clients
                if self._loop and not self._loop.is_closed():
                    asyncio.run_coroutine_threadsafe(
                        self._broadcast_update(key, data), self._loop)
            return cb

        self.create_subscription(String,
            self.get_parameter('topic_dashboard_state').value,
            make_state_cb('dashboard_state'), r10)
        self.create_subscription(String,
            self.get_parameter('topic_process_status').value,
            make_state_cb('process_status'), r10)
        self.create_subscription(String,
            self.get_parameter('topic_controller_list').value,
            make_state_cb('controller_list'), r10)
        self.create_subscription(String,
            self.get_parameter('topic_gazebo_status').value,
            make_state_cb('gazebo_status'), r10)
        self.create_subscription(String,
            self.get_parameter('topic_experiment_status').value,
            make_state_cb('experiment_status'), r10)
        self.create_subscription(String,
            self.get_parameter('topic_experiment_list').value,
            make_state_cb('experiment_list'), r10)
        self.create_subscription(String,
            self.get_parameter('topic_experiment_summary').value,
            make_state_cb('experiment_summary'), r10)
        self.create_subscription(String,
            self.get_parameter('topic_controller_comparison').value,
            make_state_cb('controller_comparison'), r10)

        # Timer for /cmd_vel safety check
        self.create_timer(1.0, self._check_cmd_vel_publishers)

        # Start aiohttp server in background thread
        if AIOHTTP_AVAILABLE:
            self._server_thread = threading.Thread(
                target=self._run_server_thread,
                daemon=True, name='aiohttp-server')
            self._server_thread.start()
        else:
            self.get_logger().error(
                '[dashboard_api] aiohttp not installed! '
                'Run: pip install aiohttp')

        self.get_logger().info(
            f'[dashboard_api_control] Server starting at '
            f'http://{self._host}:{self._port}')

    # =========================================================================
    # aiohttp server setup
    # =========================================================================

    def _run_server_thread(self):
        """Run asyncio event loop with aiohttp in background thread."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())

    async def _serve(self):
        app = web.Application()
        self._ws_clients_async: Set = set()

        # Routes
        app.router.add_get('/ws', self._ws_handler)
        app.router.add_get('/health', self._http_health)
        app.router.add_get('/api/status', self._http_status)
        app.router.add_get('/api/experiments', self._http_list_experiments)
        app.router.add_get(
            '/api/experiments/{exp_id}/summary', self._http_exp_summary)
        app.router.add_get(
            '/api/experiments/{exp_id}/csv', self._http_exp_csv)
        app.router.add_get(
            '/api/experiments/{exp_id}/plot/{plot_name}', self._http_exp_plot)
        app.router.add_post(
            '/api/experiments/compare', self._http_compare)

        # Serve frontend static files
        static_dir = self.get_parameter('static_dir').value
        if not static_dir:
            # Try to find web/dist relative to this file
            pkg_dir = Path(__file__).resolve().parent.parent
            candidates = [
                pkg_dir / 'web' / 'dist',
                pkg_dir / 'web' / 'build',
            ]
            for c in candidates:
                if c.exists():
                    static_dir = str(c)
                    break

        if static_dir and Path(static_dir).exists():
            app.router.add_static('/assets', static_dir + '/assets',
                                   name='assets')
            app.router.add_get('/{path_info:.*}', self._serve_spa)
            self._static_dir = static_dir
            self.get_logger().info(
                f'[dashboard_api] Serving frontend from: {static_dir}')
        else:
            self._static_dir = None
            self.get_logger().warn(
                '[dashboard_api] Frontend build not found. '
                'Run: cd web && npm run build')

        # CORS middleware
        app.middlewares.append(self._cors_middleware)

        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, self._host, self._port)
        await site.start()

        self.get_logger().info(
            f'[dashboard_api_control] ✅ Server ready: '
            f'http://{self._host}:{self._port}')

        # Keep running until node shuts down
        while rclpy.ok():
            await asyncio.sleep(1.0)

        await runner.cleanup()

    @web.middleware
    async def _cors_middleware(self, request, handler):
        response = await handler(request)
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Methods'] = \
            'GET, POST, OPTIONS'
        response.headers['Access-Control-Allow-Headers'] = \
            'Content-Type, Authorization'
        return response

    # =========================================================================
    # WebSocket handler
    # =========================================================================

    async def _ws_handler(self, request):
        ws = web.WebSocketResponse(heartbeat=30.0)
        await ws.prepare(request)

        self._ws_clients_async.add(ws)
        client_ip = request.remote
        self.get_logger().info(
            f'[dashboard_api] WS connected: {client_ip} | '
            f'total={len(self._ws_clients_async)}')

        # Send initial full state on connect
        await self._send_full_state(ws)

        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    await self._handle_ws_message(ws, msg.data)
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    self.get_logger().warn(
                        f'[dashboard_api] WS error: {ws.exception()}')
                    break
        except Exception as e:
            self.get_logger().warn(
                f'[dashboard_api] WS exception: {e}')
        finally:
            self._ws_clients_async.discard(ws)
            self.get_logger().info(
                f'[dashboard_api] WS disconnected: {client_ip} | '
                f'total={len(self._ws_clients_async)}')

        return ws

    async def _send_full_state(self, ws):
        """Send all current state to a newly connected client."""
        all_state = self._state.get_all()
        for key, data in all_state.items():
            if data:
                msg = json.dumps({'type': key, 'data': data})
                try:
                    await ws.send_str(msg)
                except Exception:
                    pass
        # Send recent logs
        logs = self._state.get_recent_logs(50)
        for log in logs:
            try:
                await ws.send_str(json.dumps({
                    'type': 'log',
                    'level': log['level'],
                    'msg': log['msg'],
                    'time': log['time'],
                }))
            except Exception:
                pass

    async def _broadcast_update(self, msg_type: str, data: dict):
        """Broadcast a state update to all connected WS clients."""
        if not self._ws_clients_async:
            return
        payload = json.dumps({'type': msg_type, 'data': data})
        dead = set()
        for ws in list(self._ws_clients_async):
            try:
                await ws.send_str(payload)
            except Exception:
                dead.add(ws)
        self._ws_clients_async -= dead

    async def _handle_ws_message(self, ws, raw: str):
        """Handle incoming WebSocket command from frontend."""
        data = parse_json_string(raw)
        if not data or data.get('type') != 'cmd':
            return

        action = data.get('action', '')
        payload = data.get('data', {})

        handlers = {
            'switch_controller': self._cmd_switch_controller,
            'stop_controller':   self._cmd_stop_controller,
            'emergency_stop':    self._cmd_emergency_stop,
            'emergency_reset':   self._cmd_emergency_reset,
            'manual_cmd':        self._cmd_manual_cmd,
            'start_experiment':  self._cmd_start_experiment,
            'stop_experiment':   self._cmd_stop_experiment,
            'mark_event':        self._cmd_mark_event,
            'start_process':     self._cmd_start_process,
            'stop_process':      self._cmd_stop_process,
            'restart_process':   self._cmd_restart_process,
            'start_gazebo':      self._cmd_start_gazebo,
            'stop_gazebo':       self._cmd_stop_gazebo,
            'reset_gazebo':      self._cmd_reset_gazebo,
            'compare_experiments': self._cmd_compare_experiments,
            'list_experiments':  self._cmd_list_experiments,
            'analyzer_cmd':      self._cmd_analyzer_cmd,
            'set_parameters':    self._cmd_set_parameters,
            'ros_list_nodes':    self._cmd_ros_list_nodes,
            'ros_list_topics':   self._cmd_ros_list_topics,
            'ros_list_packages': self._cmd_ros_list_packages,
            'get_status':        self._cmd_get_status,
        }

        handler = handlers.get(action)
        if handler:
            await handler(ws, payload)
        else:
            await ws.send_str(json.dumps({
                'type': 'error',
                'msg': f'Unknown action: {action}'
            }))

    # =========================================================================
    # WebSocket command handlers
    # =========================================================================

    async def _cmd_switch_controller(self, ws, data: dict):
        name = data.get('name', 'off')
        self._publish_supervisor_cmd('switch_controller', {'name': name})
        await self._ack(ws, 'switch_controller', {'name': name})

    async def _cmd_stop_controller(self, ws, data: dict):
        self._publish_supervisor_cmd('switch_controller', {'name': 'off'})
        await self._ack(ws, 'stop_controller', {})

    async def _cmd_emergency_stop(self, ws, data: dict):
        self._publish_supervisor_cmd('emergency_stop', {})
        msg = Bool()
        msg.data = True
        self._pub_estop.publish(msg)
        self._state.add_log('error', '⚠ EMERGENCY STOP activated from dashboard')
        await self._ack(ws, 'emergency_stop', {'active': True})

    async def _cmd_emergency_reset(self, ws, data: dict):
        self._publish_supervisor_cmd('emergency_reset', {})
        msg = Bool()
        msg.data = False
        self._pub_estop.publish(msg)
        self._state.add_log('info', 'Emergency stop cleared')
        await self._ack(ws, 'emergency_reset', {})

    async def _cmd_manual_cmd(self, ws, data: dict):
        twist = Twist()
        twist.linear.x  = float(data.get('v', 0.0))
        twist.angular.z = float(data.get('omega', 0.0))
        self._pub_manual.publish(twist)
        # No ack for high-frequency manual commands (joystick)

    async def _cmd_start_experiment(self, ws, data: dict):
        msg = String()
        msg.data = json.dumps({'action': 'start', 'metadata': data})
        self._pub_exp_cmd.publish(msg)
        self._state.add_log('info',
            f'Experiment started: {data.get("controller_name")} / '
            f'{data.get("scenario_name")}')
        await self._ack(ws, 'start_experiment', data)

    async def _cmd_stop_experiment(self, ws, data: dict):
        msg = String()
        msg.data = json.dumps({'action': 'stop'})
        self._pub_exp_cmd.publish(msg)
        self._state.add_log('info', 'Experiment stopped')
        await self._ack(ws, 'stop_experiment', {})

    async def _cmd_mark_event(self, ws, data: dict):
        msg = String()
        msg.data = json.dumps({
            'action': 'mark_event', 'label': data.get('label', 'event')})
        self._pub_exp_cmd.publish(msg)
        await self._ack(ws, 'mark_event', data)

    async def _cmd_start_process(self, ws, data: dict):
        msg = String()
        msg.data = json.dumps({'action': 'start', 'name': data.get('name', '')})
        self._pub_process_cmd.publish(msg)
        await self._ack(ws, 'start_process', data)

    async def _cmd_stop_process(self, ws, data: dict):
        msg = String()
        msg.data = json.dumps({'action': 'stop', 'name': data.get('name', '')})
        self._pub_process_cmd.publish(msg)
        await self._ack(ws, 'stop_process', data)

    async def _cmd_restart_process(self, ws, data: dict):
        msg = String()
        msg.data = json.dumps({'action': 'restart', 'name': data.get('name', '')})
        self._pub_process_cmd.publish(msg)
        await self._ack(ws, 'restart_process', data)

    async def _cmd_start_gazebo(self, ws, data: dict):
        msg = String()
        msg.data = json.dumps({'action': 'start', 'params': data})
        self._pub_gazebo_cmd.publish(msg)
        await self._ack(ws, 'start_gazebo', data)

    async def _cmd_stop_gazebo(self, ws, data: dict):
        msg = String()
        msg.data = json.dumps({'action': 'stop', 'params': {}})
        self._pub_gazebo_cmd.publish(msg)
        await self._ack(ws, 'stop_gazebo', {})

    async def _cmd_reset_gazebo(self, ws, data: dict):
        msg = String()
        msg.data = json.dumps({'action': 'reset', 'params': {}})
        self._pub_gazebo_cmd.publish(msg)
        await self._ack(ws, 'reset_gazebo', {})

    async def _cmd_compare_experiments(self, ws, data: dict):
        msg = String()
        msg.data = json.dumps({
            'action': 'compare',
            'experiment_ids': data.get('ids', []),
            'scenario': data.get('scenario', ''),
        })
        self._pub_analyzer.publish(msg)
        await self._ack(ws, 'compare_experiments', data)

    async def _cmd_list_experiments(self, ws, data: dict):
        msg = String()
        msg.data = json.dumps({'action': 'list_experiments'})
        self._pub_analyzer.publish(msg)
        await self._ack(ws, 'list_experiments', {})

    # =========================================================================
    # ROS Introspection
    # =========================================================================
    
    def _check_cmd_vel_publishers(self):
        # Native rclpy call
        try:
            pubs = self.get_publishers_info_by_topic('/cmd_vel')
            pub_list = [{'node': p.node_name, 'type': p.topic_type} for p in pubs]
            
            # Also get node list
            nodes = self.get_node_names_and_namespaces()
            node_list = [f"{ns}/{n}".replace('//', '/') for n, ns in nodes]
            
            data = {
                'cmd_vel_publishers': pub_list,
                'cmd_vel_count': len(pub_list),
                'node_count': len(node_list)
            }
            self._state.update('ros_introspection', data)
            if self._loop and not self._loop.is_closed():
                asyncio.run_coroutine_threadsafe(
                    self._broadcast_update('ros_introspection', data), self._loop)
        except Exception as e:
            self.get_logger().error(f"Introspection error: {e}")

    async def _cmd_ros_list_nodes(self, ws, data: dict):
        nodes = self.get_node_names_and_namespaces()
        node_list = []
        for n, ns in nodes:
            node_list.append({'name': n, 'namespace': ns})
        await self._ack(ws, 'ros_list_nodes', {'nodes': node_list})

    async def _cmd_ros_list_topics(self, ws, data: dict):
        topics = self.get_topic_names_and_types()
        topic_list = []
        for t, types in topics:
            topic_list.append({'name': t, 'types': types})
        await self._ack(ws, 'ros_list_topics', {'topics': topic_list})

    async def _cmd_ros_list_packages(self, ws, data: dict):
        # This is a bit slow and requires ament_index_python
        try:
            from ament_index_python.packages import get_packages_with_prefixes
            pkgs = get_packages_with_prefixes()
            pkg_list = [{'name': p, 'path': pkgs[p]} for p in pkgs.keys()]
        except Exception:
            pkg_list = []
        await self._ack(ws, 'ros_list_packages', {'packages': pkg_list})

    async def _cmd_ros_list_executables(self, ws, data: dict):
        pkg_name = data.get('package', '')
        if not pkg_name:
            await self._ack(ws, 'ros_list_executables', {'package': '', 'executables': []})
            return
        
        try:
            # We can run `ros2 pkg executables <pkg>`
            import subprocess
            result = subprocess.run(['ros2', 'pkg', 'executables', pkg_name], capture_output=True, text=True, timeout=5.0)
            execs = []
            for line in result.stdout.split('\n'):
                line = line.strip()
                if line and line.startswith(pkg_name):
                    parts = line.split()
                    if len(parts) >= 2:
                        execs.append(parts[1])
            await self._ack(ws, 'ros_list_executables', {'package': pkg_name, 'executables': execs})
        except Exception as e:
            self.get_logger().error(f"Executables fetch error: {e}")
            await self._ack(ws, 'ros_list_executables', {'package': pkg_name, 'executables': [], 'error': str(e)})

    async def _cmd_ros_node_params(self, ws, data: dict):
        node_name = data.get('node_name', '')
        if not node_name:
            await self._ack(ws, 'ros_node_params', {'node_name': '', 'params': {}})
            return
        try:
            import subprocess
            result = subprocess.run(['ros2', 'param', 'dump', node_name], capture_output=True, text=True, timeout=5.0)
            
            import yaml
            parsed = yaml.safe_load(result.stdout)
            params = {}
            if parsed and isinstance(parsed, dict):
                # The format is usually { node_name: { ros__parameters: { ... } } }
                node_key = list(parsed.keys())[0] if parsed else None
                if node_key and 'ros__parameters' in parsed[node_key]:
                    params = parsed[node_key]['ros__parameters']
                    
            await self._ack(ws, 'ros_node_params', {'node_name': node_name, 'params': params})
        except Exception as e:
            self.get_logger().error(f"Param fetch error: {e}")
            await self._ack(ws, 'ros_node_params', {'node_name': node_name, 'params': {}, 'error': str(e)})

    async def _cmd_get_status(self, ws, data: dict):
        await self._send_full_state(ws)

    async def _cmd_analyzer_cmd(self, ws, data: dict):
        msg = String()
        msg.data = json.dumps(data)
        self._pub_analyzer.publish(msg)
        await self._ack(ws, 'analyzer_cmd', {})

    async def _cmd_set_parameters(self, ws, data: dict):
        node_name = data.get('node', 'unknown')
        params_dict = data.get('params', {})
        try:
            import subprocess
            for k, v in params_dict.items():
                # convert type for ros2 param set
                val_str = str(v)
                if isinstance(v, bool): val_str = 'true' if v else 'false'
                subprocess.run(['ros2', 'param', 'set', node_name, k, val_str], capture_output=True, text=True, timeout=2.0)
                
            self._state.add_log('info', f'Parameters updated for {node_name}')
            await self._ack(ws, 'set_parameters', {'node': node_name, 'success': True})
        except Exception as e:
            self._state.add_log('error', f'Failed to set params: {e}')
            await self._ack(ws, 'set_parameters', {'node': node_name, 'success': False, 'error': str(e)})

    async def _ack(self, ws, action: str, data: dict):
        try:
            await ws.send_str(json.dumps({
                'type': 'ack',
                'action': action,
                'data': data,
                'time': time.time(),
            }))
        except Exception:
            pass

    # =========================================================================
    # HTTP REST handlers
    # =========================================================================

    async def _http_health(self, request):
        return web.json_response({'status': 'ok', 'time': time.time()})

    async def _http_status(self, request):
        return web.json_response(self._state.get('dashboard_state'))

    async def _http_list_experiments(self, request):
        return web.json_response(self._state.get('experiment_list'))

    async def _http_exp_summary(self, request):
        exp_id = request.match_info['exp_id']
        summary_path = self._base_dir / exp_id / 'summary.json'
        if not summary_path.exists():
            return web.json_response({'error': 'Not found'}, status=404)
        with open(summary_path) as f:
            return web.json_response(json.load(f))

    async def _http_exp_csv(self, request):
        exp_id = request.match_info['exp_id']
        csv_path = self._base_dir / exp_id / 'data.csv'
        if not csv_path.exists():
            return web.Response(text='Not found', status=404)
        return web.FileResponse(csv_path, headers={
            'Content-Disposition':
                f'attachment; filename="{exp_id}_data.csv"'
        })

    async def _http_exp_plot(self, request):
        exp_id    = request.match_info['exp_id']
        plot_name = request.match_info['plot_name']
        plot_path = self._base_dir / exp_id / 'plots' / plot_name
        if not plot_path.exists():
            return web.Response(text='Not found', status=404)
        return web.FileResponse(plot_path)

    async def _http_compare(self, request):
        data = await request.json()
        msg = String()
        msg.data = json.dumps({
            'action': 'compare',
            'experiment_ids': data.get('ids', []),
            'scenario': data.get('scenario', ''),
        })
        self._pub_analyzer.publish(msg)
        return web.json_response({'status': 'comparison_triggered'})

    async def _serve_spa(self, request):
        """Serve React SPA index.html for all non-API routes."""
        if self._static_dir:
            index = Path(self._static_dir) / 'index.html'
            if index.exists():
                return web.FileResponse(index)
        return web.Response(text='Frontend not built. Run: cd web && npm run build',
                            status=200)

    # =========================================================================
    # ROS publisher helpers
    # =========================================================================

    def _publish_supervisor_cmd(self, action: str, params: dict):
        msg = String()
        msg.data = json.dumps({'action': action, 'params': params})
        self._pub_supervisor.publish(msg)


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = DashboardApiControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
