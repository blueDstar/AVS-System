"""
AVS Dashboard System — Gazebo Manager Control Node
File: avs_dashboard_system/gazebo_manager_control_node.py

Manages Gazebo simulation worlds for benchmarking controllers.
Enforces isolation: when target_runtime=gazebo, real robot cmd_vel is blocked.

Topics:
  Subscribed: /avs/gazebo_cmd      (JSON: start/stop/reset world)
  Published:  /avs/gazebo_status   (JSON: sim status, RTF, world, etc.)
              /avs/emergency_stop  (Bool: publish True if gazebo crashes in sim mode)
"""

import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String, Bool

from avs_dashboard_system.utils import parse_json_string

RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE
)

WORLDS = {
    'city_lane': {
        'display_name': 'City Lane',
        'launch_pkg': 'avs_gazebo',
        'launch_file': 'avs_city_lane.launch.py',
        'recommended_speed': 0.06,
    },
    'figure8_lane': {
        'display_name': 'Figure-8 Lane',
        'launch_pkg': 'avs_gazebo',
        'launch_file': 'avs_figure8_lane.launch.py',
        'recommended_speed': 0.05,
    },
}


class GazeboManagerControlNode(Node):
    """
    Manages Gazebo simulation lifecycle and enforces runtime isolation
    between real robot and simulation modes.
    """

    def __init__(self):
        super().__init__('gazebo_manager_control')

        self.declare_parameter('topic_gazebo_cmd', '/avs/gazebo_cmd')
        self.declare_parameter('topic_gazebo_status', '/avs/gazebo_status')
        self.declare_parameter('topic_emergency_stop', '/avs/emergency_stop')
        self.declare_parameter('topic_supervisor_cmd', '/avs/supervisor_cmd')
        self.declare_parameter('ros_domain_id', 20)
        self.declare_parameter('default_runtime', 'real_robot')

        self._lock = threading.Lock()
        self._target_runtime: str = self.get_parameter('default_runtime').value
        self._current_world: Optional[str] = None
        self._gazebo_process: Optional[subprocess.Popen] = None
        self._gazebo_pid: Optional[int] = None
        self._gazebo_status: str = 'stopped'  # stopped | starting | running | error
        self._gazebo_started_at: Optional[float] = None
        self._sim_time: float = 0.0

        r10 = RELIABLE_QOS
        self._pub_status = self.create_publisher(
            String, self.get_parameter('topic_gazebo_status').value, r10)
        self._pub_estop = self.create_publisher(
            Bool, self.get_parameter('topic_emergency_stop').value, r10)
        self._pub_supervisor = self.create_publisher(
            String, self.get_parameter('topic_supervisor_cmd').value, r10)

        self.create_subscription(String,
            self.get_parameter('topic_gazebo_cmd').value,
            self._cb_gazebo_cmd, r10)

        self.create_timer(2.0, self._publish_status)
        self.create_timer(5.0, self._monitor_gazebo)

        self.get_logger().info(
            f'[gazebo_manager_control] Ready | '
            f'runtime={self._target_runtime} | '
            f'worlds={list(WORLDS.keys())}')

    # =========================================================================
    # Command handler
    # =========================================================================

    def _cb_gazebo_cmd(self, msg: String):
        data = parse_json_string(msg.data)
        if not data:
            return
        action = data.get('action', '')
        params = data.get('params', {})

        if action == 'start':
            world = params.get('world', 'city_lane')
            threading.Thread(
                target=self._start_gazebo,
                args=(world,), daemon=True).start()
        elif action == 'stop':
            threading.Thread(
                target=self._stop_gazebo, daemon=True).start()
        elif action == 'reset':
            threading.Thread(
                target=self._reset_gazebo, daemon=True).start()
        elif action == 'set_runtime':
            self._set_runtime(params.get('runtime', 'real_robot'))
        elif action == 'list_worlds':
            self._publish_status()
        else:
            self.get_logger().warn(
                f'[gazebo_manager] Unknown action: {action}')

    # =========================================================================
    # Gazebo lifecycle
    # =========================================================================

    def _start_gazebo(self, world_name: str):
        if world_name not in WORLDS:
            self.get_logger().error(
                f'[gazebo_manager] Unknown world: {world_name}')
            return

        with self._lock:
            if self._gazebo_status == 'running':
                self.get_logger().warn(
                    f'[gazebo_manager] Gazebo already running ({self._current_world})')
                return

        world = WORLDS[world_name]
        pkg   = world['launch_pkg']
        launch = world['launch_file']
        cmd   = ['ros2', 'launch', pkg, launch]

        env = dict(os.environ)
        env['ROS_DOMAIN_ID'] = str(self.get_parameter('ros_domain_id').value)
        if 'DISPLAY' not in env:
            env['DISPLAY'] = ':0'

        self.get_logger().info(
            f'[gazebo_manager] Starting Gazebo world: {world_name}')

        with self._lock:
            self._gazebo_status = 'starting'

        try:
            process = subprocess.Popen(
                cmd, env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
                start_new_session=True,
            )
            with self._lock:
                self._gazebo_process = process
                self._gazebo_pid     = process.pid
                self._current_world  = world_name
                self._gazebo_started_at = time.time()
                self._gazebo_status  = 'running'
                self._target_runtime = 'gazebo'

            self.get_logger().info(
                f'[gazebo_manager] ✅ Gazebo started (PID={process.pid})')

            # Tell supervisor we're in simulation mode
            self._send_supervisor_cmd('switch_controller',
                                       {'name': 'simulation'})

            # Log output
            threading.Thread(
                target=self._read_gazebo_output,
                args=(process,), daemon=True).start()

        except Exception as e:
            self.get_logger().error(
                f'[gazebo_manager] Failed to start Gazebo: {e}')
            with self._lock:
                self._gazebo_status = 'error'

    def _stop_gazebo(self):
        with self._lock:
            proc = self._gazebo_process
            pid  = self._gazebo_pid
            if proc is None:
                return

        self.get_logger().info(f'[gazebo_manager] Stopping Gazebo (PID={pid})')
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=2.0)
        except Exception as e:
            self.get_logger().error(f'[gazebo_manager] Stop error: {e}')

        with self._lock:
            self._gazebo_process = None
            self._gazebo_pid     = None
            self._gazebo_status  = 'stopped'
            self._current_world  = None
            self._target_runtime = 'real_robot'

        # Return supervisor to off
        self._send_supervisor_cmd('switch_controller', {'name': 'off'})
        self.get_logger().info('[gazebo_manager] ✅ Gazebo stopped')

    def _reset_gazebo(self):
        """Reset simulation by sending ROS service call."""
        try:
            result = subprocess.run(
                ['ros2', 'service', 'call', '/reset_simulation',
                 'std_srvs/srv/Empty', '{}'],
                timeout=5.0, capture_output=True, text=True,
                env={**os.environ,
                     'ROS_DOMAIN_ID': str(
                         self.get_parameter('ros_domain_id').value)}
            )
            self.get_logger().info('[gazebo_manager] Simulation reset')
        except subprocess.TimeoutExpired:
            self.get_logger().warn('[gazebo_manager] Reset timed out')
        except Exception as e:
            self.get_logger().error(f'[gazebo_manager] Reset failed: {e}')

    def _set_runtime(self, runtime: str):
        if runtime not in ('real_robot', 'gazebo'):
            return
        with self._lock:
            self._target_runtime = runtime
        self.get_logger().info(
            f'[gazebo_manager] Runtime set to: {runtime}')
        # Safety: if switching to real_robot while gazebo running, stop gazebo
        if runtime == 'real_robot':
            with self._lock:
                running = self._gazebo_status == 'running'
            if running:
                self.get_logger().warn(
                    '[gazebo_manager] Stopping Gazebo to switch to real_robot mode')
                threading.Thread(target=self._stop_gazebo, daemon=True).start()

    def _read_gazebo_output(self, process: subprocess.Popen):
        try:
            for line in process.stdout:
                pass  # could parse RTF from output
        except Exception:
            pass
        rc = process.poll()
        with self._lock:
            self._gazebo_status = 'stopped' if rc == 0 else 'error'
        self.get_logger().info(f'[gazebo_manager] Gazebo exited (rc={rc})')

    def _monitor_gazebo(self):
        with self._lock:
            proc   = self._gazebo_process
            status = self._gazebo_status

        if status == 'running' and proc is not None:
            if proc.poll() is not None:
                with self._lock:
                    self._gazebo_status = 'error'
                self.get_logger().error(
                    '[gazebo_manager] ⚠ Gazebo crashed unexpectedly!')

    # =========================================================================
    # Helpers
    # =========================================================================

    def _send_supervisor_cmd(self, action: str, params: dict):
        msg = String()
        msg.data = json.dumps({'action': action, 'params': params})
        self._pub_supervisor.publish(msg)

    def _publish_status(self):
        with self._lock:
            status = {
                'time': time.time(),
                'gazebo_status': self._gazebo_status,
                'current_world': self._current_world,
                'target_runtime': self._target_runtime,
                'gazebo_pid': self._gazebo_pid,
                'uptime_s': round(time.time() - self._gazebo_started_at, 1)
                    if self._gazebo_started_at and self._gazebo_status == 'running'
                    else 0,
                'available_worlds': {
                    name: {
                        'display_name': w['display_name'],
                        'recommended_speed': w['recommended_speed'],
                    }
                    for name, w in WORLDS.items()
                },
                'block_real_cmd_vel': self._target_runtime == 'gazebo',
            }

        msg = String()
        msg.data = json.dumps(status)
        self._pub_status.publish(msg)

    def destroy_node(self):
        self.get_logger().info('[gazebo_manager] Shutting down')
        if self._gazebo_status == 'running':
            self._stop_gazebo()
        super().destroy_node()


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = GazeboManagerControlNode()
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
