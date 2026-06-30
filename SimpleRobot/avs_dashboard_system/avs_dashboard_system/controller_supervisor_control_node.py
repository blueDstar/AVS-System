"""
AVS Dashboard System — Controller Supervisor Control Node
File: avs_dashboard_system/controller_supervisor_control_node.py

Manages lifecycle of controller processes:
  - Start/stop/restart controller nodes via subprocess
  - Enforce only one active controller at a time
  - Publish /avs/selected_controller when controller is switched
  - Emergency stop support
  - Config loaded from controllers.yaml

Services provided:
  /avs/switch_controller   (std_srvs/srv/SetBool → repurposed via String topic)
  → Uses ROS topics for simplicity (dashboard sends JSON commands via WS)

Topics published:
  /avs/selected_controller   std_msgs/String
  /avs/emergency_stop        std_msgs/Bool
  /avs/controller_list       std_msgs/String (JSON list)

Topics subscribed:
  /avs/supervisor_cmd        std_msgs/String (JSON commands from dashboard API)
"""

import json
import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String, Bool

from avs_dashboard_system.utils import parse_json_string, utc_now_str

RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE
)

# Controllers that are managed internally (no subprocess)
INTERNAL_CONTROLLERS = {'off', 'manual', 'simulation'}


# Controllers that are managed internally (no subprocess)
INTERNAL_CONTROLLERS = {'off', 'manual', 'simulation'}

    def get_returncode(self):
        if self.process is None:
            return None
        return self.process.poll()


class ControllerSupervisorControlNode(Node):
    """
    Manages controller processes. Receives JSON commands from
    /avs/supervisor_cmd and publishes selected controller to
    /avs/selected_controller.
    """

    def __init__(self):
        super().__init__('controller_supervisor_control')

        # ---- Parameters ----
        self.declare_parameter('controllers_config_path', '')
        self.declare_parameter('topic_selected_ctrl', '/avs/selected_controller')
        self.declare_parameter('topic_emergency_stop', '/avs/emergency_stop')
        self.declare_parameter('topic_supervisor_cmd', '/avs/supervisor_cmd')
        self.declare_parameter('topic_controller_list', '/avs/controller_list')
        self.declare_parameter('ros_domain_id', 20)

        # ---- State ----
        self._active_controller: str = 'off'
        self._emergency_stop: bool = False
        self._controllers_config: dict = {}

        # Load controller config (inline defaults + external YAML)
        self._load_controller_config()

        # ---- Publishers ----
        r10 = RELIABLE_QOS
        self._pub_selected = self.create_publisher(
            String, self.get_parameter('topic_selected_ctrl').value, r10)
        self._pub_estop = self.create_publisher(
            Bool, self.get_parameter('topic_emergency_stop').value, r10)
        self._pub_ctrl_list = self.create_publisher(
            String, self.get_parameter('topic_controller_list').value, r10)

        # ---- Subscribers ----
        self.create_subscription(String,
            self.get_parameter('topic_supervisor_cmd').value,
            self._cb_supervisor_cmd, r10)

        # ---- Timers ----
        self.create_timer(2.0, self._publish_controller_list)

        # Publish initial state
        self._publish_selected_controller()
        self._publish_controller_list()

        self.get_logger().info(
            f'[controller_supervisor_control] Started | '
            f'active={self._active_controller} | '
            f'controllers={list(self._controllers_config.keys())}'
        )

    def _load_controller_config(self):
        """Load controllers from config dict (populated by launch file params)."""
        # Default configuration — mirrors controllers.yaml
        self._controllers_config = {
            'off': {
                'type': 'internal',
                'description': 'All controllers disabled',
                'cmd_topic': '',
            },
            'manual': {
                'type': 'internal',
                'description': 'Manual control via dashboard',
                'cmd_topic': '/avs/cmd_vel/manual',
            },
            'main_pd': {
                'type': 'node',
                'package': 'avs_controlsystem',
                'executable': 'main_following_pd',
                'cmd_topic': '/avs/cmd_vel/main_pd',
                'direct_cmd_vel': False,
                'description': 'Main PD lane following',
                'remap_cmd_vel': True,
            },
            'pd_lidar': {
                'type': 'node',
                'package': 'avs_controlsystem',
                'executable': 'lane_lidar_follower_node',
                'cmd_topic': '/avs/cmd_vel/pd_lidar',
                'direct_cmd_vel': False,
                'description': 'PD + LiDAR safety',
                'remap_cmd_vel': True,
            },
            'backstepping_pd': {
                'type': 'node',
                'package': 'avs_controlsystem',
                'executable': 'backstepping_pd_node',
                'cmd_topic': '/avs/cmd_vel/backstepping_pd',
                'direct_cmd_vel': False,
                'description': 'Backstepping-PD controller',
                'remap_cmd_vel': True,
            },
            'cascade_pd': {
                'type': 'launch',
                'package': 'avs_cascadecontrol',
                'launch': 'cascade_pd_control.launch.py',
                'cmd_topic': '/avs/cmd_vel/cascade_pd',
                'direct_cmd_vel': False,
                'description': 'Cascade outer+inner PD',
                'remap_cmd_vel': True,
            },
            'simulation': {
                'type': 'internal',
                'description': 'Simulation mode (Gazebo)',
                'cmd_topic': '/avs/cmd_vel/sim',
                'cmd_topic': '/avs/cmd_vel/sim',
            },
        }

    # =========================================================================
    # Command handler
    # =========================================================================

    def _cb_supervisor_cmd(self, msg: String):
        """Handle JSON commands from dashboard API."""
        data = parse_json_string(msg.data)
        if not data:
            return
        action = data.get('action', '')
        params = data.get('params', {})

        handlers = {
            'switch_controller': self._cmd_switch_controller,
            'stop_controller':   self._cmd_stop_controller,
            'start_controller':  self._cmd_start_controller,
            'emergency_stop':    self._cmd_emergency_stop,
            'emergency_reset':   self._cmd_emergency_reset,
            'list_controllers':  self._cmd_list_controllers,
        }
        handler = handlers.get(action)
        if handler:
            try:
                handler(params)
            except Exception as e:
                self.get_logger().error(
                    f'[supervisor] Action {action} failed: {e}')
        else:
            self.get_logger().warn(
                f'[supervisor] Unknown action: {action}')

    def _cmd_switch_controller(self, params: dict):
        new_ctrl = params.get('name', 'off')
        if new_ctrl not in self._controllers_config:
            self.get_logger().warn(
                f'[supervisor] Unknown controller: {new_ctrl}')
            return
        if self._emergency_stop:
            self.get_logger().warn(
                '[supervisor] Cannot switch controller: emergency stop active')
            return

        with self._lock:
            old_ctrl = self._active_controller

        if old_ctrl == new_ctrl:
            return

        with self._lock:
            self._active_controller = new_ctrl

        self.get_logger().info(
            f'[supervisor] ✅ Switched: {old_ctrl} → {new_ctrl}')
        self._publish_selected_controller()

    def _cmd_stop_controller(self, params: dict):
        ctrl = params.get('name', self._active_controller)
        if ctrl not in INTERNAL_CONTROLLERS:
            self._stop_process(ctrl, graceful=True)
        with self._lock:
            if self._active_controller == ctrl:
                self._active_controller = 'off'
        self._publish_selected_controller()

    def _cmd_start_controller(self, params: dict):
        ctrl = params.get('name', '')
        if ctrl in INTERNAL_CONTROLLERS:
            return
        if ctrl not in self._processes:
            self.get_logger().warn(f'[supervisor] No process config for: {ctrl}')
            return
        self._start_process(ctrl)

    def _cmd_emergency_stop(self, params: dict):
        with self._lock:
            self._emergency_stop = True
        # Stop all running controllers
        for name in list(self._processes.keys()):
            if self._processes[name].is_running():
                self._stop_process(name, graceful=False)
        msg = Bool()
        msg.data = True
        self._pub_estop.publish(msg)
        self.get_logger().warn('[supervisor] ⚠ EMERGENCY STOP published')

    def _cmd_emergency_reset(self, params: dict):
        with self._lock:
            self._emergency_stop = False
        msg = Bool()
        msg.data = False
        self._pub_estop.publish(msg)
        self.get_logger().info('[supervisor] Emergency stop CLEARED')

    def _cmd_list_controllers(self, params: dict):
        self._publish_controller_list()

    # =========================================================================
    # Process management
    # =========================================================================

    def _build_cmd(self, name: str) -> list[str]:
        """Build the subprocess command for a controller."""
        cfg  = self._controllers_config[name]
        env  = dict(os.environ)
        env['ROS_DOMAIN_ID'] = str(self.get_parameter('ros_domain_id').value)

        ctrl_type = cfg.get('type', 'node')
        cmd_topic  = cfg.get('cmd_topic', '')
        remap      = cfg.get('remap_cmd_vel', False)

        if ctrl_type == 'node':
            pkg  = cfg['package']
            exec = cfg['executable']
            cmd  = ['ros2', 'run', pkg, exec, '--ros-args']
            if remap and cmd_topic:
                cmd += ['--remap', f'/cmd_vel:={cmd_topic}']
        elif ctrl_type == 'launch':
            pkg    = cfg['package']
            launch = cfg['launch']
            cmd    = ['ros2', 'launch', pkg, launch]
            extra_params = cfg.get('params', [])
            for p in extra_params:
                cmd.append(f"{p['name']}:={p['value']}")
        else:
            raise ValueError(f'Unknown controller type: {ctrl_type}')

        return cmd

    def _start_process(self, name: str):
        proc = self._processes.get(name)
        if proc is None:
            return
        if proc.is_running():
            self.get_logger().warn(
                f'[supervisor] {name} already running (PID={proc.pid})')
            return

        try:
            cmd = self._build_cmd(name)
            env = dict(os.environ)
            env['ROS_DOMAIN_ID'] = str(
                self.get_parameter('ros_domain_id').value)

            self.get_logger().info(
                f'[supervisor] Starting {name}: {" ".join(cmd)}')
            process = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            proc.process    = process
            proc.pid        = process.pid
            proc.started_at = time.time()
            proc.status     = 'running'

            # Start log reader thread
            log_thread = threading.Thread(
                target=self._read_process_output,
                args=(proc,),
                daemon=True
            )
            log_thread.start()

            self.get_logger().info(
                f'[supervisor] ✅ {name} started (PID={process.pid})')
        except Exception as e:
            self.get_logger().error(
                f'[supervisor] Failed to start {name}: {e}')
            proc.status = 'error'

    def _stop_process(self, name: str, graceful: bool = True):
        proc = self._processes.get(name)
        if proc is None or not proc.is_running():
            return
        try:
            self.get_logger().info(
                f'[supervisor] Stopping {name} (PID={proc.pid})')
            if graceful:
                proc.process.send_signal(signal.SIGINT)
                proc.process.wait(timeout=3.0)
            else:
                proc.process.kill()
                proc.process.wait(timeout=1.0)
            proc.status = 'stopped'
            self.get_logger().info(f'[supervisor] ✅ {name} stopped')
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                f'[supervisor] {name} did not stop — killing')
            proc.process.kill()
            proc.status = 'stopped'
        except Exception as e:
            self.get_logger().error(
                f'[supervisor] Error stopping {name}: {e}')

    def _read_process_output(self, proc: ControllerProcess):
        """Read stdout/stderr from controller process (background thread)."""
        try:
            for line in proc.process.stdout:
                line = line.rstrip()
                proc.log_lines.append(line)
                if len(proc.log_lines) > 200:
                    proc.log_lines.pop(0)
        except Exception:
            pass
        rc = proc.process.poll()
        proc.status = 'stopped' if rc == 0 else 'error'
        self.get_logger().info(
            f'[supervisor] {proc.name} exited (rc={rc})')

    def _monitor_processes(self):
        """Check health of managed processes."""
        for name, proc in self._processes.items():
            if not proc.is_running() and proc.status == 'running':
                proc.status = 'error'
                self.get_logger().warn(
                    f'[supervisor] ⚠ {name} unexpectedly stopped')
                with self._lock:
                    if self._active_controller == name:
                        self.get_logger().error(
                            f'[supervisor] Active controller {name} crashed! '
                            f'Switching to OFF for safety.')
                        self._active_controller = 'off'
                        self._publish_selected_controller()

    # =========================================================================
    # Publishers
    # =========================================================================

    def _publish_selected_controller(self):
        msg = String()
        with self._lock:
            msg.data = self._active_controller
        self._pub_selected.publish(msg)

    def _publish_controller_list(self):
        with self._lock:
            active = self._active_controller
            e_stop = self._emergency_stop

        ctrl_list = []
        for name, cfg in self._controllers_config.items():
            proc = self._processes.get(name)
            ctrl_list.append({
                'name': name,
                'description': cfg.get('description', ''),
                'type': cfg.get('type', 'internal'),
                'cmd_topic': cfg.get('cmd_topic', ''),
                'active': (name == active),
                'status': proc.status if proc else (
                    'active' if name == active else 'internal'),
                'pid': proc.pid if proc else None,
                'running': proc.is_running() if proc else False,
            })

        state = {
            'time': time.time(),
            'active_controller': active,
            'emergency_stop': e_stop,
            'controllers': ctrl_list,
        }
        msg = String()
        msg.data = json.dumps(state)
        self._pub_ctrl_list.publish(msg)

    def destroy_node(self):
        """Safe shutdown: stop all running controllers."""
        self.get_logger().info(
            '[supervisor] Shutting down — stopping all controllers')
        for name in list(self._processes.keys()):
            self._stop_process(name, graceful=True)
        super().destroy_node()


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = ControllerSupervisorControlNode()
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
