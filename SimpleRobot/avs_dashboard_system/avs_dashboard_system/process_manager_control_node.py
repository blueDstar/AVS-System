"""
AVS Dashboard System — Process Manager Control Node
File: avs_dashboard_system/process_manager_control_node.py

Runs whitelisted system processes (micro-ROS agent, perception, RViz, Gazebo)
from a predefined YAML config. No arbitrary shell commands are allowed.

Topics:
  Subscribed: /avs/process_cmd     (JSON: {"action": "start", "name": "rviz_tools"})
  Published:  /avs/process_status  (JSON: process list with status, PID, CPU, RAM)
"""

import json
import os
import psutil
import shlex
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String

from avs_dashboard_system.utils import parse_json_string

RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE
)

import yaml

# Default whitelist - populated from yaml later
DEFAULT_PROCESSES = {}

class ManagedProcess:
    """Tracks one managed system process."""
    def __init__(self, name: str, config: dict):
        self.name       = name
        self.config     = config
        self.process: Optional[subprocess.Popen] = None
        self.pid: Optional[int] = None
        self.status: str = 'stopped'  # stopped | running | error
        self.started_at: Optional[float] = None
        self.log_lines: list = []
        self._psutil_proc: Optional[psutil.Process] = None

    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def get_resource_usage(self) -> dict:
        """Get CPU% and RAM MB for the process."""
        if not self.is_running() or self.pid is None:
            return {'cpu_percent': 0.0, 'ram_mb': 0.0}
        try:
            if self._psutil_proc is None or self._psutil_proc.pid != self.pid:
                self._psutil_proc = psutil.Process(self.pid)
            cpu = self._psutil_proc.cpu_percent(interval=None)
            ram = self._psutil_proc.memory_info().rss / 1e6
            return {'cpu_percent': round(cpu, 1), 'ram_mb': round(ram, 1)}
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return {'cpu_percent': 0.0, 'ram_mb': 0.0}


class ProcessManagerControlNode(Node):
    """
    Manages whitelisted system processes. Accepts JSON commands from
    /avs/process_cmd, publishes process status to /avs/process_status.
    """

    def __init__(self):
        super().__init__('process_manager_control')

        self.declare_parameter('topic_process_cmd', '/avs/process_cmd')
        self.declare_parameter('topic_process_status', '/avs/process_status')
        self.declare_parameter('status_hz', 2.0)
        self.declare_parameter('ros_domain_id', 20)

        self.declare_parameter('processes_config_path', '')
        
        config_path = self.get_parameter('processes_config_path').value
        processes_cfg = {}
        if config_path and os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    yaml_data = yaml.safe_load(f)
                    processes_cfg = yaml_data.get('processes', {})
            except Exception as e:
                self.get_logger().error(f"Failed to load processes config: {e}")
        
        self._lock = threading.Lock()
        self._processes: dict[str, ManagedProcess] = {
            name: ManagedProcess(name, cfg)
            for name, cfg in processes_cfg.items()
        }

        # Publishers / Subscribers
        r10 = RELIABLE_QOS
        self._pub_status = self.create_publisher(
            String, self.get_parameter('topic_process_status').value, r10)
        self.create_subscription(String,
            self.get_parameter('topic_process_cmd').value,
            self._cb_process_cmd, r10)

        status_period = 1.0 / self.get_parameter('status_hz').value
        self.create_timer(status_period, self._publish_status)
        self.create_timer(5.0, self._monitor_processes)

        self.get_logger().info(
            f'[process_manager_control] Ready | '
            f'whitelist={list(self._processes.keys())}')

    # =========================================================================
    # Command handler
    # =========================================================================

    def _cb_process_cmd(self, msg: String):
        data = parse_json_string(msg.data)
        if not data:
            return
        action = data.get('action', '')
        name   = data.get('name', '')

        if name and name not in self._processes:
            self.get_logger().warn(
                f'[process_manager] ⛔ Rejected: "{name}" not in whitelist')
            return

        if action == 'start':
            threading.Thread(
                target=self._start_process,
                args=(name,), daemon=True).start()
        elif action == 'stop':
            threading.Thread(
                target=self._stop_process,
                args=(name, True), daemon=True).start()
        elif action == 'restart':
            threading.Thread(
                target=self._restart_process,
                args=(name,), daemon=True).start()
        elif action == 'list':
            self._publish_status()
        else:
            self.get_logger().warn(
                f'[process_manager] Unknown action: {action}')

    # =========================================================================
    # Process lifecycle
    # =========================================================================

    def _start_process(self, name: str):
        proc = self._processes[name]
        if proc.is_running():
            self.get_logger().warn(
                f'[process_manager] {name} already running')
            return

        cfg = proc.config
        p_type = cfg.get('type', 'host_script')
        container = cfg.get('container', '')
        workspace = cfg.get('workspace', '')
        package = cfg.get('package', '')
        executable = cfg.get('executable', '')
        launch_file = cfg.get('launch_file', '')
        default_args = cfg.get('default_args', '')
        script = cfg.get('script', '')
        remaps = cfg.get('remaps', [])

        env = dict(os.environ)
        domain_id = str(self.get_parameter('ros_domain_id').value)
        env['ROS_DOMAIN_ID'] = domain_id
        if 'DISPLAY' not in env:
            env['DISPLAY'] = ':0'

        cmd_parts = []

        if container:
            # Build docker exec prefix
            cmd_parts = ['docker', 'exec', '-it', container, 'bash', '-lc']
            inner_cmd = []
            if workspace:
                inner_cmd.append(f"cd {workspace}")
            inner_cmd.append("source /opt/ros/humble/setup.bash")
            inner_cmd.append("if [ -f install/setup.bash ]; then source install/setup.bash; fi")
            inner_cmd.append(f"export ROS_DOMAIN_ID={domain_id}")
            
            # The actual execution
            if p_type == 'ros2_run':
                ros_cmd = f"ros2 run {package} {executable} {default_args}"
                # Append remaps
                if remaps:
                    ros_cmd += " --ros-args"
                    for rm in remaps:
                        ros_cmd += f" -r {rm['from']}:={rm['to']}"
                inner_cmd.append(ros_cmd)
            elif p_type == 'ros2_launch':
                ros_cmd = f"ros2 launch {package} {launch_file} {default_args}"
                # Launch doesn't support generic node remaps directly via CLI in the same way, 
                # but we append them if provided
                if remaps:
                    for rm in remaps:
                        ros_cmd += f" {rm['from']}:={rm['to']}"
                inner_cmd.append(ros_cmd)
            elif p_type == 'host_script':
                inner_cmd.append(script)
                
            # Combine inner cmd string
            combined_inner = " && ".join(inner_cmd)
            cmd_parts.append(combined_inner)
            cmd_str = " ".join(cmd_parts)
        else:
            # Local execution
            if p_type == 'ros2_run':
                cmd_parts = ['ros2', 'run', package, executable]
                if default_args:
                    cmd_parts.extend(shlex.split(default_args))
                if remaps:
                    cmd_parts.append('--ros-args')
                    for rm in remaps:
                        cmd_parts.extend(['-r', f"{rm['from']}:={rm['to']}"])
                cmd_str = " ".join(cmd_parts)
            elif p_type == 'ros2_launch':
                cmd_parts = ['ros2', 'launch', package, launch_file]
                if default_args:
                    cmd_parts.extend(shlex.split(default_args))
                if remaps:
                    for rm in remaps:
                        cmd_parts.append(f"{rm['from']}:={rm['to']}")
                cmd_str = " ".join(cmd_parts)
            else:
                # script
                cmd_str = os.path.expanduser(script)
                cmd_parts = shlex.split(cmd_str)

        try:

            self.get_logger().info(
                f'[process_manager] Starting: {name} | cmd: {cmd_str}')

            process = subprocess.Popen(
                cmd_parts,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,  # separate process group
            )

            with self._lock:
                proc.process    = process
                proc.pid        = process.pid
                proc.started_at = time.time()
                proc.status     = 'running'
                proc.log_lines  = []

            # Background log reader
            threading.Thread(
                target=self._read_output,
                args=(proc,), daemon=True).start()

            self.get_logger().info(
                f'[process_manager] ✅ {name} started (PID={process.pid})')

        except FileNotFoundError as e:
            self.get_logger().error(
                f'[process_manager] Command not found for {name}: {e}')
            proc.status = 'error'
        except Exception as e:
            self.get_logger().error(
                f'[process_manager] Failed to start {name}: {e}')
            proc.status = 'error'

    def _stop_process(self, name: str, graceful: bool = True):
        proc = self._processes[name]
        if not proc.is_running():
            return
        try:
            self.get_logger().info(
                f'[process_manager] Stopping {name} (PID={proc.pid})')
            if graceful:
                os.killpg(os.getpgid(proc.process.pid), signal.SIGINT)
                proc.process.wait(timeout=5.0)
            else:
                os.killpg(os.getpgid(proc.process.pid), signal.SIGKILL)
                proc.process.wait(timeout=2.0)
            with self._lock:
                proc.status = 'stopped'
            self.get_logger().info(f'[process_manager] ✅ {name} stopped')
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.process.pid), signal.SIGKILL)
                proc.process.wait(timeout=1.0)
            except Exception:
                pass
            with self._lock:
                proc.status = 'stopped'
        except Exception as e:
            self.get_logger().error(
                f'[process_manager] Error stopping {name}: {e}')

    def _restart_process(self, name: str):
        self._stop_process(name, graceful=True)
        time.sleep(1.0)
        self._start_process(name)

    def _read_output(self, proc: ManagedProcess):
        """Read and buffer process output (background thread)."""
        try:
            for line in proc.process.stdout:
                line = line.rstrip()
                with self._lock:
                    proc.log_lines.append(line)
                    if len(proc.log_lines) > 300:
                        proc.log_lines.pop(0)
        except Exception:
            pass
        rc = proc.process.poll()
        with self._lock:
            proc.status = 'stopped' if rc == 0 else 'error'
        self.get_logger().info(
            f'[process_manager] {proc.name} exited (rc={rc})')

    def _monitor_processes(self):
        """Detect unexpected process crashes."""
        for name, proc in self._processes.items():
            with self._lock:
                was_running = proc.status == 'running'
                still_running = proc.is_running()
            if was_running and not still_running:
                with self._lock:
                    proc.status = 'error'
                self.get_logger().warn(
                    f'[process_manager] ⚠ {name} unexpectedly stopped')

    # =========================================================================
    # Status publisher
    # =========================================================================

    def _publish_status(self):
        status_list = []
        with self._lock:
            for name, proc in self._processes.items():
                resources = proc.get_resource_usage()
                status_list.append({
                    'name': name,
                    'description': proc.config.get('label', proc.config.get('description', '')),
                    'group': proc.config.get('group', ''),
                    'status': proc.status,
                    'pid': proc.pid,
                    'running': proc.is_running(),
                    'uptime_s': round(time.time() - proc.started_at, 1)
                        if proc.started_at and proc.is_running() else 0,
                    'cpu_percent': resources['cpu_percent'],
                    'ram_mb': resources['ram_mb'],
                    'recent_log': proc.log_lines[-5:],  # last 5 lines
                })

        msg = String()
        msg.data = json.dumps({
            'time': time.time(),
            'processes': status_list,
        })
        self._pub_status.publish(msg)

    def destroy_node(self):
        self.get_logger().info(
            '[process_manager] Shutting down — stopping all processes')
        for name in list(self._processes.keys()):
            if self._processes[name].is_running():
                self._stop_process(name, graceful=True)
        super().destroy_node()


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = ProcessManagerControlNode()
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
