"""
AVS Dashboard System — Experiment Recorder Control Node
File: avs_dashboard_system/experiment_recorder_control_node.py

Records time-series data from all sensor/control topics to:
  ~/avs_experiments/YYYYMMDD_HHMMSS_<controller>_<scenario>/
    ├── data.csv         (structured time-series, ~10Hz)
    ├── raw_messages.jsonl (complete JSON dump)
    ├── metadata.json    (experiment metadata)
    ├── summary.json     (quick metrics on stop)
    └── plots/           (auto-generated matplotlib plots)

Controlled via topic /avs/experiment/cmd (JSON):
  {"action": "start", "metadata": {...}}
  {"action": "stop"}
  {"action": "mark_event", "label": "turn_start"}

Also publishes /avs/experiment_status (JSON status).
"""

import csv
import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

from avs_dashboard_system.utils import (
    safe_quat_to_euler, parse_json_string, safe_get,
    compute_rmse, compute_mae, compute_smoothness, utc_now_str
)

RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE
)

# CSV columns definition
CSV_COLUMNS = [
    'time_s', 'controller_name', 'scenario_name', 'mode',
    'cmd_v', 'cmd_omega',
    'odom_x', 'odom_y', 'odom_yaw', 'odom_v', 'odom_omega',
    'imu_yaw', 'imu_wz',
    'front_min_m',
    'epsilon_x_mm', 'epsilon_y_mm', 'e_lat_m', 'theta_rad',
    'lane_valid', 'lane_state',
    'v_ref', 'omega_ref',
    'v_left_ref', 'v_right_ref', 'v_left_meas', 'v_right_meas',
    'v_cmd_raw', 'omega_cmd_raw',
    'odom_timeout', 'ref_timeout', 'lidar_stop',
    'detection_count',
    'main_lane_conf', 'other_lane_conf', 'turn_lane_conf',
    'stop_line_conf', 'parking_zone_conf', 'vehicle_conf',
    'solid_white_conf', 'solid_yellow_conf', 'dashed_white_conf',
    'dashed_yellow_conf'
]


class ExperimentRecorderControlNode(Node):
    """Records experiment data to CSV/JSONL with auto-plotting on stop."""

    def __init__(self):
        super().__init__('experiment_recorder_control')

        # Parameters
        self.declare_parameter('base_dir', '~/avs_experiments')
        self.declare_parameter('csv_flush_interval_s', 1.0)
        self.declare_parameter('auto_plot', True)
        self.declare_parameter('record_hz', 10.0)
        # Topics
        self.declare_parameter('topic_cmd_vel', '/cmd_vel')
        self.declare_parameter('topic_odom_raw', '/odom_raw')
        self.declare_parameter('topic_imu', '/imu')
        self.declare_parameter('topic_scan', '/scan')
        self.declare_parameter('topic_control_error', '/avs/control_error')
        self.declare_parameter('topic_dashboard_state', '/avs/dashboard_state')
        self.declare_parameter('topic_experiment_cmd', '/avs/experiment/cmd')
        self.declare_parameter('topic_experiment_status', '/avs/experiment_status')

        self._base_dir = Path(
            self.get_parameter('base_dir').value).expanduser()
        self._base_dir.mkdir(parents=True, exist_ok=True)
        self._auto_plot = self.get_parameter('auto_plot').value

        # ---- State ----
        self._lock = threading.Lock()
        self._recording: bool = False
        self._exp_dir: Optional[Path] = None
        self._csv_file = None
        self._csv_writer = None
        self._jsonl_file = None
        self._exp_metadata: dict = {}
        self._start_time: float = 0.0
        self._events: list = []
        self._row_count: int = 0
        self._last_flush_time: float = 0.0

        # Latest sensor values (thread-safe)
        self._latest: dict = {
            'cmd_v': 0.0, 'cmd_omega': 0.0,
            'odom_x': 0.0, 'odom_y': 0.0, 'odom_yaw': 0.0,
            'odom_v': 0.0, 'odom_omega': 0.0, 'odom_timeout': False,
            'imu_yaw': 0.0, 'imu_wz': 0.0,
            'front_min_m': 9.9,
            'epsilon_x_mm': 0.0, 'epsilon_y_mm': 0.0,
            'e_lat_m': 0.0, 'theta_rad': 0.0,
            'lane_valid': False, 'lane_state': 'UNKNOWN',
            'v_ref': 0.0, 'omega_ref': 0.0,
            'v_left_ref': 0.0, 'v_right_ref': 0.0,
            'v_left_meas': 0.0, 'v_right_meas': 0.0,
            'v_cmd_raw': 0.0, 'omega_cmd_raw': 0.0,
            'ref_timeout': False, 'lidar_stop': False,
            'detection_count': 0,
            'main_lane_conf': 0.0, 'other_lane_conf': 0.0, 'turn_lane_conf': 0.0,
            'stop_line_conf': 0.0, 'parking_zone_conf': 0.0, 'vehicle_conf': 0.0,
            'solid_white_conf': 0.0, 'solid_yellow_conf': 0.0, 'dashed_white_conf': 0.0,
            'dashed_yellow_conf': 0.0,
        }

        # ---- Publishers ----
        self._pub_status = self.create_publisher(
            String,
            self.get_parameter('topic_experiment_status').value,
            RELIABLE_QOS
        )

        # ---- Subscribers ----
        sd  = qos_profile_sensor_data
        r10 = QoSProfile(depth=10)
        self.create_subscription(Twist,
            self.get_parameter('topic_cmd_vel').value,
            self._cb_cmd_vel, sd)
        self.create_subscription(Odometry,
            self.get_parameter('topic_odom_raw').value,
            self._cb_odom, sd)
        self.create_subscription(Imu,
            self.get_parameter('topic_imu').value,
            self._cb_imu, sd)
        self.create_subscription(LaserScan,
            self.get_parameter('topic_scan').value,
            self._cb_scan, sd)
        self.create_subscription(String,
            self.get_parameter('topic_control_error').value,
            self._cb_control_error, r10)
        self.create_subscription(String,
            self.get_parameter('topic_dashboard_state').value,
            self._cb_dashboard_state, r10)
        self.create_subscription(String,
            self.get_parameter('topic_experiment_cmd').value,
            self._cb_experiment_cmd, r10)

        # Record timer
        period = 1.0 / self.get_parameter('record_hz').value
        self.create_timer(period, self._record_row)
        self.create_timer(2.0, self._publish_status)

        self.get_logger().info(
            f'[experiment_recorder_control] Ready | base_dir={self._base_dir}')

    # =========================================================================
    # Sensor callbacks
    # =========================================================================

    def _cb_cmd_vel(self, msg: Twist):
        with self._lock:
            self._latest['cmd_v']     = msg.linear.x
            self._latest['cmd_omega'] = msg.angular.z

    def _cb_odom(self, msg: Odometry):
        with self._lock:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            _, _, yaw = safe_quat_to_euler(q.x, q.y, q.z, q.w)
            self._latest.update({
                'odom_x': p.x, 'odom_y': p.y, 'odom_yaw': yaw,
                'odom_v': msg.twist.twist.linear.x,
                'odom_omega': msg.twist.twist.angular.z,
                'odom_timeout': False,
            })

    def _cb_imu(self, msg: Imu):
        with self._lock:
            q = msg.orientation
            _, _, yaw = safe_quat_to_euler(q.x, q.y, q.z, q.w)
            self._latest['imu_yaw'] = yaw
            self._latest['imu_wz']  = msg.angular_velocity.z

    def _cb_scan(self, msg: LaserScan):
        with self._lock:
            ranges = msg.ranges
            if not ranges:
                return
            front_min = msg.range_max
            angle_min = msg.angle_min
            angle_inc = msg.angle_increment
            half = math.radians(30.0)
            for i, r in enumerate(ranges):
                if not math.isfinite(r):
                    continue
                angle = angle_min + i * angle_inc
                if abs(angle) <= half:
                    front_min = min(front_min, r)
            self._latest['front_min_m'] = front_min

    def _cb_control_error(self, msg: String):
        data = parse_json_string(msg.data)
        if not data:
            return
        with self._lock:
            self._latest.update({
                'epsilon_x_mm': float(safe_get(data, 'epsilon_x_mm', default=0.0)),
                'epsilon_y_mm': float(safe_get(data, 'epsilon_y_mm', default=0.0)),
                'e_lat_m': float(safe_get(data, 'e_lat_m', default=0.0)),
                'theta_rad': float(safe_get(data, 'theta_rad', default=0.0)),
                'lane_valid': bool(safe_get(data, 'valid', default=False)),
                'lane_state': str(safe_get(data, 'state', default='UNKNOWN')),
                'ref_timeout': bool(safe_get(data, 'ref_timeout', default=False)),
            })

    def _cb_dashboard_state(self, msg: String):
        """Pull controller debug fields from aggregated dashboard state."""
        data = parse_json_string(msg.data)
        if not data:
            return
        dbg = safe_get(data, 'controller_debug', default={})
        with self._lock:
            self._latest.update({
                'v_ref': float(dbg.get('v_ref', self._latest['v_ref'])),
                'omega_ref': float(dbg.get('omega_ref', self._latest['omega_ref'])),
                'v_left_ref': float(dbg.get('v_left_ref', self._latest['v_left_ref'])),
                'v_right_ref': float(dbg.get('v_right_ref', self._latest['v_right_ref'])),
                'v_left_meas': float(dbg.get('v_left_meas', self._latest['v_left_meas'])),
                'v_right_meas': float(dbg.get('v_right_meas', self._latest['v_right_meas'])),
                'v_cmd_raw': float(dbg.get('v_cmd_raw', self._latest['v_cmd_raw'])),
                'omega_cmd_raw': float(dbg.get('omega_cmd_raw', self._latest['omega_cmd_raw'])),
                'lidar_stop': bool(dbg.get('lidar_stop', self._latest['lidar_stop'])),
            })
            
        lane = safe_get(data, 'lane', default={})
        if lane:
            with self._lock:
                self._latest['detection_count'] = len(lane.get('classes', []))
                classes_detected = lane.get('classes_detected', [])
                
                # Reset confidences for this tick
                conf_fields = ['main_lane_conf', 'other_lane_conf', 'turn_lane_conf', 'stop_line_conf', 
                               'parking_zone_conf', 'vehicle_conf', 'solid_white_conf', 'solid_yellow_conf',
                               'dashed_white_conf', 'dashed_yellow_conf']
                for f in conf_fields:
                    self._latest[f] = 0.0
                    
                for c in classes_detected:
                    c_name = c.get('class_name', '')
                    conf = c.get('max_confidence', 0.0)
                    if c_name == 'main-lane': self._latest['main_lane_conf'] = conf
                    elif c_name == 'other-lane': self._latest['other_lane_conf'] = conf
                    elif c_name == 'turn-lane': self._latest['turn_lane_conf'] = conf
                    elif c_name == 'stop-line': self._latest['stop_line_conf'] = conf
                    elif c_name == 'parking-zone': self._latest['parking_zone_conf'] = conf
                    elif c_name == 'vehicle': self._latest['vehicle_conf'] = conf
                    elif c_name == 'solid-white': self._latest['solid_white_conf'] = conf
                    elif c_name == 'solid-yellow': self._latest['solid_yellow_conf'] = conf
                    elif c_name == 'dashed-white': self._latest['dashed_white_conf'] = conf
                    elif c_name == 'dashed-yellow': self._latest['dashed_yellow_conf'] = conf

    # =========================================================================
    # Experiment control command handler
    # =========================================================================

    def _cb_experiment_cmd(self, msg: String):
        data = parse_json_string(msg.data)
        if not data:
            return
        action = data.get('action', '')
        if action == 'start':
            self._start_experiment(data.get('metadata', {}))
        elif action == 'stop':
            self._stop_experiment()
        elif action == 'mark_event':
            self._mark_event(data.get('label', 'event'))
        else:
            self.get_logger().warn(
                f'[recorder] Unknown experiment action: {action}')

    def _start_experiment(self, metadata: dict):
        with self._lock:
            if self._recording:
                self.get_logger().warn('[recorder] Already recording')
                return

        ctrl     = metadata.get('controller_name', 'unknown')
        scenario = metadata.get('scenario_name', 'unnamed')
        mode     = metadata.get('mode', 'real')
        ts       = utc_now_str()

        dir_name = f'{ts}_{ctrl}_{scenario}'
        exp_dir  = self._base_dir / dir_name
        exp_dir.mkdir(parents=True, exist_ok=True)
        plots_dir = exp_dir / 'plots'
        plots_dir.mkdir(exist_ok=True)

        with self._lock:
            self._exp_dir  = exp_dir
            self._exp_metadata = {
                'dir': str(exp_dir),
                'timestamp': ts,
                'controller_name': ctrl,
                'scenario_name': scenario,
                'mode': mode,
                'notes': metadata.get('notes', ''),
                'track': metadata.get('track', ''),
                'lighting': metadata.get('lighting', ''),
                'speed_setting': metadata.get('speed_setting', ''),
            }
            self._start_time  = time.time()
            self._events      = []
            self._row_count   = 0
            self._recording   = True

            # Open files
            csv_path   = exp_dir / 'data.csv'
            jsonl_path = exp_dir / 'raw_messages.jsonl'
            self._csv_file   = open(csv_path, 'w', newline='')
            self._csv_writer = csv.DictWriter(
                self._csv_file, fieldnames=CSV_COLUMNS)
            self._csv_writer.writeheader()
            self._jsonl_file = open(jsonl_path, 'w')

        # Write metadata
        with open(exp_dir / 'metadata.json', 'w') as f:
            json.dump(self._exp_metadata, f, indent=2)

        self.get_logger().info(
            f'[recorder] ▶ Experiment started: {dir_name}')
        self._publish_status()

    def _stop_experiment(self):
        with self._lock:
            if not self._recording:
                return
            self._recording = False
            exp_dir = self._exp_dir
            metadata = dict(self._exp_metadata)

        if self._csv_file:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file = None
        if self._jsonl_file:
            self._jsonl_file.close()
            self._jsonl_file = None

        self.get_logger().info(
            f'[recorder] ⏹ Experiment stopped | rows={self._row_count}')

        # Generate summary and plots in background thread
        if self._auto_plot and exp_dir:
            t = threading.Thread(
                target=self._generate_analysis,
                args=(exp_dir, metadata),
                daemon=True
            )
            t.start()

        self._publish_status()

    def _mark_event(self, label: str):
        with self._lock:
            if not self._recording:
                return
            t = time.time() - self._start_time
            self._events.append({'time_s': round(t, 3), 'label': label})
        self.get_logger().info(f'[recorder] Event marked: {label}')

    # =========================================================================
    # Record one CSV row (called by timer)
    # =========================================================================

    def _record_row(self):
        with self._lock:
            if not self._recording:
                return
            elapsed = round(time.time() - self._start_time, 3)
            meta = self._exp_metadata
            row = {col: 0 for col in CSV_COLUMNS}
            row['time_s']          = elapsed
            row['controller_name'] = meta.get('controller_name', '')
            row['scenario_name']   = meta.get('scenario_name', '')
            row['mode']            = meta.get('mode', '')
            row.update(self._latest)
            row['odom_timeout'] = int(self._latest.get('odom_timeout', False))
            row['ref_timeout']  = int(self._latest.get('ref_timeout', False))
            row['lidar_stop']   = int(self._latest.get('lidar_stop', False))
            row['lane_valid']   = int(self._latest.get('lane_valid', False))

        if self._csv_writer:
            self._csv_writer.writerow(row)
            self._row_count += 1
        if self._jsonl_file:
            self._jsonl_file.write(json.dumps(row) + '\n')

        # Flush periodically
        now = time.monotonic()
        flush_interval = self.get_parameter('csv_flush_interval_s').value
        if now - self._last_flush_time > flush_interval:
            if self._csv_file:
                self._csv_file.flush()
            if self._jsonl_file:
                self._jsonl_file.flush()
            self._last_flush_time = now

    # =========================================================================
    # Post-experiment analysis (background)
    # =========================================================================

    def _generate_analysis(self, exp_dir: Path, metadata: dict):
        """Generate summary JSON and matplotlib plots."""
        try:
            import pandas as pd
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            csv_path = exp_dir / 'data.csv'
            if not csv_path.exists():
                return

            df = pd.read_csv(csv_path)
            if df.empty:
                return

            duration = float(df['time_s'].max())

            # Compute metrics
            lat_err   = df['epsilon_x_mm'].abs()
            head_err  = df['theta_rad'].abs()
            summary = {
                'metadata': metadata,
                'duration_s': round(duration, 2),
                'row_count': len(df),
                'mean_abs_lateral_error_mm': round(float(lat_err.mean()), 3),
                'rmse_lateral_error_mm': round(compute_rmse(df['epsilon_x_mm'].tolist()), 3),
                'max_abs_lateral_error_mm': round(float(lat_err.max()), 3),
                'mean_abs_heading_error_rad': round(float(head_err.mean()), 5),
                'rmse_heading_error_rad': round(compute_rmse(df['theta_rad'].tolist()), 5),
                'mean_cmd_v': round(float(df['cmd_v'].mean()), 4),
                'mean_abs_cmd_omega': round(float(df['cmd_omega'].abs().mean()), 5),
                'max_abs_cmd_omega': round(float(df['cmd_omega'].abs().max()), 5),
                'cmd_omega_smoothness': round(
                    compute_smoothness(df['cmd_omega'].tolist()), 6),
                'cmd_v_smoothness': round(
                    compute_smoothness(df['cmd_v'].tolist()), 6),
                'path_length_m': round(
                    float(((df['odom_x'].diff()**2 + df['odom_y'].diff()**2)**0.5).sum()), 3),
                'average_front_distance_m': round(
                    float(df['front_min_m'].mean()), 3),
                'lane_timeout_count': int((df['ref_timeout'] == 1).sum()),
                'odom_timeout_count': int((df['odom_timeout'] == 1).sum()),
                'lidar_stop_count': int((df['lidar_stop'] == 1).sum()),
                'tracking_valid_ratio': round(
                    float(df['lane_valid'].mean() * 100.0), 1),
                'mean_detection_count': round(float(df['detection_count'].mean()), 2),
                'perception_summary': {
                    'main_lane_seen_ratio': round(float((df['main_lane_conf'] > 0).mean()), 2) if 'main_lane_conf' in df else 0,
                    'stop_line_detect_count': int((df['stop_line_conf'] > 0).sum()) if 'stop_line_conf' in df else 0,
                    'parking_zone_detect_count': int((df['parking_zone_conf'] > 0).sum()) if 'parking_zone_conf' in df else 0,
                    'turn_lane_detect_count': int((df['turn_lane_conf'] > 0).sum()) if 'turn_lane_conf' in df else 0,
                    'vehicle_detect_count': int((df['vehicle_conf'] > 0).sum()) if 'vehicle_conf' in df else 0,
                    'lane_lost_count': int((df['lane_valid'] == 0).sum()),
                    'average_detection_confidence': {
                        'main-lane': round(float(df['main_lane_conf'][df['main_lane_conf'] > 0].mean() if len(df['main_lane_conf'][df['main_lane_conf'] > 0]) > 0 else 0), 2) if 'main_lane_conf' in df else 0,
                        'solid-yellow': round(float(df['solid_yellow_conf'][df['solid_yellow_conf'] > 0].mean() if len(df['solid_yellow_conf'][df['solid_yellow_conf'] > 0]) > 0 else 0), 2) if 'solid_yellow_conf' in df else 0,
                        'solid-white': round(float(df['solid_white_conf'][df['solid_white_conf'] > 0].mean() if len(df['solid_white_conf'][df['solid_white_conf'] > 0]) > 0 else 0), 2) if 'solid_white_conf' in df else 0,
                    }
                }
            }
            with open(exp_dir / 'summary.json', 'w') as f:
                json.dump(summary, f, indent=2)

            # Generate plots
            plots_dir = exp_dir / 'plots'
            self._plot_lateral_error(df, plots_dir)
            self._plot_heading_error(df, plots_dir)
            self._plot_cmd_vel(df, plots_dir)
            self._plot_odom_path(df, plots_dir)
            self._plot_wheel_speed(df, plots_dir)

            self.get_logger().info(
                f'[recorder] ✅ Analysis complete: {exp_dir.name}')

        except ImportError as e:
            self.get_logger().warn(
                f'[recorder] pandas/matplotlib not available: {e}')
        except Exception as e:
            self.get_logger().error(
                f'[recorder] Analysis failed: {e}')

    def _plot_lateral_error(self, df, plots_dir: Path):
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(df['time_s'], df['epsilon_x_mm'], color='#00d4aa', label='ε_x (mm)')
        ax.axhline(0, color='white', linewidth=0.5, linestyle='--', alpha=0.3)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Lateral Error (mm)')
        ax.set_title('Lateral Error vs Time')
        ax.legend()
        ax.set_facecolor('#111827')
        fig.patch.set_facecolor('#0a0e17')
        ax.tick_params(colors='white')
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        ax.title.set_color('white')
        plt.tight_layout()
        plt.savefig(plots_dir / 'lateral_error.png', dpi=150)
        plt.close(fig)

    def _plot_heading_error(self, df, plots_dir: Path):
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(df['time_s'], df['theta_rad'], color='#818cf8', label='θ (rad)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Heading Error (rad)')
        ax.set_title('Heading Error vs Time')
        ax.legend()
        ax.set_facecolor('#111827')
        fig.patch.set_facecolor('#0a0e17')
        ax.tick_params(colors='white')
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        ax.title.set_color('white')
        plt.tight_layout()
        plt.savefig(plots_dir / 'heading_error.png', dpi=150)
        plt.close(fig)

    def _plot_cmd_vel(self, df, plots_dir: Path):
        import matplotlib.pyplot as plt
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        ax1.plot(df['time_s'], df['cmd_v'], color='#00d4aa', label='v (m/s)')
        ax1.set_ylabel('Linear Velocity (m/s)')
        ax1.legend()
        ax2.plot(df['time_s'], df['cmd_omega'], color='#f87171', label='ω (rad/s)')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Angular Velocity (rad/s)')
        ax2.legend()
        fig.suptitle('Command Velocity vs Time', color='white')
        for ax in [ax1, ax2]:
            ax.set_facecolor('#111827')
            ax.tick_params(colors='white')
            ax.xaxis.label.set_color('white')
            ax.yaxis.label.set_color('white')
        fig.patch.set_facecolor('#0a0e17')
        plt.tight_layout()
        plt.savefig(plots_dir / 'cmd_vel.png', dpi=150)
        plt.close(fig)

    def _plot_odom_path(self, df, plots_dir: Path):
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.plot(df['odom_x'], df['odom_y'], color='#00d4aa', linewidth=1.5)
        ax.scatter([df['odom_x'].iloc[0]], [df['odom_y'].iloc[0]],
                   color='#10b981', s=100, label='Start', zorder=5)
        ax.scatter([df['odom_x'].iloc[-1]], [df['odom_y'].iloc[-1]],
                   color='#f87171', s=100, label='End', zorder=5)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Odometry Path (XY)')
        ax.set_aspect('equal')
        ax.legend()
        ax.set_facecolor('#111827')
        fig.patch.set_facecolor('#0a0e17')
        ax.tick_params(colors='white')
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        ax.title.set_color('white')
        plt.tight_layout()
        plt.savefig(plots_dir / 'odom_path.png', dpi=150)
        plt.close(fig)

    def _plot_wheel_speed(self, df, plots_dir: Path):
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(df['time_s'], df['v_left_ref'],   color='#00d4aa', label='v_L ref')
        ax.plot(df['time_s'], df['v_right_ref'],  color='#818cf8', label='v_R ref')
        ax.plot(df['time_s'], df['v_left_meas'],  color='#34d399', linestyle='--', label='v_L meas')
        ax.plot(df['time_s'], df['v_right_meas'], color='#a78bfa', linestyle='--', label='v_R meas')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Wheel Speed (m/s)')
        ax.set_title('Wheel Speed Reference vs Measured')
        ax.legend()
        ax.set_facecolor('#111827')
        fig.patch.set_facecolor('#0a0e17')
        ax.tick_params(colors='white')
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        ax.title.set_color('white')
        plt.tight_layout()
        plt.savefig(plots_dir / 'wheel_speed.png', dpi=150)
        plt.close(fig)

    # =========================================================================
    # Status publisher
    # =========================================================================

    def _publish_status(self):
        with self._lock:
            rec    = self._recording
            rows   = self._row_count
            meta   = dict(self._exp_metadata)
            start  = self._start_time

        status = {
            'recording': rec,
            'row_count': rows,
            'duration_s': round(time.time() - start, 1) if rec else 0,
            'metadata': meta,
        }
        msg = String()
        msg.data = json.dumps(status)
        self._pub_status.publish(msg)

    def destroy_node(self):
        if self._recording:
            self._stop_experiment()
        super().destroy_node()


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRecorderControlNode()
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
