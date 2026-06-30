"""
AVS Dashboard System — Telemetry Aggregator Control Node
File: avs_dashboard_system/telemetry_aggregator_control_node.py

Subscribes to all sensor/control topics, computes Hz & timeout flags,
and publishes a single consolidated JSON state to /avs/dashboard_state
at a configurable rate (default 10 Hz).

Node name: telemetry_aggregator_control
Publish:   /avs/dashboard_state  (std_msgs/String, JSON)
"""

import json
import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
    qos_profile_sensor_data
)
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

from avs_dashboard_system.utils import (
    HzCalculator, TimeoutChecker, safe_quat_to_euler,
    parse_json_string, safe_get, clamp
)


# QoS for reliable publisher
RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE
)


class TelemetryAggregatorControlNode(Node):
    """
    Subscribes to ~15 ROS topics, aggregates into one JSON dashboard_state,
    publishes at 10 Hz. Computes per-topic Hz using sliding window and
    timeout flags.
    """

    def __init__(self):
        super().__init__('telemetry_aggregator_control')

        # ---- Parameters ----
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('hz_window_s', 2.0)
        self.declare_parameter('ros_domain_id', 20)
        self.declare_parameter('runtime_mode', 'real_robot')
        # Timeout thresholds
        self.declare_parameter('timeout_odom_s', 1.0)
        self.declare_parameter('timeout_imu_s', 2.0)
        self.declare_parameter('timeout_scan_s', 2.0)
        self.declare_parameter('timeout_control_error_s', 3.0)
        # Topics (configurable)
        self.declare_parameter('topic_cmd_vel', '/cmd_vel')
        self.declare_parameter('topic_odom_raw', '/odom_raw')
        self.declare_parameter('topic_imu', '/imu')
        self.declare_parameter('topic_scan', '/scan')
        self.declare_parameter('topic_control_error', '/avs/control_error')
        self.declare_parameter('topic_telemetry', '/avs/telemetry')
        self.declare_parameter('topic_telemetry_rw', '/avs/telemetry_realworld')
        self.declare_parameter('topic_main_pd_debug', '/avs/main_following_pd_debug')
        self.declare_parameter('topic_lane_pd_state', '/avs/lane_pd_state')
        self.declare_parameter('topic_wheel_pd_state', '/avs/wheel_pd_state')
        self.declare_parameter('topic_cascade_state', '/avs/cascade_control_state')
        self.declare_parameter('topic_selected_ctrl', '/avs/selected_controller')
        self.declare_parameter('topic_emergency_stop', '/avs/emergency_stop')
        self.declare_parameter('topic_dashboard_state', '/avs/dashboard_state')

        # Read parameters
        pub_hz = self.get_parameter('publish_hz').value
        win_s  = self.get_parameter('hz_window_s').value

        # ---- Internal state (thread-safe via lock) ----
        self._lock = threading.Lock()

        # cmd_vel state
        self._cmd_v: float = 0.0
        self._cmd_omega: float = 0.0
        self._cmd_hz = HzCalculator(win_s)

        # Odometry
        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._odom_yaw: float = 0.0
        self._odom_v: float = 0.0
        self._odom_omega: float = 0.0
        self._odom_hz = HzCalculator(win_s)
        self._odom_timeout = TimeoutChecker(
            self.get_parameter('timeout_odom_s').value)

        # IMU
        self._imu_yaw: float = 0.0
        self._imu_wz: float = 0.0
        self._imu_hz = HzCalculator(win_s)
        self._imu_timeout = TimeoutChecker(
            self.get_parameter('timeout_imu_s').value)

        # LiDAR
        self._lidar_front_min: float = 9.9
        self._lidar_left_min: float = 9.9
        self._lidar_right_min: float = 9.9
        self._lidar_hz = HzCalculator(win_s)
        self._lidar_timeout = TimeoutChecker(
            self.get_parameter('timeout_scan_s').value)

        # Lane / control error (from /avs/control_error or /avs/telemetry)
        self._lane_valid: bool = False
        self._lane_state: str = 'UNKNOWN'
        self._lane_eps_x: float = 0.0
        self._lane_eps_y: float = 0.0
        self._lane_theta: float = 0.0
        self._lane_fps: float = 0.0
        self._lane_hz = HzCalculator(win_s)
        self._lane_timeout = TimeoutChecker(
            self.get_parameter('timeout_control_error_s').value)
        self._lane_classes: list = []
        self._lane_classes_detected: list = []
        self._lane_polygons: list = []
        self._lane_bboxes: list = []

        # Controller debug (cascade/PD wheel states)
        self._ctrl_debug: dict = {}
        self._ctrl_debug_hz = HzCalculator(win_s)

        # System state
        self._selected_controller: str = 'off'
        self._emergency_stop: bool = False
        self._runtime_mode: str = self.get_parameter('runtime_mode').value

        # Publisher
        self._pub_state = self.create_publisher(
            String,
            self.get_parameter('topic_dashboard_state').value,
            RELIABLE_QOS
        )

        # ---- Subscribers ----
        sd = qos_profile_sensor_data  # Best-effort, for sensor topics
        r10 = QoSProfile(depth=10)    # Reliable, for control topics

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
            self.get_parameter('topic_telemetry').value,
            self._cb_telemetry, r10)
        self.create_subscription(String,
            self.get_parameter('topic_telemetry_rw').value,
            self._cb_telemetry_rw, r10)
        self.create_subscription(String,
            self.get_parameter('topic_lane_pd_state').value,
            self._cb_lane_pd_state, r10)
        self.create_subscription(String,
            self.get_parameter('topic_wheel_pd_state').value,
            self._cb_wheel_pd_state, r10)
        self.create_subscription(String,
            self.get_parameter('topic_cascade_state').value,
            self._cb_cascade_state, r10)
        self.create_subscription(String,
            self.get_parameter('topic_selected_ctrl').value,
            self._cb_selected_controller, r10)
        self.create_subscription(Bool,
            self.get_parameter('topic_emergency_stop').value,
            self._cb_emergency_stop, r10)

        # ---- Publish timer ----
        period = 1.0 / pub_hz
        self._publish_timer = self.create_timer(period, self._publish_state)

        self.get_logger().info(
            f'[telemetry_aggregator_control] started, publishing at {pub_hz} Hz '
            f'on {self.get_parameter("topic_dashboard_state").value}'
        )

    # =========================================================================
    # Subscription callbacks
    # =========================================================================

    def _cb_cmd_vel(self, msg: Twist):
        with self._lock:
            self._cmd_v     = msg.linear.x
            self._cmd_omega = msg.angular.z
            self._cmd_hz.tick()

    def _cb_odom(self, msg: Odometry):
        with self._lock:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            _, _, yaw = safe_quat_to_euler(q.x, q.y, q.z, q.w)
            self._odom_x     = p.x
            self._odom_y     = p.y
            self._odom_yaw   = yaw
            self._odom_v     = msg.twist.twist.linear.x
            self._odom_omega = msg.twist.twist.angular.z
            self._odom_hz.tick()
            self._odom_timeout.touch()

    def _cb_imu(self, msg: Imu):
        with self._lock:
            q = msg.orientation
            _, _, yaw = safe_quat_to_euler(q.x, q.y, q.z, q.w)
            self._imu_yaw = yaw
            self._imu_wz  = msg.angular_velocity.z
            self._imu_hz.tick()
            self._imu_timeout.touch()

    def _cb_scan(self, msg: LaserScan):
        """Parse LaserScan into front/left/right sector minimums."""
        with self._lock:
            ranges = msg.ranges
            n = len(ranges)
            if n == 0:
                return

            angle_min  = msg.angle_min
            angle_inc  = msg.angle_increment
            range_max  = msg.range_max
            range_min  = msg.range_min

            front_min = range_max
            left_min  = range_max
            right_min = range_max

            front_half_deg = math.radians(30.0)

            for i, r in enumerate(ranges):
                if not math.isfinite(r) or r < range_min or r > range_max:
                    continue
                angle = angle_min + i * angle_inc
                # Front: ±30°
                if abs(angle) <= front_half_deg:
                    front_min = min(front_min, r)
                # Left: 30°..150°
                elif front_half_deg < angle <= math.radians(150.0):
                    left_min = min(left_min, r)
                # Right: -150°..-30°
                elif -math.radians(150.0) <= angle < -front_half_deg:
                    right_min = min(right_min, r)

            self._lidar_front_min = front_min
            self._lidar_left_min  = left_min
            self._lidar_right_min = right_min
            self._lidar_hz.tick()
            self._lidar_timeout.touch()

    def _cb_control_error(self, msg: String):
        """Parse /avs/control_error JSON (lane detection state)."""
        data = parse_json_string(msg.data)
        if not data:
            return
        with self._lock:
            self._lane_valid  = bool(safe_get(data, 'valid', default=False))
            self._lane_state  = str(safe_get(data, 'state', default='UNKNOWN'))
            self._lane_eps_x  = float(safe_get(data, 'epsilon_x_mm', default=0.0))
            self._lane_eps_y  = float(safe_get(data, 'epsilon_y_mm', default=0.0))
            self._lane_theta  = float(safe_get(data, 'theta_rad', default=0.0))
            self._lane_fps    = float(safe_get(data, 'fps_est', default=0.0))
            self._lane_classes = safe_get(data, 'classes', default=safe_get(data, 'detected_classes', default=[]))
            self._lane_polygons = safe_get(data, 'polygons', default=[])
            self._lane_bboxes = safe_get(data, 'bboxes', default=[])
            self._lane_hz.tick()
            self._lane_timeout.touch()

    def _cb_telemetry(self, msg: String):
        """Parse /avs/telemetry — merged with control_error if needed."""
        data = parse_json_string(msg.data)
        if not data:
            return
        with self._lock:
            # Update fields not already covered by control_error
            if 'lane' in data:
                lane = data['lane']
                self._lane_valid  = bool(lane.get('valid', self._lane_valid))
                self._lane_state  = str(lane.get('state', self._lane_state))
                self._lane_eps_x  = float(lane.get('epsilon_x_mm', self._lane_eps_x))
                self._lane_fps    = float(lane.get('fps_est', self._lane_fps))
                self._lane_classes = lane.get('classes', lane.get('detected_classes', self._lane_classes))
                self._lane_polygons = lane.get('polygons', self._lane_polygons)
                self._lane_bboxes = lane.get('bboxes', self._lane_bboxes)
            if 'polygons' in data:
                self._lane_polygons = data['polygons']
            if 'bboxes' in data:
                self._lane_bboxes = data['bboxes']
            if 'controller_debug' in data:
                self._ctrl_debug.update(data['controller_debug'])
                self._ctrl_debug_hz.tick()

            # Parse detections for classes_detected stats
            raw_detections = data.get('detections', data.get('objects', data.get('segments', [])))
            if isinstance(raw_detections, list) and len(raw_detections) > 0:
                class_stats = {}
                for det in raw_detections:
                    if not isinstance(det, dict): continue
                    c_name = str(det.get('class_name', det.get('label', det.get('class', 'unknown'))))
                    conf = float(det.get('confidence', det.get('score', det.get('prob', 0.0))))
                    if c_name not in class_stats:
                        class_stats[c_name] = {'count': 0, 'max': 0.0, 'sum': 0.0}
                    class_stats[c_name]['count'] += 1
                    class_stats[c_name]['max'] = max(class_stats[c_name]['max'], conf)
                    class_stats[c_name]['sum'] += conf
                
                new_detected = []
                for c_name, stats in class_stats.items():
                    new_detected.append({
                        'class_name': c_name,
                        'count': stats['count'],
                        'max_confidence': round(stats['max'], 2),
                        'mean_confidence': round(stats['sum'] / stats['count'], 2),
                        'last_seen_time': time.time()
                    })
                
                # Merge with existing classes to update last_seen
                current_time = time.time()
                merged_classes = []
                # First add all new ones
                merged_classes.extend(new_detected)
                # Then add old ones that weren't detected this frame, if they aren't too old
                new_names = {d['class_name'] for d in new_detected}
                for old_det in self._lane_classes_detected:
                    if old_det['class_name'] not in new_names:
                        merged_classes.append(old_det)
                
                # Calculate age
                for d in merged_classes:
                    d['last_seen_age_s'] = round(current_time - d.get('last_seen_time', current_time), 2)
                
                self._lane_classes_detected = merged_classes

    def _cb_telemetry_rw(self, msg: String):
        """Parse /avs/telemetry_realworld — same structure, overrides."""
        self._cb_telemetry(msg)

    def _cb_lane_pd_state(self, msg: String):
        data = parse_json_string(msg.data)
        if not data:
            return
        with self._lock:
            self._ctrl_debug.update(data)
            self._ctrl_debug_hz.tick()

    def _cb_wheel_pd_state(self, msg: String):
        data = parse_json_string(msg.data)
        if not data:
            return
        with self._lock:
            self._ctrl_debug.update(data)
            self._ctrl_debug_hz.tick()

    def _cb_cascade_state(self, msg: String):
        data = parse_json_string(msg.data)
        if not data:
            return
        with self._lock:
            self._ctrl_debug.update(data)
            self._ctrl_debug_hz.tick()

    def _cb_selected_controller(self, msg: String):
        with self._lock:
            self._selected_controller = msg.data.strip()

    def _cb_emergency_stop(self, msg: Bool):
        with self._lock:
            self._emergency_stop = msg.data

    # =========================================================================
    # Publish aggregated state
    # =========================================================================

    def _publish_state(self):
        with self._lock:
            state = {
                "time": time.time(),
                "system": {
                    "ros_domain_id": self.get_parameter('ros_domain_id').value,
                    "mode": self._runtime_mode,
                    "active_controller": self._selected_controller,
                    "emergency_stop": self._emergency_stop,
                    "cmd_vel_hz": round(self._cmd_hz.hz, 2),
                },
                "cmd_vel": {
                    "v": round(self._cmd_v, 4),
                    "omega": round(self._cmd_omega, 4),
                    "hz": round(self._cmd_hz.hz, 2),
                },
                "odom": {
                    "x": round(self._odom_x, 4),
                    "y": round(self._odom_y, 4),
                    "yaw": round(self._odom_yaw, 4),
                    "v": round(self._odom_v, 4),
                    "omega": round(self._odom_omega, 4),
                    "hz": round(self._odom_hz.hz, 2),
                    "timeout": self._odom_timeout.timed_out,
                },
                "imu": {
                    "yaw": round(self._imu_yaw, 4),
                    "wz": round(self._imu_wz, 4),
                    "hz": round(self._imu_hz.hz, 2),
                    "timeout": self._imu_timeout.timed_out,
                },
                "lidar": {
                    "front_min": round(self._lidar_front_min, 3),
                    "left_min": round(self._lidar_left_min, 3),
                    "right_min": round(self._lidar_right_min, 3),
                    "hz": round(self._lidar_hz.hz, 2),
                    "timeout": self._lidar_timeout.timed_out,
                },
                "lane": {
                    "valid": self._lane_valid,
                    "state": self._lane_state,
                    "epsilon_x_mm": round(self._lane_eps_x, 2),
                    "epsilon_y_mm": round(self._lane_eps_y, 2),
                    "theta_rad": round(self._lane_theta, 4),
                    "fps_est": round(self._lane_fps, 2),
                    "hz": round(self._lane_hz.hz, 2),
                    "timeout": self._lane_timeout.timed_out,
                    "classes": self._lane_classes,
                    "classes_detected": self._lane_classes_detected,
                    "polygons": self._lane_polygons,
                    "bboxes": self._lane_bboxes,
                    "special_zones": {
                        "stop_line_detected": any(c['class_name'] == 'stop-line' and c['last_seen_age_s'] < 0.5 for c in self._lane_classes_detected),
                        "parking_zone_detected": any(c['class_name'] == 'parking-zone' and c['last_seen_age_s'] < 0.5 for c in self._lane_classes_detected),
                        "turn_lane_detected": any(c['class_name'] == 'turn-lane' and c['last_seen_age_s'] < 0.5 for c in self._lane_classes_detected),
                        "vehicle_detected": any(c['class_name'] == 'vehicle' and c['last_seen_age_s'] < 0.5 for c in self._lane_classes_detected),
                    }
                },
                "controller_debug": dict(self._ctrl_debug),
            }

        msg = String()
        msg.data = json.dumps(state, ensure_ascii=False)
        self._pub_state.publish(msg)


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryAggregatorControlNode()
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
