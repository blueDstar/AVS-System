"""
AVS Dashboard System — cmd_vel Multiplexer Control Node
File: avs_dashboard_system/cmd_vel_mux_control_node.py

SAFETY CRITICAL: Only ONE source publishes to /cmd_vel at any time.

Architecture:
  /avs/cmd_vel/manual          ─┐
  /avs/cmd_vel/main_pd         ─┤
  /avs/cmd_vel/pd_lidar        ─┼─ cmd_vel_mux_control ──► /cmd_vel
  /avs/cmd_vel/backstepping_pd ─┤
  /avs/cmd_vel/cascade_pd      ─┤
  /avs/cmd_vel/sim             ─┘

  /avs/selected_controller     ─► selects active source
  /avs/emergency_stop          ─► overrides ALL with zero

Safety rules (priority order):
  1. emergency_stop = True  → publish zero, ignore all else
  2. selected_controller = 'off' → publish zero
  3. Active source timed out    → publish zero, log warning
  4. Apply rate limiting (accel ramp)
  5. Apply velocity clamp
  6. Publish to /cmd_vel

Publishes:
  /cmd_vel                     geometry_msgs/Twist (20 Hz)
  /avs/cmd_vel_mux_state       std_msgs/String JSON (5 Hz)
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
    qos_profile_sensor_data
)
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

from avs_dashboard_system.utils import (
    RateLimiter, TimeoutChecker, clamp
)


RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE
)

# Map: controller name → source topic name (must match selected_controller values)
CONTROLLER_TOPICS = {
    'manual':         '/avs/cmd_vel/manual',
    'main_pd':        '/avs/cmd_vel/main_pd',
    'pd_lidar':       '/avs/cmd_vel/pd_lidar',
    'backstepping_pd': '/avs/cmd_vel/backstepping_pd',
    'cascade_pd':     '/avs/cmd_vel/cascade_pd',
    'simulation':     '/avs/cmd_vel/sim',
}


class SourceState:
    """Holds latest cmd_vel from one source."""
    def __init__(self, timeout_s: float = 0.5):
        self.v: float = 0.0
        self.omega: float = 0.0
        self.timeout = TimeoutChecker(timeout_s)

    def update(self, v: float, omega: float):
        self.v = v
        self.omega = omega
        self.timeout.touch()

    @property
    def timed_out(self) -> bool:
        return self.timeout.timed_out


class CmdVelMuxControlNode(Node):
    """
    Reads cmd_vel from multiple sources, selects the active one based on
    /avs/selected_controller, applies safety checks, rate limiting, and
    clamp, then publishes to /cmd_vel.
    """

    def __init__(self):
        super().__init__('cmd_vel_mux_control')

        # ---- Parameters ----
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('status_hz', 5.0)
        self.declare_parameter('cmd_source_timeout_s', 0.5)
        self.declare_parameter('v_max', 0.3)
        self.declare_parameter('v_min', -0.1)
        self.declare_parameter('omega_max', 2.0)
        self.declare_parameter('max_accel', 0.5)
        self.declare_parameter('max_alpha', 1.0)
        self.declare_parameter('topic_cmd_vel', '/cmd_vel')
        self.declare_parameter('topic_selected_ctrl', '/avs/selected_controller')
        self.declare_parameter('topic_emergency_stop', '/avs/emergency_stop')
        self.declare_parameter('topic_mux_state', '/avs/cmd_vel_mux_state')

        timeout_s    = self.get_parameter('cmd_source_timeout_s').value
        self._v_max  = self.get_parameter('v_max').value
        self._v_min  = self.get_parameter('v_min').value
        self._w_max  = self.get_parameter('omega_max').value
        pub_hz       = self.get_parameter('publish_hz').value
        status_hz    = self.get_parameter('status_hz').value

        # ---- Internal state ----
        self._lock = threading.Lock()
        self._emergency_stop: bool = False
        self._selected_controller: str = 'off'
        self._last_pub_v: float = 0.0
        self._last_pub_omega: float = 0.0

        # Rate limiters
        self._v_limiter     = RateLimiter(self.get_parameter('max_accel').value)
        self._omega_limiter = RateLimiter(self.get_parameter('max_alpha').value)

        # One SourceState per controller
        self._sources: dict[str, SourceState] = {
            name: SourceState(timeout_s) for name in CONTROLLER_TOPICS
        }

        # ---- Publishers ----
        self._pub_cmd_vel = self.create_publisher(
            Twist,
            self.get_parameter('topic_cmd_vel').value,
            RELIABLE_QOS
        )
        self._pub_mux_state = self.create_publisher(
            String,
            self.get_parameter('topic_mux_state').value,
            RELIABLE_QOS
        )

        # ---- Subscribers for each source ----
        r10 = QoSProfile(depth=10)
        sd  = qos_profile_sensor_data

        for ctrl_name, topic in CONTROLLER_TOPICS.items():
            # Capture ctrl_name in closure
            def _make_cb(name):
                def _cb(msg: Twist):
                    with self._lock:
                        self._sources[name].update(
                            msg.linear.x, msg.angular.z)
                return _cb
            self.create_subscription(Twist, topic, _make_cb(ctrl_name), sd)

        self.create_subscription(String,
            self.get_parameter('topic_selected_ctrl').value,
            self._cb_selected_controller, r10)
        self.create_subscription(Bool,
            self.get_parameter('topic_emergency_stop').value,
            self._cb_emergency_stop, r10)

        # ---- Timers ----
        self._dt = 1.0 / pub_hz
        self._last_pub_time = time.monotonic()
        self.create_timer(self._dt, self._publish_cmd_vel)
        self.create_timer(1.0 / status_hz, self._publish_mux_state)

        self.get_logger().info(
            f'[cmd_vel_mux_control] started | '
            f'publish={pub_hz}Hz | timeout={timeout_s}s | '
            f'v_max={self._v_max} | omega_max={self._w_max}'
        )

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _cb_selected_controller(self, msg: String):
        ctrl = msg.data.strip()
        with self._lock:
            if ctrl != self._selected_controller:
                self.get_logger().info(
                    f'[cmd_vel_mux_control] Controller switched: '
                    f'{self._selected_controller} → {ctrl}'
                )
                # Reset rate limiters on switch for smooth transition
                self._v_limiter.reset(0.0)
                self._omega_limiter.reset(0.0)
                self._selected_controller = ctrl

    def _cb_emergency_stop(self, msg: Bool):
        with self._lock:
            if msg.data and not self._emergency_stop:
                self.get_logger().warn(
                    '[cmd_vel_mux_control] ⚠ EMERGENCY STOP ACTIVATED')
                self._v_limiter.reset(0.0)
                self._omega_limiter.reset(0.0)
            elif not msg.data and self._emergency_stop:
                self.get_logger().info(
                    '[cmd_vel_mux_control] Emergency stop cleared')
            self._emergency_stop = msg.data

    # =========================================================================
    # Publish cmd_vel
    # =========================================================================

    def _publish_cmd_vel(self):
        now = time.monotonic()
        dt  = now - self._last_pub_time
        self._last_pub_time = now

        with self._lock:
            e_stop  = self._emergency_stop
            ctrl    = self._selected_controller
            v_max   = self._v_max
            v_min   = self._v_min
            w_max   = self._w_max

        # --- Priority 1: Emergency stop ---
        if e_stop:
            v_target, omega_target = 0.0, 0.0
            stop_reason = 'emergency_stop'
        elif ctrl == 'off':
            v_target, omega_target = 0.0, 0.0
            stop_reason = 'controller_off'
        elif ctrl not in self._sources:
            v_target, omega_target = 0.0, 0.0
            stop_reason = f'unknown_controller:{ctrl}'
            self.get_logger().warn(
                f'[cmd_vel_mux_control] Unknown controller: {ctrl}')
        else:
            with self._lock:
                src = self._sources[ctrl]
                timed_out = src.timed_out
                v_raw     = src.v
                omega_raw = src.omega

            if timed_out:
                v_target, omega_target = 0.0, 0.0
                stop_reason = f'source_timeout:{ctrl}'
            else:
                v_target     = clamp(v_raw, v_min, v_max)
                omega_target = clamp(omega_raw, -w_max, w_max)
                stop_reason  = None

        # --- Rate limiting (smooth deceleration/acceleration) ---
        with self._lock:
            v_out     = self._v_limiter.update(v_target, dt)
            omega_out = self._omega_limiter.update(omega_target, dt)
            # On stop commands: allow instant zero (safety)
            if stop_reason and abs(v_out) < 0.001:
                self._v_limiter.reset(0.0)
                self._omega_limiter.reset(0.0)
            self._last_pub_v     = v_out
            self._last_pub_omega = omega_out

        # --- Publish ---
        twist = Twist()
        twist.linear.x  = v_out
        twist.angular.z = omega_out
        self._pub_cmd_vel.publish(twist)

        # Warn on sustained timeout
        if stop_reason and 'timeout' in (stop_reason or ''):
            self.get_logger().warn(
                f'[cmd_vel_mux_control] Source timed out → publishing zero | '
                f'controller={ctrl}'
            )

    def _publish_mux_state(self):
        """Publish mux status JSON for monitoring."""
        with self._lock:
            e_stop = self._emergency_stop
            ctrl   = self._selected_controller
            v_pub  = self._last_pub_v
            w_pub  = self._last_pub_omega
            sources_info = {}
            for name, src in self._sources.items():
                sources_info[name] = {
                    'v': round(src.v, 4),
                    'omega': round(src.omega, 4),
                    'timed_out': src.timed_out,
                    'age_s': round(src.timeout.age_s, 3),
                    'active': (name == ctrl),
                }

        state = {
            'time': time.time(),
            'emergency_stop': e_stop,
            'selected_controller': ctrl,
            'cmd_vel_out': {
                'v': round(v_pub, 4),
                'omega': round(w_pub, 4),
            },
            'sources': sources_info,
        }
        msg = String()
        msg.data = json.dumps(state)
        self._pub_mux_state.publish(msg)

    def _send_zero(self):
        """Send zero twist (called on shutdown)."""
        twist = Twist()
        self._pub_cmd_vel.publish(twist)

    def destroy_node(self):
        """Safe shutdown: publish zero before destroying."""
        self.get_logger().info(
            '[cmd_vel_mux_control] Shutting down — sending zero cmd_vel')
        self._send_zero()
        super().destroy_node()


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMuxControlNode()
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
