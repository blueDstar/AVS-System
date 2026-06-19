/**
 * pure_pursuit_node.cpp
 *
 * Pure Pursuit Controller for 4-Wheel Drive (skid-steer) robot.
 *
 * Subscribes to /avs/control_error (computed by control_node) and converts
 * the 3 geometric error parameters into a ROS2 Twist command:
 *
 *   Inputs (from /avs/control_error):
 *     epsilon_x_mm  — lateral deviation  (mm), positive = waypoint is to the right
 *     epsilon_y_mm  — look-ahead distance (mm) = L_d
 *     theta_rad     — heading error       (rad)
 *     curvature_inv_mm — lane curvature   (1/mm)
 *
 *   Longitudinal control:
 *     v = v_max * cos(θ) / (1 + k_c * |κ_m|)     [m/s]
 *     clamped to [v_min, v_max]
 *
 *   Lateral control (Pure Pursuit):
 *     ω = 2 * v * ε_x_m / L_d_m²                  [rad/s]
 *     clamped to [-omega_max, +omega_max]
 *
 * Output:
 *   /cmd_vel  (geometry_msgs/Twist)
 *     linear.x  = v     [m/s]   — forward speed
 *     angular.z = omega [rad/s] — yaw rate (positive = turn left)
 *
 * Subscriptions:
 *   /avs/control_error  (std_msgs/String JSON)
 *
 * Publications:
 *   /cmd_vel            (geometry_msgs/Twist)
 */

#include <memory>
#include <string>
#include <cmath>
#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode() : Node("pure_pursuit_node") {
        // ── Declare parameters ───────────────────────────────────────────────
        // Longitudinal
        this->declare_parameter<double>("v_max",  0.5);    // m/s — max forward speed
        this->declare_parameter<double>("v_min",  0.1);    // m/s — min speed (prevents stall)
        this->declare_parameter<double>("k_c",    500.0);  // curvature penalty factor
        // Lateral
        this->declare_parameter<double>("omega_max", 2.0); // rad/s — clamp angular velocity
        this->declare_parameter<double>("Ld_min_m",  0.05);// m    — min look-ahead (avoid /0)
        // Safety
        this->declare_parameter<double>("control_error_timeout_s", 0.5);

        load_params();

        // ── Publisher ────────────────────────────────────────────────────────
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // ── Subscriber ───────────────────────────────────────────────────────
        control_error_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/avs/control_error", 10,
            std::bind(&PurePursuitNode::control_error_callback, this, std::placeholders::_1)
        );

        // ── Watchdog: zero velocity if control_error goes stale ──────────────
        watchdog_timer_ = this->create_wall_timer(
            100ms,
            std::bind(&PurePursuitNode::watchdog_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "PurePursuitNode started.");
        RCLCPP_INFO(this->get_logger(), "  v=[%.2f, %.2f] m/s  |  omega_max=%.2f rad/s  |  k_c=%.1f",
            v_min_, v_max_, omega_max_, k_c_);
        RCLCPP_INFO(this->get_logger(), "  Subscribing: /avs/control_error");
        RCLCPP_INFO(this->get_logger(), "  Publishing:  /cmd_vel");
    }

private:
    // ── Parameter reload ─────────────────────────────────────────────────────
    void load_params() {
        v_max_     = this->get_parameter("v_max").as_double();
        v_min_     = this->get_parameter("v_min").as_double();
        k_c_       = this->get_parameter("k_c").as_double();
        omega_max_ = this->get_parameter("omega_max").as_double();
        Ld_min_m_  = this->get_parameter("Ld_min_m").as_double();
        timeout_s_ = this->get_parameter("control_error_timeout_s").as_double();
    }

    // ── Control error callback ────────────────────────────────────────────────
    void control_error_callback(const std_msgs::msg::String::SharedPtr msg) {
        last_msg_time_ = this->get_clock()->now();
        has_received_ = true;

        // Reload params (allows runtime tuning via ros2 param set)
        load_params();

        try {
            json err = json::parse(msg->data);

            // ── 1. Read errors (convert mm → m) ────────────────────────────
            // epsilon_x: lateral deviation (m) — positive = waypoint is to the RIGHT
            // L_d: look-ahead distance (m)
            // theta: heading error (rad)
            // curvature: lane curvature (1/m)
            double epsilon_x_m = err.value("epsilon_x_mm", 0.0) / 1000.0;
            double L_d_m       = err.value("epsilon_y_mm", 300.0) / 1000.0;
            double theta       = err.value("theta_rad",    0.0);
            double kappa_m     = err.value("curvature_inv_mm", 0.0) * 1000.0; // 1/mm → 1/m

            // ── 2. Longitudinal control: adaptive speed ─────────────────────
            // Reduce speed when:
            //   - heading error is large  → cos(θ) decreases
            //   - lane curvature is high  → denominator increases
            double v = (v_max_ * std::cos(theta)) / (1.0 + k_c_ * std::abs(kappa_m));
            v = std::clamp(v, v_min_, v_max_);

            // ── 3. Lateral control: Pure Pursuit ───────────────────────────
            // Formula: ω = 2 * v * ε_x / L_d²
            // Derivation: curvature of arc = 2*ε_x/L_d², ω = v * curvature
            // Note: wheelbase cancels out — no vehicle geometry needed here
            double omega = 0.0;
            if (L_d_m > Ld_min_m_) {
                omega = (2.0 * v * epsilon_x_m) / (L_d_m * L_d_m);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "L_d=%.3fm below threshold. omega set to 0.", L_d_m);
            }
            omega = std::clamp(omega, -omega_max_, omega_max_);

            // ── 4. Publish /cmd_vel ─────────────────────────────────────────
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x  = v;
            cmd.angular.z = omega;
            cmd_vel_pub_->publish(cmd);

            RCLCPP_DEBUG(this->get_logger(),
                "ε_x=%.1fmm  L_d=%.0fmm  θ=%.3frad  κ=%.4f/m  →  v=%.3fm/s  ω=%.3frad/s",
                epsilon_x_m * 1000.0, L_d_m * 1000.0, theta, kappa_m, v, omega);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "control_error_callback parse error: %s", e.what());
        }
    }

    // ── Watchdog: zero velocity if no error received within timeout ──────────
    void watchdog_callback() {
        if (!has_received_) return;

        double age_s = (this->get_clock()->now() - last_msg_time_).seconds();
        if (age_s > timeout_s_) {
            geometry_msgs::msg::Twist zero;
            cmd_vel_pub_->publish(zero);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "control_error stale (%.2fs). Publishing zero velocity.", age_s);
        }
    }

    // ── ROS2 interfaces ──────────────────────────────────────────────────────
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        control_error_sub_;
    rclcpp::TimerBase::SharedPtr                                   watchdog_timer_;

    // ── Staleness tracking ───────────────────────────────────────────────────
    bool          has_received_  = false;
    rclcpp::Time  last_msg_time_{0, 0, RCL_ROS_TIME};

    // ── Cached parameters ────────────────────────────────────────────────────
    double v_max_, v_min_, k_c_, omega_max_, Ld_min_m_, timeout_s_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
