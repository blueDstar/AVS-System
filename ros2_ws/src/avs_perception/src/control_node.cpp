/**
 * control_node.cpp
 *
 * AVS Lane Error Publisher — Computes and publishes the 3 control error
 * parameters in the vehicle frame coordinate system:
 *
 *   Vehicle Frame (origin O = bottom-center of camera frame projected to ground):
 *     X — lateral  (right = positive, left = negative)
 *     Y — forward  (ahead = positive)
 *
 *   Control errors published:
 *     epsilon_x_mm   : lateral deviation  = x-coordinate of look-ahead waypoint
 *     epsilon_y_mm   : longitudinal deviation = y-coordinate of look-ahead waypoint
 *     theta_rad      : heading error = angle of line (O → waypoint) from Y-axis
 *                      = atan2(epsilon_x, epsilon_y)
 *
 * Lane selection state (which lane's waypoints serve as setpoint to origin O):
 *   FOLLOW_MAIN  : main-lane centerline is the setpoint
 *   LANE_CHANGE  : other-lane centerline is the setpoint
 *   TURNING      : turn-lane centerline is the setpoint
 *
 * Subscriptions:
 *   /avs/telemetry_realworld  (std_msgs/String JSON) — pre-computed look-ahead errors
 *   /avs/cmd                  (std_msgs/String JSON) — {"cmd": "lane_change"|"turn"|"resume"}
 *
 * Publications:
 *   /avs/control_error        (std_msgs/String JSON) — {epsilon_x_mm, epsilon_y_mm, theta_rad, ...}
 *   /avs/lane_state           (std_msgs/String JSON) — current lane selection state
 */

#include <memory>
#include <string>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────────────────────
// Lane selection state
// ─────────────────────────────────────────────────────────────────────────────
enum class LaneState {
    FOLLOW_MAIN,  // Track main-lane (label=3)
    LANE_CHANGE,  // Track other-lane (label=4) as target
    TURNING       // Track turn-lane (label=10) as target
};

static const char* lane_state_name(LaneState s) {
    switch (s) {
        case LaneState::FOLLOW_MAIN: return "FOLLOW_MAIN";
        case LaneState::LANE_CHANGE: return "LANE_CHANGE";
        case LaneState::TURNING:     return "TURNING";
    }
    return "UNKNOWN";
}

// ─────────────────────────────────────────────────────────────────────────────
// LaneErrorNode
// ─────────────────────────────────────────────────────────────────────────────
class LaneErrorNode : public rclcpp::Node {
public:
    LaneErrorNode() : Node("control_node") {
        // ── Declare turn trigger thresholds ──────────────────────────────────
        // These determine when to switch to turn-lane errors.
        // The actual PD controller is a separate node.
        this->declare_parameter<double>("turn_proximity_mm",  500.0);
        this->declare_parameter<double>("turn_done_mm",       200.0);
        this->declare_parameter<double>("theta_done_rad",     0.1);

        turn_proximity_mm_ = this->get_parameter("turn_proximity_mm").as_double();
        turn_done_mm_      = this->get_parameter("turn_done_mm").as_double();
        theta_done_rad_    = this->get_parameter("theta_done_rad").as_double();

        // ── Publishers ───────────────────────────────────────────────────────
        control_error_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/avs/control_error", 10);
        lane_state_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/avs/lane_state", 10);

        // ── Subscribers ──────────────────────────────────────────────────────
        telemetry_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/avs/telemetry_realworld", 10,
            std::bind(&LaneErrorNode::telemetry_callback, this, std::placeholders::_1)
        );
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/avs/cmd", 10,
            std::bind(&LaneErrorNode::cmd_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "LaneErrorNode started. Initial state: FOLLOW_MAIN");
        RCLCPP_INFO(this->get_logger(), "Subscribing: /avs/telemetry_realworld, /avs/cmd");
        RCLCPP_INFO(this->get_logger(), "Publishing:  /avs/control_error, /avs/lane_state");
    }

private:
    // ── External command callback ────────────────────────────────────────────
    void cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
        try {
            json cmd_json = json::parse(msg->data);
            std::string cmd = cmd_json.value("cmd", "");

            if (cmd == "lane_change" && state_ == LaneState::FOLLOW_MAIN) {
                state_ = LaneState::LANE_CHANGE;
                RCLCPP_INFO(this->get_logger(), "CMD: FOLLOW_MAIN → LANE_CHANGE");

            } else if (cmd == "turn" && state_ == LaneState::FOLLOW_MAIN) {
                // Arm the turn intent — actual transition happens in telemetry_callback
                // once perception conditions are also satisfied
                external_turn_cmd_ = true;
                RCLCPP_INFO(this->get_logger(), "CMD: turn intent armed.");

            } else if (cmd == "resume") {
                state_ = LaneState::FOLLOW_MAIN;
                external_turn_cmd_ = false;
                RCLCPP_INFO(this->get_logger(), "CMD: → FOLLOW_MAIN (resume)");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "cmd_callback parse error: %s", e.what());
        }
    }

    // ── Telemetry callback: select lane, extract errors, publish ─────────────
    void telemetry_callback(const std_msgs::msg::String::SharedPtr msg) {
        // Reload thresholds in case they were updated at runtime
        turn_proximity_mm_ = this->get_parameter("turn_proximity_mm").as_double();
        turn_done_mm_      = this->get_parameter("turn_done_mm").as_double();
        theta_done_rad_    = this->get_parameter("theta_done_rad").as_double();

        try {
            json telemetry = json::parse(msg->data);
            if (!telemetry.contains("objects") || !telemetry["objects"].is_array()) return;

            // ── Collect lane objects by label ───────────────────────────────
            const json* main_lane_obj  = nullptr;
            const json* other_lane_obj = nullptr;
            const json* turn_lane_obj  = nullptr;
            bool stop_line_detected    = false;

            for (const auto& obj : telemetry["objects"]) {
                int label = obj.value("label", -1);
                if (label == 3)  main_lane_obj  = &obj;
                if (label == 4)  other_lane_obj = &obj;
                if (label == 10) turn_lane_obj  = &obj;
                if (label == 9)  stop_line_detected = true;
            }

            // ── State transition logic ──────────────────────────────────────
            update_lane_state(turn_lane_obj, stop_line_detected);

            // ── Select target lane based on current state ───────────────────
            const json* target_obj = nullptr;
            switch (state_) {
                case LaneState::FOLLOW_MAIN:
                    target_obj = main_lane_obj;
                    break;
                case LaneState::LANE_CHANGE:
                    // If other-lane not visible, fallback to main-lane
                    target_obj = (other_lane_obj != nullptr) ? other_lane_obj : main_lane_obj;
                    break;
                case LaneState::TURNING:
                    target_obj = turn_lane_obj;
                    break;
            }

            // ── Extract and publish control errors ──────────────────────────
            if (target_obj != nullptr) {
                publish_control_error(*target_obj);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "[%s] Target lane not detected — no error published.",
                    lane_state_name(state_));
            }

            // ── Always publish lane state ───────────────────────────────────
            publish_lane_state(
                main_lane_obj  != nullptr,
                other_lane_obj != nullptr,
                turn_lane_obj  != nullptr,
                stop_line_detected
            );

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "telemetry_callback error: %s", e.what());
        }
    }

    // ── Lane state transition logic ──────────────────────────────────────────
    void update_lane_state(const json* turn_lane_obj, bool stop_line_detected) {
        switch (state_) {

            case LaneState::FOLLOW_MAIN: {
                // Transition to TURNING when all conditions are met:
                //   1. turn-lane detected (label=10)
                //   2. turn-lane longitudinal offset < threshold (lane is close ahead)
                //   3. stop-line detected OR external turn command was received
                if (turn_lane_obj != nullptr && external_turn_cmd_) {
                    double long_off  = turn_lane_obj->value("longitudinal_offset_mm", 1e9);
                    bool turn_close  = (long_off < turn_proximity_mm_);
                    bool turn_cued   = stop_line_detected || external_turn_cmd_;

                    if (turn_close && turn_cued) {
                        state_ = LaneState::TURNING;
                        external_turn_cmd_ = false;
                        RCLCPP_INFO(this->get_logger(),
                            "State → TURNING (long_off=%.0fmm, stop_line=%s)",
                            long_off, stop_line_detected ? "yes" : "no");
                    }
                }
                break;
            }

            case LaneState::LANE_CHANGE:
                // Controlled externally via /avs/cmd "resume"
                // (higher-level planner decides when lane change is complete)
                break;

            case LaneState::TURNING: {
                // Turn complete when heading is near zero AND vehicle has passed turn point
                if (turn_lane_obj != nullptr) {
                    double theta_t  = turn_lane_obj->value("lookahead_theta_rad", 1e9);
                    double long_off = turn_lane_obj->value("longitudinal_offset_mm", 1e9);
                    bool heading_ok = (std::abs(theta_t) < theta_done_rad_);
                    bool past_turn  = (long_off < -turn_done_mm_);  // negative = passed the turn

                    if (heading_ok && past_turn) {
                        state_ = LaneState::FOLLOW_MAIN;
                        RCLCPP_INFO(this->get_logger(), "State → FOLLOW_MAIN (turn complete)");
                    }
                } else {
                    // Turn-lane lost: revert to main-lane following
                    state_ = LaneState::FOLLOW_MAIN;
                    RCLCPP_WARN(this->get_logger(), "Turn-lane lost. State → FOLLOW_MAIN");
                }
                break;
            }
        }
    }

    // ── Publish the 3 control error parameters ───────────────────────────────
    //
    //  For main-lane / other-lane  (x(y) polynomial — sweep along Y):
    //    epsilon_x = lookahead_x_mm     (lateral position of centerline at look-ahead d)
    //    epsilon_y = lookahead_d_mm     (forward distance = the look-ahead distance)
    //    theta     = lookahead_theta    (= atan2(epsilon_x, epsilon_y))
    //
    //  For turn-lane  (y(x) polynomial — sweep along X):
    //    epsilon_x = 0                  (vehicle aims through the turn centerline at x=0)
    //    epsilon_y = longitudinal_offset_mm  (forward distance remaining to turn lane)
    //    theta     = lookahead_theta    (heading angle derived from polynomial a1 at x=0)
    //
    void publish_control_error(const json& obj) {
        int label = obj.value("label", -1);

        double epsilon_x, epsilon_y, theta;

        if (label == 10) {
            // Turn-lane: x(0)=a0=longitudinal distance, lateral target = x=0
            epsilon_x = 0.0;
            epsilon_y = obj.value("longitudinal_offset_mm", 0.0);
            theta     = obj.value("lookahead_theta_rad",    0.0);
        } else {
            // main-lane or other-lane: look-ahead point on x(y) centerline
            epsilon_x = obj.value("lookahead_x_mm",     0.0);
            epsilon_y = obj.value("lookahead_d_mm",      0.0);
            theta     = obj.value("lookahead_theta_rad", 0.0);
        }

        json out;
        out["lane_state"]    = lane_state_name(state_);
        out["target_label"]  = label;
        out["epsilon_x_mm"]  = std::round(epsilon_x * 10.0) / 10.0;  // lateral deviation (mm)
        out["epsilon_y_mm"]  = std::round(epsilon_y * 10.0) / 10.0;  // longitudinal deviation (mm)
        out["theta_rad"]     = std::round(theta      * 1000.0) / 1000.0; // heading error (rad)
        // Supplementary info (useful for controller tuning / dashboard)
        out["curvature_inv_mm"] = obj.value("curvature_inv_mm", 0.0);
        out["lookahead_d_mm"]   = obj.value("lookahead_d_mm",   0.0);

        std_msgs::msg::String msg;
        msg.data = out.dump();
        control_error_pub_->publish(msg);
    }

    // ── Publish lane detection state ─────────────────────────────────────────
    void publish_lane_state(bool has_main, bool has_other, bool has_turn, bool has_stop) {
        json state_json;
        state_json["lane_state"]         = lane_state_name(state_);
        state_json["main_lane_detected"]  = has_main;
        state_json["other_lane_detected"] = has_other;
        state_json["turn_lane_detected"]  = has_turn;
        state_json["stop_line_detected"]  = has_stop;
        state_json["external_turn_cmd"]   = external_turn_cmd_;

        std_msgs::msg::String msg;
        msg.data = state_json.dump();
        lane_state_pub_->publish(msg);
    }

    // ── ROS2 interfaces ──────────────────────────────────────────────────────
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    control_error_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    lane_state_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetry_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;

    // ── State ────────────────────────────────────────────────────────────────
    LaneState state_           = LaneState::FOLLOW_MAIN;
    bool      external_turn_cmd_ = false;

    // ── Thresholds ───────────────────────────────────────────────────────────
    double turn_proximity_mm_ = 500.0;  // distance to arm turn transition
    double turn_done_mm_      = 200.0;  // past-turn threshold
    double theta_done_rad_    = 0.1;    // heading threshold for turn completion
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneErrorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
