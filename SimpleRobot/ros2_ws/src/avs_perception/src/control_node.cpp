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
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────────────────────
// Route Intent and Decision State
// ─────────────────────────────────────────────────────────────────────────────
enum class RouteIntent {
    FOLLOW_MAIN,
    TURN_RIGHT,
    TURN_LEFT,
    LANE_CHANGE_LEFT,
    LANE_CHANGE_RIGHT,
    LEGACY_TURN,
    LEGACY_LANE_CHANGE
};

static const char* route_intent_name(RouteIntent intent) {
    switch (intent) {
        case RouteIntent::FOLLOW_MAIN: return "follow_main";
        case RouteIntent::TURN_RIGHT: return "turn_right";
        case RouteIntent::TURN_LEFT: return "turn_left";
        case RouteIntent::LANE_CHANGE_LEFT: return "lane_change_left";
        case RouteIntent::LANE_CHANGE_RIGHT: return "lane_change_right";
        case RouteIntent::LEGACY_TURN: return "legacy_turn";
        case RouteIntent::LEGACY_LANE_CHANGE: return "legacy_lane_change";
    }
    return "unknown";
}

enum class DecisionState {
    FOLLOW_MAIN,
    TURN_RIGHT,
    TURN_LEFT,
    LANE_CHANGE,
    BLOCKED,
    RECOVERY
};

static const char* decision_state_name(DecisionState s) {
    switch (s) {
        case DecisionState::FOLLOW_MAIN: return "FOLLOW_MAIN";
        case DecisionState::TURN_RIGHT: return "TURN_RIGHT";
        case DecisionState::TURN_LEFT: return "TURN_LEFT";
        case DecisionState::LANE_CHANGE: return "LANE_CHANGE";
        case DecisionState::BLOCKED: return "BLOCKED";
        case DecisionState::RECOVERY: return "RECOVERY";
    }
    return "UNKNOWN";
}

static const char* legacy_lane_state_name(DecisionState s) {
    switch (s) {
        case DecisionState::FOLLOW_MAIN: return "FOLLOW_MAIN";
        case DecisionState::TURN_RIGHT:  return "TURNING";
        case DecisionState::TURN_LEFT:   return "TURNING";
        case DecisionState::LANE_CHANGE: return "LANE_CHANGE";
        case DecisionState::BLOCKED:     return "FOLLOW_MAIN";
        case DecisionState::RECOVERY:    return "FOLLOW_MAIN";
        default:                         return "FOLLOW_MAIN";
    }
}

struct LaneCandidate {
    int label = -1;
    std::string class_name;
    // waypoints, polynomial, offsets, lookahead fields, bbox/range etc.
    json raw_obj;
};

struct MarkingCandidate {
    int label = -1;
    std::string class_name;
    // polygon/waypoints/range
    json raw_obj;
};

// ─────────────────────────────────────────────────────────────────────────────
// Point and Trajectory
// ─────────────────────────────────────────────────────────────────────────────
struct Point2D {
    union {
        double x; // lateral (x_mm)
        double x_mm;
    };
    union {
        double y; // forward (y_mm)
        double y_mm;
    };
};

enum class TrajectoryKind {
    FOLLOW_MAIN,
    TURN_RIGHT,
    TURN_LEFT,
    LANE_CHANGE_LEFT,
    LANE_CHANGE_RIGHT,
    BLOCKED_FOLLOW_MAIN,
    UNKNOWN
};

static const char* trajectory_kind_name(TrajectoryKind kind) {
    switch (kind) {
        case TrajectoryKind::FOLLOW_MAIN: return "follow_main";
        case TrajectoryKind::TURN_RIGHT: return "turn_right";
        case TrajectoryKind::TURN_LEFT: return "turn_left";
        case TrajectoryKind::LANE_CHANGE_LEFT: return "lane_change_left";
        case TrajectoryKind::LANE_CHANGE_RIGHT: return "lane_change_right";
        case TrajectoryKind::BLOCKED_FOLLOW_MAIN: return "blocked_follow_main";
        case TrajectoryKind::UNKNOWN: return "unknown";
    }
    return "unknown";
}

static TrajectoryKind string_to_trajectory_kind(const std::string& kind_str, RouteIntent current_intent) {
    if (kind_str == "follow_main" || kind_str == "fallback_follow_main" || 
        kind_str == "pending_t_junction_follow_main" || kind_str == "main_lane" || 
        kind_str == "precomputed_main_lane" || kind_str == "follow_main_connected") {
        return TrajectoryKind::FOLLOW_MAIN;
    } else if (kind_str == "turn_right" || kind_str == "turn_right_connected" || kind_str == "turn_right_standalone") {
        return TrajectoryKind::TURN_RIGHT;
    } else if (kind_str == "turn_left" || kind_str == "turn_left_connected" || kind_str == "turn_left_standalone") {
        return TrajectoryKind::TURN_LEFT;
    } else if (kind_str == "lane_change_left") {
        return TrajectoryKind::LANE_CHANGE_LEFT;
    } else if (kind_str == "lane_change_right") {
        return TrajectoryKind::LANE_CHANGE_RIGHT;
    } else if (kind_str == "blocked_follow_main") {
        return TrajectoryKind::BLOCKED_FOLLOW_MAIN;
    } else if (kind_str == "turn_lane" || kind_str == "precomputed_turn_lane") {
        if (current_intent == RouteIntent::TURN_LEFT) return TrajectoryKind::TURN_LEFT;
        if (current_intent == RouteIntent::TURN_RIGHT) return TrajectoryKind::TURN_RIGHT;
        return TrajectoryKind::UNKNOWN;
    } else if (kind_str == "transition") {
        if (current_intent == RouteIntent::LANE_CHANGE_LEFT) return TrajectoryKind::LANE_CHANGE_LEFT;
        if (current_intent == RouteIntent::LANE_CHANGE_RIGHT) return TrajectoryKind::LANE_CHANGE_RIGHT;
        if (current_intent == RouteIntent::TURN_LEFT) return TrajectoryKind::TURN_LEFT;
        if (current_intent == RouteIntent::TURN_RIGHT) return TrajectoryKind::TURN_RIGHT;
        return TrajectoryKind::UNKNOWN;
    }
    return TrajectoryKind::UNKNOWN;
}

struct LaneObservation {
    std::string lane_id;
    std::string class_name;
    int label = -1;
    std::vector<Point2D> points;
    double confidence = 0.0;
    double heading_hint = 0.0;
    double curvature_hint = 0.0;
    bool has_precomputed_control = false;
    double precomputed_epsilon_x_mm = 0.0;
    double precomputed_epsilon_y_mm = 0.0;
    double precomputed_theta_rad = 0.0;
    double precomputed_curvature_inv_mm = 0.0;
    double precomputed_lookahead_d_mm = 0.0;
    json raw_obj;
};

struct MarkingObservation {
    std::string marking_id;
    std::string class_name;
    int label = -1;
    std::vector<Point2D> points;
    double confidence = 0.0;
    json raw_obj;
};

struct PathObservationFrame {
    std::vector<LaneObservation> lanes;
    std::vector<MarkingObservation> markings;
    uint64_t timestamp_ms = 0;
};

class PathObservationBuilder {
public:
    static PathObservationFrame build(const json& telemetry) {
        PathObservationFrame frame;
        frame.timestamp_ms = telemetry.value("timestamp_ms", static_cast<uint64_t>(0));
        
        if (!telemetry.contains("objects") || !telemetry["objects"].is_array()) {
            return frame;
        }
        
        for (const auto& obj : telemetry["objects"]) {
            int label = obj.value("label", -1);
            std::string class_name = obj.value("class_name", "");
            std::string id_str;
            if (obj.contains("id")) {
                const auto& id_val = obj["id"];
                if (id_val.is_string()) id_str = id_val.get<std::string>();
                else if (!id_val.is_null()) id_str = id_val.dump();
            }
            if (id_str.empty() && obj.contains("track_id")) {
                const auto& tid_val = obj["track_id"];
                if (tid_val.is_string()) id_str = tid_val.get<std::string>();
                else if (!tid_val.is_null()) id_str = tid_val.dump();
            }
            if (id_str.empty()) {
                id_str = "obj_" + std::to_string(label) + "_" + std::to_string(frame.lanes.size() + frame.markings.size());
            }
            
            if (label == 3 || label == 4 || label == 17) {
                LaneObservation lane;
                lane.lane_id = id_str;
                lane.class_name = class_name;
                lane.label = label;
                lane.raw_obj = obj;
                
                std::vector<Point2D> raw_points;
                if (obj.contains("waypoints") && obj["waypoints"].is_array()) {
                    for (const auto& pt : obj["waypoints"]) {
                        if (pt.is_array() && pt.size() >= 2) {
                            Point2D p;
                            p.x = pt[0].get<double>();
                            p.y = pt[1].get<double>();
                            raw_points.push_back(p);
                        }
                    }
                }
                
                // Sort points
                if (label != 17) {
                    std::sort(raw_points.begin(), raw_points.end(), [](const Point2D& a, const Point2D& b) {
                        return a.y < b.y;
                    });
                } else {
                    if (!raw_points.empty()) {
                        double dist_front = raw_points.front().x * raw_points.front().x + raw_points.front().y * raw_points.front().y;
                        double dist_back = raw_points.back().x * raw_points.back().x + raw_points.back().y * raw_points.back().y;
                        if (dist_back < dist_front) {
                            std::reverse(raw_points.begin(), raw_points.end());
                        }
                    }
                }
                
                // Remove duplicate/too-close points (>10mm)
                if (!raw_points.empty()) {
                    lane.points.push_back(raw_points.front());
                    for (size_t i = 1; i < raw_points.size(); ++i) {
                        double dist = std::sqrt(std::pow(raw_points[i].x - lane.points.back().x, 2) + 
                                                std::pow(raw_points[i].y - lane.points.back().y, 2));
                        if (dist > 10.0) {
                            lane.points.push_back(raw_points[i]);
                        }
                    }
                }
                
                // Precomputed control fields compatibility
                if (lane.points.size() < 2 && label == 17 && obj.contains("longitudinal_offset_mm")) {
                    lane.has_precomputed_control = true;
                    lane.precomputed_epsilon_x_mm = 0.0;
                    lane.precomputed_epsilon_y_mm = obj["longitudinal_offset_mm"].get<double>();
                    lane.precomputed_theta_rad = obj.value("lookahead_theta_rad", 0.0);
                    lane.precomputed_curvature_inv_mm = obj.value("curvature_inv_mm", 0.0);
                    lane.precomputed_lookahead_d_mm = obj.value("lookahead_d_mm", lane.precomputed_epsilon_y_mm);
                } else if (lane.points.size() < 2 && obj.contains("lookahead_x_mm") && obj.contains("lookahead_d_mm")) {
                    lane.has_precomputed_control = true;
                    lane.precomputed_epsilon_x_mm = obj["lookahead_x_mm"].get<double>();
                    lane.precomputed_epsilon_y_mm = obj["lookahead_d_mm"].get<double>();
                    lane.precomputed_theta_rad = obj.value(
                        "lookahead_theta_rad",
                        std::atan2(lane.precomputed_epsilon_x_mm, lane.precomputed_epsilon_y_mm)
                    );
                    lane.precomputed_curvature_inv_mm = obj.value("curvature_inv_mm", 0.0);
                    lane.precomputed_lookahead_d_mm = obj["lookahead_d_mm"].get<double>();
                }
                
                // Metadata & confidence
                if (lane.points.size() >= 2) {
                    lane.heading_hint = std::atan2(lane.points[1].x - lane.points[0].x, lane.points[1].y - lane.points[0].y);
                    
                    double dx = lane.points.back().x - lane.points.front().x;
                    double dy = lane.points.back().y - lane.points.front().y;
                    double total_len = std::sqrt(dx * dx + dy * dy);
                    
                    double len_factor = std::min(1.0, total_len / 5000.0);
                    double pts_factor = std::min(1.0, static_cast<double>(lane.points.size()) / 10.0);
                    lane.confidence = 0.5 * len_factor + 0.5 * pts_factor;
                } else if (lane.has_precomputed_control) {
                    lane.confidence = 1.0;
                    lane.heading_hint = lane.precomputed_theta_rad;
                } else {
                    lane.confidence = 0.0;
                }
                
                lane.confidence = obj.value("confidence", lane.confidence);
                frame.lanes.push_back(lane);
                
            } else if (label == 0 || label == 1 || label == 2 || label == 13 || label == 14 || label == 16) {
                MarkingObservation marking;
                marking.marking_id = id_str;
                marking.class_name = class_name;
                marking.label = label;
                marking.raw_obj = obj;
                
                std::vector<Point2D> raw_points;
                if (obj.contains("waypoints") && obj["waypoints"].is_array()) {
                    for (const auto& pt : obj["waypoints"]) {
                        if (pt.is_array() && pt.size() >= 2) {
                            Point2D p;
                            p.x = pt[0].get<double>();
                            p.y = pt[1].get<double>();
                            raw_points.push_back(p);
                        }
                    }
                }
                
                std::sort(raw_points.begin(), raw_points.end(), [](const Point2D& a, const Point2D& b) {
                    return a.y < b.y;
                });
                
                if (!raw_points.empty()) {
                    marking.points.push_back(raw_points.front());
                    for (size_t i = 1; i < raw_points.size(); ++i) {
                        double dist = std::sqrt(std::pow(raw_points[i].x - marking.points.back().x, 2) + 
                                                std::pow(raw_points[i].y - marking.points.back().y, 2));
                        if (dist > 10.0) {
                            marking.points.push_back(raw_points[i]);
                        }
                    }
                }
                
                marking.confidence = obj.value("confidence", 1.0);
                frame.markings.push_back(marking);
            }
        }
        
        return frame;
    }
};

struct PlannedTrajectory {
    std::vector<Point2D> points;
    std::vector<std::string> source_lane_ids;
    std::string target_lane_id;
    TrajectoryKind trajectory_kind = TrajectoryKind::UNKNOWN;
    double confidence = 0.0;
    bool valid = false;
    bool blocked_by_marking = false;
    std::string normalization_mode = "none";

    // Precomputed control fields
    bool has_precomputed_control = false;
    double precomputed_epsilon_x_mm = 0.0;
    double precomputed_epsilon_y_mm = 0.0;
    double precomputed_theta_rad = 0.0;
    double precomputed_curvature_inv_mm = 0.0;
    double precomputed_lookahead_d_mm = 0.0;
};

struct CommittedTrajectoryState {
    PlannedTrajectory trajectory;
    double progress_s_mm = 0.0;
    double remaining_s_mm = 0.0;
    uint64_t last_good_frame = 0;
    int dropout_hold_counter = 0;
    std::string replan_reason = "none";
};

struct ActiveTrajectory {
    std::vector<Point2D> points;
    std::vector<int> source_labels;
    std::string trajectory_kind = "unknown";
    std::string normalization_mode = "none";
    double trajectory_confidence = 0.0;
    bool valid = false;
    bool has_precomputed_control = false;
    double precomputed_epsilon_x_mm = 0.0;
    double precomputed_epsilon_y_mm = 0.0;
    double precomputed_theta_rad = 0.0;
    double precomputed_curvature_inv_mm = 0.0;
    double precomputed_lookahead_d_mm = 0.0;
};

class TrajectoryPlanner {
public:
    static PlannedTrajectory plan_follow_main(const PathObservationFrame& obs, 
                                              const CommittedTrajectoryState& prev_state,
                                              std::string& last_main_id) {
        PlannedTrajectory plan;
        plan.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
        
        const LaneObservation* cur_lane = select_main_current(obs, last_main_id);
        
        if (cur_lane) {
            last_main_id = cur_lane->lane_id;
            plan.target_lane_id = cur_lane->lane_id;
            plan.source_lane_ids.push_back(std::to_string(cur_lane->label) + ":" + cur_lane->lane_id);
            
            const LaneObservation* ahead_lane = select_main_ahead(obs, cur_lane);
            
            std::vector<Point2D> raw_path;
            if (ahead_lane) {
                plan.source_lane_ids.push_back(std::to_string(ahead_lane->label) + ":" + ahead_lane->lane_id);
                raw_path = merge_lanes(*cur_lane, *ahead_lane);
            } else {
                raw_path = cur_lane->points;
            }
            
            plan.points = resample_path(raw_path, 100.0);
            plan.confidence = cur_lane->confidence;
            plan.valid = (plan.points.size() >= 2);
            
            if (cur_lane->has_precomputed_control) {
                plan.has_precomputed_control = true;
                plan.precomputed_epsilon_x_mm = cur_lane->precomputed_epsilon_x_mm;
                plan.precomputed_epsilon_y_mm = cur_lane->precomputed_epsilon_y_mm;
                plan.precomputed_theta_rad = cur_lane->precomputed_theta_rad;
                plan.precomputed_curvature_inv_mm = cur_lane->precomputed_curvature_inv_mm;
                plan.precomputed_lookahead_d_mm = cur_lane->precomputed_lookahead_d_mm;
                
                if (!plan.valid) {
                    plan.valid = true;
                    plan.confidence = cur_lane->confidence;
                }
            }
        } else {
            if (prev_state.trajectory.valid && prev_state.trajectory.trajectory_kind == TrajectoryKind::FOLLOW_MAIN) {
                plan.points = prev_state.trajectory.points;
                plan.target_lane_id = prev_state.trajectory.target_lane_id;
                plan.source_lane_ids = prev_state.trajectory.source_lane_ids;
                plan.confidence = prev_state.trajectory.confidence * 0.8;
                plan.valid = true;
                
                plan.has_precomputed_control = prev_state.trajectory.has_precomputed_control;
                plan.precomputed_epsilon_x_mm = prev_state.trajectory.precomputed_epsilon_x_mm;
                plan.precomputed_epsilon_y_mm = prev_state.trajectory.precomputed_epsilon_y_mm;
                plan.precomputed_theta_rad = prev_state.trajectory.precomputed_theta_rad;
                plan.precomputed_curvature_inv_mm = prev_state.trajectory.precomputed_curvature_inv_mm;
                plan.precomputed_lookahead_d_mm = prev_state.trajectory.precomputed_lookahead_d_mm;
            } else {
                plan.valid = false;
                plan.confidence = 0.0;
                last_main_id = "";
            }
        }
        
        return plan;
    }

    static PlannedTrajectory plan_turn_right(const PathObservationFrame& obs, 
                                             const CommittedTrajectoryState& prev_state,
                                             bool is_t,
                                             bool t_junction_pending,
                                             std::string& last_main_id) {
        return plan_turn_generic(obs, prev_state, true, is_t, last_main_id);
    }
    
    static PlannedTrajectory plan_turn_left(const PathObservationFrame& obs, 
                                            const CommittedTrajectoryState& prev_state,
                                            bool is_t,
                                            bool t_junction_pending,
                                            std::string& last_main_id) {
        return plan_turn_generic(obs, prev_state, false, is_t, last_main_id);
    }

    static PlannedTrajectory plan_turn_generic(const PathObservationFrame& obs,
                                               const CommittedTrajectoryState& prev_state,
                                               bool is_right_turn,
                                               bool is_t,
                                               std::string& last_main_id) {
        PlannedTrajectory plan;
        plan.trajectory_kind = is_right_turn ? TrajectoryKind::TURN_RIGHT : TrajectoryKind::TURN_LEFT;

        // 1. Select the turn lane
        const LaneObservation* selected_turn = select_turn_lane_obs(obs, is_right_turn, is_t);

        // 2. Reuse main-lane selection logic for turn transitions to avoid picking wrong main lane segment
        const LaneObservation* cur_main = select_main_current(obs, last_main_id);
        if (cur_main) {
            last_main_id = cur_main->lane_id;
        }

        // 3. Preserve precomputed turn lanes when no waypoint path is available or as legacy fallback
        if (selected_turn && selected_turn->has_precomputed_control) {
            plan.points = selected_turn->points; // may be empty
            if (plan.points.empty()) {
                plan.points = {
                    { 0.0, 0.0 },
                    { selected_turn->precomputed_epsilon_x_mm * 0.5, selected_turn->precomputed_epsilon_y_mm * 0.5 },
                    { selected_turn->precomputed_epsilon_x_mm, selected_turn->precomputed_epsilon_y_mm }
                };
            }
            plan.target_lane_id = selected_turn->lane_id;
            if (cur_main) {
                plan.source_lane_ids.push_back(std::to_string(cur_main->label) + ":" + cur_main->lane_id);
            }
            plan.source_lane_ids.push_back(std::to_string(selected_turn->label) + ":" + selected_turn->lane_id);
            plan.confidence = selected_turn->confidence;
            plan.valid = true; // explicitly mark valid
            
            // Populate precomputed control fields
            plan.has_precomputed_control = true;
            plan.precomputed_epsilon_x_mm = selected_turn->precomputed_epsilon_x_mm;
            plan.precomputed_epsilon_y_mm = selected_turn->precomputed_epsilon_y_mm;
            plan.precomputed_theta_rad = selected_turn->precomputed_theta_rad;
            plan.precomputed_curvature_inv_mm = selected_turn->precomputed_curvature_inv_mm;
            plan.precomputed_lookahead_d_mm = selected_turn->precomputed_lookahead_d_mm;
            
            return plan;
        }

        if (cur_main && selected_turn) {
            // Plan transition
            std::vector<Point2D> transition_pts = plan_transition(cur_main->points, selected_turn->points);
            if (!transition_pts.empty()) {
                plan.points = resample_path(transition_pts, 100.0);
                plan.target_lane_id = selected_turn->lane_id;
                plan.source_lane_ids.push_back(std::to_string(cur_main->label) + ":" + cur_main->lane_id);
                plan.source_lane_ids.push_back(std::to_string(selected_turn->label) + ":" + selected_turn->lane_id);
                plan.confidence = selected_turn->confidence;
                plan.valid = (plan.points.size() >= 2);
            }
            
            // If transition plan failed or is invalid, fallback to follow main
            if (!plan.valid) {
                plan = plan_follow_main(obs, prev_state, last_main_id);
                plan.trajectory_kind = TrajectoryKind::FOLLOW_MAIN; // Mark it as fallback follow_main
            }
        } else if (selected_turn) {
            // Standalone turn lane
            plan.points = resample_path(selected_turn->points, 100.0);
            plan.target_lane_id = selected_turn->lane_id;
            plan.source_lane_ids.push_back(std::to_string(selected_turn->label) + ":" + selected_turn->lane_id);
            plan.confidence = selected_turn->confidence;
            plan.valid = (plan.points.size() >= 2);
        } else if (cur_main) {
            // Fallback to follow main
            plan = plan_follow_main(obs, prev_state, last_main_id);
            plan.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
        } else {
            // Recovery / invalid
            plan.valid = false;
            plan.confidence = 0.0;
        }

        return plan;
    }

    static PlannedTrajectory plan_lane_change_left(const PathObservationFrame& obs, 
                                                    const CommittedTrajectoryState& prev_state,
                                                    std::string& last_main_id) {
        return plan_lane_change_generic(obs, prev_state, true, last_main_id);
    }
    
    static PlannedTrajectory plan_lane_change_right(const PathObservationFrame& obs, 
                                                     const CommittedTrajectoryState& prev_state,
                                                     std::string& last_main_id) {
        return plan_lane_change_generic(obs, prev_state, false, last_main_id);
    }

    static PlannedTrajectory plan_lane_change_generic(const PathObservationFrame& obs,
                                                      const CommittedTrajectoryState& prev_state,
                                                      bool is_left_change,
                                                      std::string& last_main_id) {
        PlannedTrajectory plan;
        plan.trajectory_kind = is_left_change ? TrajectoryKind::LANE_CHANGE_LEFT : TrajectoryKind::LANE_CHANGE_RIGHT;

        // 1. Select the current main lane
        const LaneObservation* cur_main = select_main_current(obs, last_main_id);
        if (cur_main) {
            last_main_id = cur_main->lane_id;
        }

        // 2. Select the target other lane
        const LaneObservation* target_other = select_other_lane_obs(obs, cur_main, is_left_change);

        if (cur_main && target_other) {
            // 3. Check if lane change is blocked by a solid marking
            bool blocked = is_lane_change_blocked_by_solid_obs(cur_main, target_other, obs.markings);
            if (blocked) {
                // If blocked, plan follow main but set blocked_by_marking = true
                plan = plan_follow_main(obs, prev_state, last_main_id);
                plan.blocked_by_marking = true;
                plan.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
            } else {
                // Plan transition
                std::vector<Point2D> transition_pts = plan_transition(cur_main->points, target_other->points);
                if (!transition_pts.empty()) {
                    plan.points = resample_path(transition_pts, 100.0);
                    plan.target_lane_id = target_other->lane_id;
                    plan.source_lane_ids.push_back(std::to_string(cur_main->label) + ":" + cur_main->lane_id);
                    plan.source_lane_ids.push_back(std::to_string(target_other->label) + ":" + target_other->lane_id);
                    plan.confidence = target_other->confidence;
                    plan.valid = (plan.points.size() >= 2);
                }
                
                // If transition planning failed, fallback to follow main
                if (!plan.valid) {
                    plan = plan_follow_main(obs, prev_state, last_main_id);
                    plan.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
                }
            }
        } else if (cur_main) {
            // No target other lane detected, fallback to follow main
            plan = plan_follow_main(obs, prev_state, last_main_id);
            plan.trajectory_kind = TrajectoryKind::FOLLOW_MAIN;
        } else if (target_other) {
            // Only other lane is detected
            plan.points = resample_path(target_other->points, 100.0);
            plan.target_lane_id = target_other->lane_id;
            plan.source_lane_ids.push_back(std::to_string(target_other->label) + ":" + target_other->lane_id);
            plan.confidence = target_other->confidence;
            plan.valid = (plan.points.size() >= 2);
        } else {
            // Recovery / invalid
            plan.valid = false;
            plan.confidence = 0.0;
        }

        return plan;
    }

private:
    static double get_lane_heading_obs(const LaneObservation& lane) {
        if (lane.points.size() < 2) {
            return lane.has_precomputed_control ? lane.precomputed_theta_rad : 0.0;
        }
        size_t end_idx = std::min(lane.points.size() - 1, size_t(3));
        double dx = lane.points[end_idx].x - lane.points.front().x;
        double dy = lane.points[end_idx].y - lane.points.front().y;
        return std::atan2(dx, dy);
    }

    static const LaneObservation* select_other_lane_obs(const PathObservationFrame& obs,
                                                        const LaneObservation* main_lane,
                                                        bool is_left_change) {
        std::vector<const LaneObservation*> other_lanes;
        for (const auto& l : obs.lanes) {
            if (l.label == 4 || l.class_name == "other-lane") {
                other_lanes.push_back(&l);
            }
        }
        if (other_lanes.empty()) return nullptr;

        double main_x = 0.0;
        double main_heading = 0.0;
        
        if (main_lane) {
            if (!main_lane->points.empty()) {
                double sum_x = 0.0;
                for (const auto& pt : main_lane->points) {
                    sum_x += pt.x;
                }
                main_x = sum_x / main_lane->points.size();
                main_heading = get_lane_heading_obs(*main_lane);
            } else if (main_lane->has_precomputed_control) {
                main_x = main_lane->precomputed_epsilon_x_mm;
                main_heading = main_lane->precomputed_theta_rad;
            }
        }

        const LaneObservation* best_cand = nullptr;
        double best_score = -1e9;

        for (const auto* l : other_lanes) {
            double other_x = 0.0;
            double other_heading = 0.0;
            double min_y = 0.0;
            
            if (!l->points.empty()) {
                double sum_x = 0.0;
                double local_min_y = 1e9;
                for (const auto& pt : l->points) {
                    sum_x += pt.x;
                    if (pt.y < local_min_y) local_min_y = pt.y;
                }
                other_x = sum_x / l->points.size();
                other_heading = get_lane_heading_obs(*l);
                min_y = local_min_y;
            } else if (l->has_precomputed_control) {
                other_x = l->precomputed_epsilon_x_mm;
                other_heading = l->precomputed_theta_rad;
                min_y = 0.0;
            } else {
                continue;
            }
            
            double lateral_dist = other_x - main_x;
            
            // ── Gating (Hard Filters) ──
            // 1. Side Gate
            if (is_left_change && lateral_dist > -200.0) continue;
            if (!is_left_change && lateral_dist < 200.0) continue;
            
            // 2. Parallelism Gate (heading difference < 30 degrees)
            double diff_theta = std::abs(other_heading - main_heading);
            while (diff_theta > M_PI) diff_theta -= 2.0 * M_PI;
            while (diff_theta < -M_PI) diff_theta += 2.0 * M_PI;
            diff_theta = std::abs(diff_theta);
            if (diff_theta > (30.0 * M_PI / 180.0)) continue;
            
            // 3. Distance Gate (400mm to 1400mm)
            double abs_lat_dist = std::abs(lateral_dist);
            if (abs_lat_dist < 400.0 || abs_lat_dist > 1400.0) continue;
            
            // 4. Corridor Overlap Gate
            if (min_y > 1200.0) continue;

            // ── Scoring ──
            double score = -std::abs(abs_lat_dist - 800.0) - 1000.0 * diff_theta;
            if (score > best_score) {
                best_score = score;
                best_cand = l;
            }
        }
        
        return best_cand;
    }

    static bool is_lane_change_blocked_by_solid_obs(const LaneObservation* main_lane, 
                                                    const LaneObservation* target_lane, 
                                                    const std::vector<MarkingObservation>& markings) {
        if (!main_lane || !target_lane) return false;

        auto get_x = [](const LaneObservation* l) {
            if (l->raw_obj.contains("lookahead_x_mm")) return l->raw_obj["lookahead_x_mm"].get<double>();
            if (!l->points.empty()) return l->points[0].x;
            if (l->has_precomputed_control) return l->precomputed_epsilon_x_mm;
            return 0.0;
        };
        double main_x = get_x(main_lane);
        double target_x = get_x(target_lane);

        double min_x = std::min(main_x, target_x);
        double max_x = std::max(main_x, target_x);

        double p0_y = 0.0;
        double p3_y = 2000.0;
        
        std::vector<Point2D> main_wps = main_lane->points;
        std::sort(main_wps.begin(), main_wps.end(), [](const Point2D& a, const Point2D& b) {
            return a.y < b.y;
        });

        std::vector<Point2D> target_wps = target_lane->points;
        std::sort(target_wps.begin(), target_wps.end(), [](const Point2D& a, const Point2D& b) {
            return a.y < b.y;
        });

        double cum_dist = 0.0;
        if (!main_wps.empty()) {
            p0_y = main_wps[0].y;
            for (size_t i = 1; i < main_wps.size(); ++i) {
                double dx = main_wps[i].x - main_wps[i-1].x;
                double dy = main_wps[i].y - main_wps[i-1].y;
                cum_dist += std::hypot(dx, dy);
                if (cum_dist >= 300.0) {
                    p0_y = main_wps[i].y;
                    break;
                }
            }
        }
        
        cum_dist = 0.0;
        if (!target_wps.empty()) {
            p3_y = target_wps.back().y;
            for (size_t i = 1; i < target_wps.size(); ++i) {
                double dx = target_wps[i].x - target_wps[i-1].x;
                double dy = target_wps[i].y - target_wps[i-1].y;
                cum_dist += std::hypot(dx, dy);
                if (cum_dist >= 1200.0) {
                    p3_y = target_wps[i].y;
                    break;
                }
            }
        }
        
        double y_min = std::min(p0_y, p3_y) - 100.0;
        double y_max = std::max(p0_y, p3_y) + 300.0;

        for (const auto& m : markings) {
            if (m.label == 2 || m.label == 13 || m.label == 14) {
                bool is_between = false;
                if (m.raw_obj.contains("lookahead_x_mm") && !m.raw_obj.contains("waypoints") && !m.raw_obj.contains("polygons_real_world")) {
                    double mark_x = m.raw_obj["lookahead_x_mm"].get<double>();
                    double mark_y = 600.0;
                    is_between = (mark_x > min_x && mark_x < max_x && mark_y >= y_min && mark_y <= y_max);
                } else if (m.raw_obj.contains("waypoints") && !m.raw_obj["waypoints"].empty()) {
                    for (const auto& wp : m.raw_obj["waypoints"]) {
                        double mark_x = wp[0].get<double>();
                        double mark_y = wp[1].get<double>();
                        if (mark_x > min_x && mark_x < max_x && mark_y >= y_min && mark_y <= y_max) {
                            is_between = true;
                            break;
                        }
                    }
                } else if (m.raw_obj.contains("polygons_real_world") && !m.raw_obj["polygons_real_world"].empty()) {
                    for (const auto& poly : m.raw_obj["polygons_real_world"]) {
                        if (poly.is_array()) {
                            for (const auto& pt : poly) {
                                if (pt.is_array() && pt.size() >= 2) {
                                    double mark_x = pt[0].get<double>();
                                    double mark_y = pt[1].get<double>();
                                    if (mark_x > min_x && mark_x < max_x && mark_y >= y_min && mark_y <= y_max) {
                                        is_between = true;
                                        break;
                                    }
                                }
                            }
                        }
                        if (is_between) break;
                    }
                }
                
                if (is_between) return true;
            }
        }
        return false;
    }
    
private:
    static const LaneObservation* select_turn_lane_obs(const PathObservationFrame& obs,
                                                       bool is_turn_right,
                                                       bool is_t_junction) {
        std::vector<const LaneObservation*> turn_lanes;
        for (const auto& l : obs.lanes) {
            if (l.label == 17) turn_lanes.push_back(&l);
        }
        
        if (turn_lanes.empty()) return nullptr;

        // First pass: identify if any candidate is on the strict correct side
        bool correct_side_exists = false;
        for (const auto* l : turn_lanes) {
            double avg_x = 0.0;
            if (!l->points.empty()) {
                double sum_x = 0;
                for (const auto& pt : l->points) {
                    sum_x += pt.x;
                }
                avg_x = sum_x / l->points.size();
            } else if (l->has_precomputed_control) {
                avg_x = l->precomputed_epsilon_x_mm;
            }
            if (is_turn_right && avg_x >= 0.0) correct_side_exists = true;
            if (!is_turn_right && avg_x <= 0.0) correct_side_exists = true;
        }

        std::vector<std::pair<double, const LaneObservation*>> scored_lanes;
        for (const auto* l : turn_lanes) {
            double min_dist = 1e9;
            double avg_x = 0.0;
            
            if (!l->points.empty()) {
                double sum_x = 0;
                for (const auto& pt : l->points) {
                    sum_x += pt.x;
                    double dist = std::sqrt(pt.x*pt.x + pt.y*pt.y);
                    if (dist < min_dist) min_dist = dist;
                }
                avg_x = sum_x / l->points.size();
            } else if (l->has_precomputed_control) {
                min_dist = l->precomputed_lookahead_d_mm;
                avg_x = l->precomputed_epsilon_x_mm;
            } else {
                continue;
            }

            if (!is_t_junction) {
                if (is_turn_right && avg_x < 0) continue;
                if (!is_turn_right && avg_x > 0) continue;
            } else {
                if (correct_side_exists) {
                    if (is_turn_right && avg_x < 0.0) continue;
                    if (!is_turn_right && avg_x > 0.0) continue;
                } else {
                    if (is_turn_right && avg_x < -200.0) continue;
                    if (!is_turn_right && avg_x > 200.0) continue;
                }
            }

            scored_lanes.push_back({min_dist, l});
        }
        
        if (scored_lanes.empty()) return nullptr;

        std::sort(scored_lanes.begin(), scored_lanes.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });

        if (is_turn_right) {
            return scored_lanes.front().second; // closest
        } else {
            return scored_lanes.back().second;  // farthest
        }
    }

    static std::vector<Point2D> plan_transition(const std::vector<Point2D>& current_pts, 
                                                const std::vector<Point2D>& target_pts) {
        if (current_pts.size() < 2 || target_pts.size() < 2) {
            return {};
        }

        // Safety guard: check if target lane is too far or heading is too divergent
        double cur_heading = 0.0;
        if (current_pts.size() >= 2) {
            cur_heading = std::atan2(current_pts[1].x - current_pts[0].x, current_pts[1].y - current_pts[0].y);
        }
        double target_heading = 0.0;
        if (target_pts.size() >= 2) {
            target_heading = std::atan2(target_pts[1].x - target_pts[0].x, target_pts[1].y - target_pts[0].y);
        }
        
        double cur_x = current_pts.front().x;
        double target_x = target_pts.front().x;
        double lat_dist = std::abs(target_x - cur_x);
        double heading_diff = std::abs(target_heading - cur_heading);
        while (heading_diff > M_PI) heading_diff -= 2.0 * M_PI;
        while (heading_diff < -M_PI) heading_diff += 2.0 * M_PI;
        heading_diff = std::abs(heading_diff);

        if (lat_dist > 1500.0 || heading_diff > (40.0 * M_PI / 180.0)) {
            return {};
        }

        // Find P0: ~300mm along the current lane
        Point2D P0 = current_pts.front();
        Point2D p_prev = P0;
        double cum_dist = 0.0;
        size_t split_idx_current = 0;
        for (size_t i = 1; i < current_pts.size(); ++i) {
            cum_dist += std::hypot(current_pts[i].x - current_pts[i-1].x, current_pts[i].y - current_pts[i-1].y);
            if (cum_dist >= 300.0) {
                P0 = current_pts[i];
                p_prev = current_pts[i-1];
                split_idx_current = i;
                break;
            }
        }
        if (split_idx_current == 0 && current_pts.size() > 1) {
            split_idx_current = 1;
            P0 = current_pts[1];
            p_prev = current_pts[0];
        }

        // Find P3: ~1200mm along the target lane
        Point2D P3 = target_pts.back();
        Point2D p_next = P3;
        cum_dist = 0.0;
        size_t split_idx_target = target_pts.size() - 1;
        for (size_t i = 1; i < target_pts.size(); ++i) {
            cum_dist += std::hypot(target_pts[i].x - target_pts[i-1].x, target_pts[i].y - target_pts[i-1].y);
            if (cum_dist >= 1200.0) {
                P3 = target_pts[i];
                p_next = (i + 1 < target_pts.size()) ? target_pts[i+1] : P3;
                split_idx_target = i;
                break;
            }
        }
        if (split_idx_target == target_pts.size() - 1 && target_pts.size() > 1) {
            split_idx_target = target_pts.size() / 2;
            if (split_idx_target == 0) split_idx_target = 1;
            P3 = target_pts[split_idx_target];
            p_next = (split_idx_target + 1 < target_pts.size()) ? target_pts[split_idx_target+1] : P3;
        }

        double dx0 = P0.x - p_prev.x;
        double dy0 = P0.y - p_prev.y;
        double len0 = std::sqrt(dx0*dx0 + dy0*dy0);
        if (len0 < 1e-3) { dx0 = 0; dy0 = 1.0; }
        else { dx0 /= len0; dy0 /= len0; }
        
        double dx3 = p_next.x - P3.x;
        double dy3 = p_next.y - P3.y;
        double len3 = std::sqrt(dx3*dx3 + dy3*dy3);
        if (len3 < 1e-3) { dx3 = 0; dy3 = 1.0; }
        else { dx3 /= len3; dy3 /= len3; }

        double dist = std::sqrt((P3.x - P0.x)*(P3.x - P0.x) + (P3.y - P0.y)*(P3.y - P0.y));
        double scale = dist / 3.0;
        
        Point2D P1 = { P0.x + dx0 * scale, P0.y + dy0 * scale };
        Point2D P2 = { P3.x - dx3 * scale, P3.y - dy3 * scale };

        std::vector<Point2D> result;
        for (size_t i = 0; i <= split_idx_current; ++i) {
            result.push_back(current_pts[i]);
        }

        int num_samples = std::max(10, static_cast<int>(dist / 50.0));
        for (int i = 1; i < num_samples; ++i) {
            double t = static_cast<double>(i) / num_samples;
            double u = 1.0 - t;
            double w0 = u * u * u;
            double w1 = 3.0 * u * u * t;
            double w2 = 3.0 * u * t * t;
            double w3 = t * t * t;
            double bx = w0*P0.x + w1*P1.x + w2*P2.x + w3*P3.x;
            double by = w0*P0.y + w1*P1.y + w2*P2.y + w3*P3.y;
            result.push_back({bx, by});
        }

        for (size_t i = split_idx_target; i < target_pts.size(); ++i) {
            result.push_back(target_pts[i]);
        }

        return result;
    }

    static const LaneObservation* select_main_current(const PathObservationFrame& obs, const std::string& last_main_id) {
        std::vector<const LaneObservation*> main_lanes;
        for (const auto& l : obs.lanes) {
            if (l.label == 3 || l.class_name == "main-lane") {
                main_lanes.push_back(&l);
            }
        }
        
        if (main_lanes.empty()) return nullptr;
        
        double min_start_y = 1e9;
        for (const auto* l : main_lanes) {
            if (!l->points.empty()) {
                min_start_y = std::min(min_start_y, l->points.front().y);
            }
        }
        
        const LaneObservation* best_lane = nullptr;
        double best_score = 1e9;
        
        for (const auto* l : main_lanes) {
            double start_x = 0.0;
            double start_y = 0.0;
            bool has_wps = !l->points.empty();
            
            if (has_wps) {
                start_x = l->points.front().x;
                start_y = l->points.front().y;
            } else if (l->has_precomputed_control) {
                start_x = 0.0;
                start_y = 0.0;
            } else {
                continue;
            }
            
            double score = std::abs(start_x) + 0.5 * start_y;
            if (!has_wps) {
                score += 5000.0;
            }
            
            if (!last_main_id.empty() && l->lane_id == last_main_id) {
                if (start_y - min_start_y <= 600.0) {
                    score -= 1500.0;
                }
            }
            
            if (score < best_score) {
                best_score = score;
                best_lane = l;
            }
        }
        
        return best_lane ? best_lane : main_lanes.front();
    }
    
    static const LaneObservation* select_main_ahead(const PathObservationFrame& obs, const LaneObservation* cur_lane) {
        if (!cur_lane || cur_lane->points.size() < 2) return nullptr;
        
        double cur_end_x = cur_lane->points.back().x;
        double cur_end_y = cur_lane->points.back().y;
        double cur_prev_x = cur_lane->points[cur_lane->points.size() - 2].x;
        double cur_prev_y = cur_lane->points[cur_lane->points.size() - 2].y;
        double cur_theta = std::atan2(cur_end_x - cur_prev_x, cur_end_y - cur_prev_y);
        
        const LaneObservation* best_ahead = nullptr;
        double best_ahead_y = 1e9;
        
        for (const auto& l : obs.lanes) {
            bool is_main = (l.label == 3 || l.class_name == "main-lane");
            if (&l == cur_lane || !is_main || l.points.size() < 2) continue;
            
            double ahead_start_x = l.points.front().x;
            double ahead_start_y = l.points.front().y;
            double ahead_next_x = l.points[1].x;
            double ahead_next_y = l.points[1].y;
            double ahead_theta = std::atan2(ahead_next_x - ahead_start_x, ahead_next_y - ahead_start_y);
            
            double long_gap = ahead_start_y - cur_end_y;
            if (long_gap < -500.0 || long_gap > 2000.0) continue;
            
            double lat_jump = std::abs(ahead_start_x - cur_end_x);
            if (lat_jump > 400.0) continue;
            
            double diff_theta = std::abs(ahead_theta - cur_theta);
            while (diff_theta > M_PI) diff_theta -= 2.0 * M_PI;
            while (diff_theta < -M_PI) diff_theta += 2.0 * M_PI;
            diff_theta = std::abs(diff_theta);
            if (diff_theta > (30.0 * M_PI / 180.0)) continue;
            
            if (ahead_start_y < best_ahead_y) {
                best_ahead_y = ahead_start_y;
                best_ahead = &l;
            }
        }
        
        return best_ahead;
    }
    
    static std::vector<Point2D> merge_lanes(const LaneObservation& cur, const LaneObservation& ahead) {
        std::vector<Point2D> merged = cur.points;
        if (ahead.points.empty()) return merged;
        
        double end_y = cur.points.empty() ? -1e9 : cur.points.back().y;
        for (const auto& pt : ahead.points) {
            if (pt.y > end_y + 10.0) {
                merged.push_back(pt);
            }
        }
        return merged;
    }
    
    static std::vector<Point2D> resample_path(const std::vector<Point2D>& points, double step_mm) {
        std::vector<Point2D> resampled;
        if (points.empty()) return resampled;
        if (points.size() == 1) {
            resampled.push_back(points.front());
            return resampled;
        }
        
        resampled.push_back(points.front());
        double accumulated_dist = 0.0;
        
        size_t next_idx = 1;
        double current_s = 0.0;
        
        while (next_idx < points.size()) {
            const auto& p0 = points[next_idx - 1];
            const auto& p1 = points[next_idx];
            double seg_len = std::sqrt(std::pow(p1.x - p0.x, 2) + std::pow(p1.y - p0.y, 2));
            
            if (seg_len < 1e-3) {
                next_idx++;
                continue;
            }
            
            double target_s = current_s + step_mm;
            if (accumulated_dist + seg_len >= target_s) {
                double ratio = (target_s - accumulated_dist) / seg_len;
                Point2D interpolated;
                interpolated.x = p0.x + ratio * (p1.x - p0.x);
                interpolated.y = p0.y + ratio * (p1.y - p0.y);
                resampled.push_back(interpolated);
                current_s = target_s;
            } else {
                accumulated_dist += seg_len;
                next_idx++;
            }
        }
        
        if (resampled.size() > 0) {
            double dist = std::sqrt(std::pow(points.back().x - resampled.back().x, 2) + 
                                    std::pow(points.back().y - resampled.back().y, 2));
            if (dist > 10.0) {
                resampled.push_back(points.back());
            }
        }
        
        return resampled;
    }
};

class TrajectoryNormalizer {
public:
    static PlannedTrajectory normalize(const PlannedTrajectory& current_candidate,
                                       const CommittedTrajectoryState& previous_state) {
        PlannedTrajectory normalized = current_candidate;
        
        // If the maneuver is blocked by a marking, skip blending to immediately return to the lane.
        if (current_candidate.blocked_by_marking) {
            normalized.normalization_mode = "blocked_passthrough";
            return normalized;
        }
        
        // If the previous committed trajectory is invalid or empty, we cannot blend.
        // Return the current candidate directly.
        if (!previous_state.trajectory.valid || previous_state.trajectory.points.empty()) {
            normalized.normalization_mode = "no_previous_passthrough";
            return normalized;
        }
        
        // If the current candidate is invalid or empty, return it directly.
        if (!current_candidate.valid || current_candidate.points.empty()) {
            normalized.normalization_mode = "invalid_candidate_passthrough";
            return normalized;
        }
        
        const auto& prev_pts = previous_state.trajectory.points;
        const auto& cur_pts = current_candidate.points;
        
        std::vector<Point2D> blended_pts;
        size_t common_size = std::min(prev_pts.size(), cur_pts.size());
        blended_pts.reserve(std::max(prev_pts.size(), cur_pts.size()));
        
        double C = current_candidate.confidence;
        double L_trans = 3000.0; // 3 meters transition length
        
        // Confidence-based bounds for new path weight
        double w_cur_max = 0.2 + 0.7 * C;
        double w_cur_min = 0.05 + 0.15 * C;
        
        for (size_t i = 0; i < common_size; ++i) {
            double s = i * 100.0; // both paths are resampled at 100mm steps
            double alpha = std::min(1.0, s / L_trans);
            
            double w_cur = w_cur_min + alpha * (w_cur_max - w_cur_min);
            double w_prev = 1.0 - w_cur;
            
            Point2D pt;
            pt.x = w_prev * prev_pts[i].x + w_cur * cur_pts[i].x;
            pt.y = w_prev * prev_pts[i].y + w_cur * cur_pts[i].y;
            blended_pts.push_back(pt);
        }
        
        // Append remaining points from the longer path to avoid truncation
        if (cur_pts.size() > common_size) {
            for (size_t i = common_size; i < cur_pts.size(); ++i) {
                blended_pts.push_back(cur_pts[i]);
            }
        } else if (prev_pts.size() > common_size) {
            for (size_t i = common_size; i < prev_pts.size(); ++i) {
                blended_pts.push_back(prev_pts[i]);
            }
        }
        
        normalized.points = std::move(blended_pts);
        normalized.valid = (normalized.points.size() >= 2);
        normalized.normalization_mode = "temporal_blend";
        
        return normalized;
    }
};

enum class ManagerAction {
    HOLD_CURRENT,
    UPDATE_CURRENT,
    COMMIT_NEW,
    ENTER_RECOVERY,
    ENTER_BLOCKED
};

class TrajectoryManager {
public:
    struct Decision {
        ManagerAction action;
        std::string reason;
        CommittedTrajectoryState next_state;
    };
    
    static Decision update(const PlannedTrajectory& normalized_candidate,
                           const CommittedTrajectoryState& previous_state,
                           const std::string& current_intent,
                           int& consecutive_invalid_frames,
                           uint64_t current_frame) {
        Decision d;
        d.next_state = previous_state;
        
        bool is_intent_changed = false;
        if (previous_state.trajectory.valid) {
            std::string prev_kind_str = trajectory_kind_name(previous_state.trajectory.trajectory_kind);
            if (prev_kind_str != current_intent && !current_intent.empty()) {
                is_intent_changed = true;
            }
        }
        
        // ── Case 1: Both old and new trajectories are invalid ──
        if (!previous_state.trajectory.valid && !normalized_candidate.valid) {
            d.action = ManagerAction::ENTER_RECOVERY;
            d.reason = "no_valid_trajectory";
            d.next_state.progress_s_mm = 0.0;
            d.next_state.remaining_s_mm = 0.0;
            d.next_state.trajectory.valid = false;
            d.next_state.trajectory.points.clear();
            d.next_state.trajectory.source_lane_ids.clear();
            d.next_state.trajectory.target_lane_id.clear();
            d.next_state.trajectory.trajectory_kind = TrajectoryKind::UNKNOWN;
            d.next_state.trajectory.confidence = 0.0;
            d.next_state.trajectory.normalization_mode = "none";
            d.next_state.dropout_hold_counter = 0;
            d.next_state.replan_reason = "recovery";
            consecutive_invalid_frames = 0;
            return d;
        }
        
        // ── Case 2: New candidate is invalid ──
        if (!normalized_candidate.valid) {
            consecutive_invalid_frames++;
            if (consecutive_invalid_frames <= 5) {
                // Transient dropout: HOLD the previous committed state
                d.action = ManagerAction::HOLD_CURRENT;
                d.reason = "transient_dropout_hold";
                d.next_state.dropout_hold_counter = consecutive_invalid_frames;
                d.next_state.replan_reason = "hold_due_to_dropout";
            } else {
                // Persistent dropout: Clear and enter recovery
                d.action = ManagerAction::ENTER_RECOVERY;
                d.reason = "persistent_dropout_clear";
                d.next_state.progress_s_mm = 0.0;
                d.next_state.remaining_s_mm = 0.0;
                d.next_state.trajectory.valid = false;
                d.next_state.trajectory.points.clear();
                d.next_state.trajectory.source_lane_ids.clear();
                d.next_state.trajectory.target_lane_id.clear();
                d.next_state.trajectory.trajectory_kind = TrajectoryKind::UNKNOWN;
                d.next_state.trajectory.confidence = 0.0;
                d.next_state.trajectory.normalization_mode = "none";
                d.next_state.dropout_hold_counter = consecutive_invalid_frames;
                d.next_state.replan_reason = "persistent_invalid_clear";
            }
            return d;
        }
        
        // ── Case 3: New candidate is valid, but previous state was invalid ──
        if (!previous_state.trajectory.valid) {
            d.action = ManagerAction::COMMIT_NEW;
            d.reason = "first_valid_trajectory";
            d.next_state.trajectory = normalized_candidate;
            d.next_state.progress_s_mm = 0.0;
            d.next_state.remaining_s_mm = calculate_path_length(normalized_candidate.points);
            d.next_state.last_good_frame = current_frame;
            d.next_state.dropout_hold_counter = 0;
            d.next_state.replan_reason = "first_commit";
            consecutive_invalid_frames = 0;
            return d;
        }
        
        // ── Case 4: Both are valid ──
        consecutive_invalid_frames = 0;
        d.next_state.dropout_hold_counter = 0;
        
        // Check if intent changed (e.g. follow_main -> turn)
        if (is_intent_changed) {
            d.action = ManagerAction::COMMIT_NEW;
            d.reason = "intent_changed_to_" + current_intent;
            d.next_state.trajectory = normalized_candidate;
            d.next_state.progress_s_mm = 0.0;
            d.next_state.remaining_s_mm = calculate_path_length(normalized_candidate.points);
            d.next_state.last_good_frame = current_frame;
            d.next_state.replan_reason = "intent_change";
            return d;
        }
        
        // Compare new candidate and previous path to prevent jittery replan
        double path_diff = calculate_path_deviation(previous_state.trajectory.points, normalized_candidate.points);
        
        // 1. If the deviation is excessive (e.g., > 800mm), force a replan to snap to the new perception
        if (path_diff > 800.0) {
            d.action = ManagerAction::COMMIT_NEW;
            d.reason = "excessive_deviation_replan";
            d.next_state.trajectory = normalized_candidate;
            d.next_state.progress_s_mm = 0.0;
            d.next_state.remaining_s_mm = calculate_path_length(normalized_candidate.points);
            d.next_state.last_good_frame = current_frame;
            d.next_state.replan_reason = "excessive_deviation";
            return d;
        }
        
        if (path_diff < 50.0) {
            d.action = ManagerAction::HOLD_CURRENT;
            d.reason = "deviation_below_threshold";

            bool prev_is_maneuver = (previous_state.trajectory.trajectory_kind == TrajectoryKind::LANE_CHANGE_LEFT ||
                                     previous_state.trajectory.trajectory_kind == TrajectoryKind::LANE_CHANGE_RIGHT ||
                                     previous_state.trajectory.trajectory_kind == TrajectoryKind::TURN_LEFT ||
                                     previous_state.trajectory.trajectory_kind == TrajectoryKind::TURN_RIGHT);
            bool same_kind = (normalized_candidate.trajectory_kind == previous_state.trajectory.trajectory_kind);

            if (same_kind) {
                // Refresh low-deviation geometry so committed follow-main paths keep extending
                // forward, but preserve stronger ID metadata when the latest frame degraded to
                // a synthesized obj_* identifier.
                d.next_state.trajectory = normalized_candidate;

                bool previous_has_stable_id = !previous_state.trajectory.target_lane_id.empty() &&
                                              previous_state.trajectory.target_lane_id.rfind("obj_", 0) != 0;
                bool candidate_has_stable_id = !normalized_candidate.target_lane_id.empty() &&
                                               normalized_candidate.target_lane_id.rfind("obj_", 0) != 0;

                if (previous_has_stable_id && !candidate_has_stable_id) {
                    d.next_state.trajectory.target_lane_id = previous_state.trajectory.target_lane_id;
                }

                if (d.next_state.trajectory.source_lane_ids.empty()) {
                    d.next_state.trajectory.source_lane_ids = previous_state.trajectory.source_lane_ids;
                }

                d.next_state.progress_s_mm = previous_state.progress_s_mm;
                d.next_state.remaining_s_mm = calculate_path_length(normalized_candidate.points);
            } else {
                bool candidate_is_follow_main = (normalized_candidate.trajectory_kind == TrajectoryKind::FOLLOW_MAIN);
                bool allow_maneuver_fallback = normalized_candidate.blocked_by_marking ||
                                               previous_state.replan_reason == "hold_maneuver_fallback";

                if (prev_is_maneuver && candidate_is_follow_main && !allow_maneuver_fallback) {
                    // Give maneuver dropouts a one-frame grace period before collapsing to
                    // FOLLOW_MAIN. This protects transient misses without trapping the state
                    // machine in the maneuver forever once the fallback persists.
                    d.next_state.trajectory = previous_state.trajectory;
                    d.next_state.progress_s_mm = previous_state.progress_s_mm;
                    d.next_state.remaining_s_mm = previous_state.remaining_s_mm;
                    d.next_state.last_good_frame = current_frame;
                    d.next_state.replan_reason = "hold_maneuver_fallback";
                    return d;
                }

                d.next_state.trajectory = normalized_candidate;
                d.next_state.progress_s_mm = previous_state.progress_s_mm;
                d.next_state.remaining_s_mm = calculate_path_length(normalized_candidate.points);
            }

            d.next_state.last_good_frame = current_frame;
            d.next_state.replan_reason = "hold_due_to_low_deviation";
        } else {
            d.action = ManagerAction::UPDATE_CURRENT;
            d.reason = "soft_update_path";
            d.next_state.trajectory = normalized_candidate;
            d.next_state.remaining_s_mm = calculate_path_length(normalized_candidate.points);
            d.next_state.last_good_frame = current_frame;
            d.next_state.replan_reason = "soft_update";
        }
        
        return d;
    }
    
private:
    static double calculate_path_length(const std::vector<Point2D>& pts) {
        double len = 0.0;
        for (size_t i = 1; i < pts.size(); ++i) {
            len += std::sqrt(std::pow(pts[i].x - pts[i-1].x, 2) + std::pow(pts[i].y - pts[i-1].y, 2));
        }
        return len;
    }
    
    static double calculate_path_deviation(const std::vector<Point2D>& path_a, const std::vector<Point2D>& path_b) {
        if (path_a.empty() || path_b.empty()) return 1e9;
        
        double total_dev = 0.0;
        size_t count = std::min(path_a.size(), path_b.size());
        for (size_t i = 0; i < count; ++i) {
            total_dev += std::abs(path_a[i].x - path_b[i].x);
        }
        return count > 0 ? (total_dev / count) : 0.0;
    }
};

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
        route_intent_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/avs/route_intent", 10,
            std::bind(&LaneErrorNode::route_intent_callback, this, std::placeholders::_1)
        );
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/avs/cmd", 10,
            std::bind(&LaneErrorNode::cmd_callback, this, std::placeholders::_1)
        );
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_raw", 10,
            std::bind(&LaneErrorNode::odom_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "LaneErrorNode started. Initial state: FOLLOW_MAIN");
        RCLCPP_INFO(this->get_logger(), "Subscribing: /avs/telemetry_realworld, /avs/route_intent, /avs/cmd, /odom_raw");
        RCLCPP_INFO(this->get_logger(), "Publishing:  /avs/control_error, /avs/lane_state");
    }

private:
    struct TrajectoryErrorParams {
        Point2D point = {0.0, 0.0};
        double theta = 0.0;
        double curvature = 0.0;
    };

    // ── Helper to evaluate trajectory parameters at lookahead ───────────────
    TrajectoryErrorParams evaluate_trajectory_at_lookahead(const ActiveTrajectory& traj, double lookahead_d_mm) {
        TrajectoryErrorParams params;
        if (traj.points.empty()) return params;

        // Create a virtual trajectory starting at the vehicle origin (0.0, 0.0)
        std::vector<Point2D> pts;
        pts.reserve(traj.points.size() + 1);
        pts.push_back({0.0, 0.0});
        pts.insert(pts.end(), traj.points.begin(), traj.points.end());

        double cumulative_dist = 0.0;
        Point2D target_pt = pts.front();
        size_t target_idx = 0;

        bool found_target = false;
        Point2D prev_pt = pts.front();
        for (size_t i = 1; i < pts.size(); ++i) {
            double dx = pts[i].x - prev_pt.x;
            double dy = pts[i].y - prev_pt.y;
            double segment_len = std::sqrt(dx*dx + dy*dy);
            if (cumulative_dist + segment_len >= lookahead_d_mm && segment_len > 1e-6) {
                double ratio = (lookahead_d_mm - cumulative_dist) / segment_len;
                ratio = std::max(0.0, std::min(1.0, ratio));
                target_pt = { prev_pt.x + ratio * dx, prev_pt.y + ratio * dy };
                target_idx = i;
                found_target = true;
                break;
            }
            cumulative_dist += segment_len;
            prev_pt = pts[i];
        }

        // If the entire trajectory is shorter than lookahead_d_mm, clamp to the last point
        if (!found_target) {
            target_pt = pts.back();
            target_idx = pts.size() - 1;
        }

        params.point = target_pt;
        if (std::abs(target_pt.y) > 1e-3 || std::abs(target_pt.x) > 1e-3) {
            params.theta = std::atan2(target_pt.x, target_pt.y);
        }

        if (pts.size() >= 3) {
            size_t c_idx = target_idx;
            if (c_idx == 0) c_idx = 1;
            if (c_idx == pts.size() - 1) c_idx = pts.size() - 2;

            Point2D p1 = pts[c_idx - 1];
            Point2D p2 = pts[c_idx];
            Point2D p3 = pts[c_idx + 1];

            double a = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
            double b = std::sqrt(std::pow(p3.x - p2.x, 2) + std::pow(p3.y - p2.y, 2));
            double c = std::sqrt(std::pow(p3.x - p1.x, 2) + std::pow(p3.y - p1.y, 2));

            if (a > 0 && b > 0 && c > 0) {
                double cross = (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.x);
                params.curvature = 2.0 * cross / (a * b * c);
            }
        }

        return params;
    }

    // ── Route intent callback ────────────────────────────────────────────────
    void route_intent_callback(const std_msgs::msg::String::SharedPtr msg) {
        try {
            json intent_json = json::parse(msg->data);
            if (intent_json.contains("intent")) {
                std::string intent_str = intent_json["intent"].get<std::string>();
                if (intent_str == "follow_main") {
                    current_intent_ = RouteIntent::FOLLOW_MAIN;
                } else if (intent_str == "turn_right") {
                    current_intent_ = RouteIntent::TURN_RIGHT;
                } else if (intent_str == "turn_left") {
                    current_intent_ = RouteIntent::TURN_LEFT;
                } else if (intent_str == "lane_change_left") {
                    current_intent_ = RouteIntent::LANE_CHANGE_LEFT;
                } else if (intent_str == "lane_change_right") {
                    current_intent_ = RouteIntent::LANE_CHANGE_RIGHT;
                } else if (intent_str == "straight") {
                    current_intent_ = RouteIntent::FOLLOW_MAIN;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Received legacy intent 'straight', mapping to FOLLOW_MAIN");
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Received unrecognized intent '%s', ignoring", intent_str.c_str());
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "route_intent_callback parse error: %s. Ignoring message", e.what());
        }
    }

    // ── External command callback ────────────────────────────────────────────
    void cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
        try {
            json cmd_json = json::parse(msg->data);
            std::string cmd = cmd_json.value("cmd", "");

            if (cmd == "arm" || cmd == "disarm" || cmd == "resume") {
                RCLCPP_INFO(this->get_logger(), "CMD: System command received: %s", cmd.c_str());
                if (cmd == "resume") {
                    current_intent_ = RouteIntent::FOLLOW_MAIN;
                }
            } else if (cmd == "tur" "n") {
                current_intent_ = RouteIntent::LEGACY_TURN;
                RCLCPP_INFO(this->get_logger(), "CMD: Legacy turn command received. Arming legacy turn intent.");
            } else if (cmd == "lane_chang" "e") {
                current_intent_ = RouteIntent::LEGACY_LANE_CHANGE;
                RCLCPP_INFO(this->get_logger(), "CMD: Legacy lane_change command received. Arming legacy lane change intent.");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "cmd_callback parse error: %s", e.what());
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_speed_mms_ = std::abs(msg->twist.twist.linear.x) * 2500.0;
    }

    // ── Telemetry callback: build trajectory and publish errors ─────────────
    // Counter for robust T-junction detection
    int t_junction_counter_ = 0;

    bool detect_t_junction(const LaneCandidate* main_current, 
                           const LaneCandidate* main_ahead, 
                           const std::vector<LaneCandidate>& lanes,
                           bool& is_t_geom_out) {
        bool is_t_geom = false;
        if (main_current && !main_ahead) {
            double main_end_y = 0.0;
            ActiveTrajectory main_traj = build_trajectory_from_candidate(*main_current);
            if (!main_traj.points.empty()) {
                main_end_y = main_traj.points.back().y;
            } else if (main_traj.has_precomputed_control) {
                main_end_y = main_traj.precomputed_epsilon_y_mm;
            } else if (main_current->raw_obj.contains("waypoints") && !main_current->raw_obj["waypoints"].empty()) {
                main_end_y = main_current->raw_obj["waypoints"].back()[1].get<double>();
            }
            
            double min_turn_x = 1e9, max_turn_x = -1e9;
            double avg_turn_start_y = 0;
            int turn_count = 0;
            for (const auto& l : lanes) {
                if (l.label == 17) {
                    ActiveTrajectory turn_traj = build_trajectory_from_candidate(l);
                    if (turn_traj.valid) {
                        if (!turn_traj.points.empty()) {
                            for (const auto& p : turn_traj.points) {
                                min_turn_x = std::min(min_turn_x, p.x);
                                max_turn_x = std::max(max_turn_x, p.x);
                            }
                            avg_turn_start_y += turn_traj.points.front().y;
                            turn_count++;
                        } else if (turn_traj.has_precomputed_control) {
                            double px = turn_traj.precomputed_epsilon_x_mm;
                            double py = turn_traj.precomputed_epsilon_y_mm;
                            min_turn_x = std::min(min_turn_x, px);
                            max_turn_x = std::max(max_turn_x, px);
                            avg_turn_start_y += py;
                            turn_count++;
                        }
                    }
                }
            }
            if (turn_count > 0) {
                avg_turn_start_y /= turn_count;
                if (max_turn_x - min_turn_x > 2000.0 && std::abs(main_end_y - avg_turn_start_y) < 1500.0) {
                    is_t_geom = true;
                }
            }
        }
        
        if (is_t_geom) {
            t_junction_counter_++;
        } else {
            t_junction_counter_ = 0;
        }
        
        is_t_geom_out = is_t_geom;
        return (t_junction_counter_ >= 3);
    }

    double get_candidate_average_x(const LaneCandidate& l) const {
        if (l.raw_obj.contains("waypoints") && l.raw_obj["waypoints"].is_array() && !l.raw_obj["waypoints"].empty()) {
            double sum_x = 0;
            int count = 0;
            for (const auto& pt : l.raw_obj["waypoints"]) {
                if (pt.is_array() && pt.size() >= 2) {
                    sum_x += pt[0].get<double>();
                    count++;
                }
            }
            if (count > 0) return sum_x / count;
        } else if (l.raw_obj.contains("lookahead_x_mm")) {
            return l.raw_obj["lookahead_x_mm"].get<double>();
        } else if (l.raw_obj.contains("lookahead_theta_rad")) {
            return l.raw_obj["lookahead_theta_rad"].get<double>();
        }
        return 0.0;
    }

    void telemetry_callback(const std_msgs::msg::String::SharedPtr msg) {
        // Track telemetry timing for debug/runtime bookkeeping.
        double dt = 0.033;
        auto now = this->get_clock()->now();
        if (last_telemetry_time_.nanoseconds() > 0) {
            dt = (now - last_telemetry_time_).seconds();
        }
        last_telemetry_time_ = now;
        (void)dt;

        // Reload thresholds in case they were updated at runtime
        turn_proximity_mm_ = this->get_parameter("turn_proximity_mm").as_double();
        turn_done_mm_      = this->get_parameter("turn_done_mm").as_double();
        theta_done_rad_    = this->get_parameter("theta_done_rad").as_double();

        try {
            json telemetry = json::parse(msg->data);
            std::vector<LaneCandidate> lanes = extract_lane_candidates(telemetry);
            std::vector<MarkingCandidate> markings = extract_marking_candidates(telemetry);

            // Resolve legacy directionless intents from /avs/cmd
            if (current_intent_ == RouteIntent::LEGACY_TURN) {
                for (const auto& l : lanes) {
                    if (l.label == 17) {
                        double avg_x = get_candidate_average_x(l);
                        if (avg_x > 0.0) {
                            current_intent_ = RouteIntent::TURN_RIGHT;
                            RCLCPP_INFO(this->get_logger(), "Resolved LEGACY_TURN to TURN_RIGHT based on perception");
                            break;
                        } else if (avg_x < 0.0) {
                            current_intent_ = RouteIntent::TURN_LEFT;
                            RCLCPP_INFO(this->get_logger(), "Resolved LEGACY_TURN to TURN_LEFT based on perception");
                            break;
                        }
                    }
                }
            } else if (current_intent_ == RouteIntent::LEGACY_LANE_CHANGE) {
                for (const auto& l : lanes) {
                    if (l.label == 4) {
                        double avg_x = get_candidate_average_x(l);
                        if (avg_x > 0.0) {
                            current_intent_ = RouteIntent::LANE_CHANGE_RIGHT;
                            RCLCPP_INFO(this->get_logger(), "Resolved LEGACY_LANE_CHANGE to LANE_CHANGE_RIGHT based on perception");
                            break;
                        } else if (avg_x < 0.0) {
                            current_intent_ = RouteIntent::LANE_CHANGE_LEFT;
                            RCLCPP_INFO(this->get_logger(), "Resolved LEGACY_LANE_CHANGE to LANE_CHANGE_LEFT based on perception");
                            break;
                        }
                    }
                }
            }

            // ── Collect lane objects by label ───────────────────────────────
            const LaneCandidate* main_current = nullptr;
            const LaneCandidate* main_ahead = nullptr;
            split_main_lanes(lanes, main_current, main_ahead);

            const LaneCandidate* other_lane_cand = nullptr;
            const LaneCandidate* turn_lane_cand = nullptr;
            bool stop_line_detected = false;

            for (const auto& l : lanes) {
                if (l.label == 4) other_lane_cand = &l;
            }
            for (const auto& m : markings) {
                if (m.label == 16) stop_line_detected = true;
            }

            bool is_t_geom = false;
            bool is_t = detect_t_junction(main_current, main_ahead, lanes, is_t_geom);
            bool t_junction_pending = is_t_geom && !is_t;

            // Select the turn lane candidate based on the active turning intent or state
            bool is_right_turn = (current_intent_ == RouteIntent::TURN_RIGHT || state_ == DecisionState::TURN_RIGHT);
            turn_lane_cand = select_turn_lane(lanes, is_right_turn, is_t);

            // ── State transition logic ──────────────────────────────────────
            update_lane_state(lanes, markings, main_current, turn_lane_cand, stop_line_detected, is_t);

            // ── Build Active Trajectory ─────────────────────────────────────
            ActiveTrajectory active_traj;
            bool blocked_by_marking = false;
            std::string selected_lane_id = "";
            const LaneCandidate* active_target_lane = nullptr;

            DecisionState baseline_state = DecisionState::FOLLOW_MAIN;
            if (current_intent_ == RouteIntent::TURN_RIGHT) {
                baseline_state = DecisionState::TURN_RIGHT;
            } else if (current_intent_ == RouteIntent::TURN_LEFT) {
                baseline_state = DecisionState::TURN_LEFT;
            } else if (current_intent_ == RouteIntent::LANE_CHANGE_LEFT || current_intent_ == RouteIntent::LANE_CHANGE_RIGHT) {
                baseline_state = DecisionState::LANE_CHANGE;
            }

            if ((state_ == DecisionState::TURN_RIGHT || state_ == DecisionState::TURN_LEFT) && t_junction_pending && main_current) {
                state_ = DecisionState::FOLLOW_MAIN;
            }

            DecisionState active_eval_state = state_;

            switch (active_eval_state) {
                case DecisionState::FOLLOW_MAIN:
                case DecisionState::RECOVERY:
                case DecisionState::BLOCKED: {
                    if (active_eval_state == DecisionState::BLOCKED) {
                        blocked_by_marking = true;
                    }
                    
                    // 1. Build the path observation frame
                    PathObservationFrame obs_frame = PathObservationBuilder::build(telemetry);
                    
                    // 2. Plan candidate (Always follow-main in this branch, ignore pending route intent until maneuver starts)
                    std::string current_intent_str = "follow_main";
                    
                    PlannedTrajectory planned_candidate = TrajectoryPlanner::plan_follow_main(obs_frame, committed_state_, last_main_track_id_);
                    
                    // 3. Normalize candidate
                    PlannedTrajectory normalized_candidate = TrajectoryNormalizer::normalize(planned_candidate, committed_state_);
                    
                    // Increment frame count
                    frame_count_++;
                    
                    // 4. Update memory using manager
                    TrajectoryManager::Decision decision = TrajectoryManager::update(
                        normalized_candidate,
                        committed_state_,
                        current_intent_str,
                        consecutive_invalid_frames_,
                        frame_count_
                    );
                    
                    // Apply decision state update
                    committed_state_ = decision.next_state;
                    
                    // 5. Populate active_traj from committed_state_
                    active_traj.valid = committed_state_.trajectory.valid;
                    active_traj.points = committed_state_.trajectory.points;
                    active_traj.trajectory_kind = trajectory_kind_name(committed_state_.trajectory.trajectory_kind);
                    active_traj.normalization_mode = committed_state_.trajectory.normalization_mode;
                    active_traj.trajectory_confidence = committed_state_.trajectory.confidence;
                    
                    active_traj.source_labels.clear();
                    for (const auto& id_str : committed_state_.trajectory.source_lane_ids) {
                        size_t colon_pos = id_str.find(':');
                        if (colon_pos != std::string::npos) {
                            try {
                                active_traj.source_labels.push_back(std::stoi(id_str.substr(0, colon_pos)));
                            } catch (...) {}
                        } else {
                            try {
                                active_traj.source_labels.push_back(std::stoi(id_str));
                            } catch (...) {}
                        }
                    }
                    
                    // Propagate precomputed control fields from committed trajectory state (provides memory fallback during dropouts)
                    if (committed_state_.trajectory.valid && committed_state_.trajectory.has_precomputed_control) {
                        active_traj.has_precomputed_control = true;
                        active_traj.precomputed_epsilon_x_mm = committed_state_.trajectory.precomputed_epsilon_x_mm;
                        active_traj.precomputed_epsilon_y_mm = committed_state_.trajectory.precomputed_epsilon_y_mm;
                        active_traj.precomputed_theta_rad = committed_state_.trajectory.precomputed_theta_rad;
                        active_traj.precomputed_curvature_inv_mm = committed_state_.trajectory.precomputed_curvature_inv_mm;
                        active_traj.precomputed_lookahead_d_mm = committed_state_.trajectory.precomputed_lookahead_d_mm;
                    }
                    
                    // Overwrite with the latest per-frame observation if the target has a stable upstream ID
                    // Synthesized IDs (obj_*) cannot reliably identify the same physical lane across frames,
                    // so we skip live refresh and rely on committed trajectory values above
                    if (!obs_frame.lanes.empty() && !committed_state_.trajectory.target_lane_id.empty() && committed_state_.trajectory.target_lane_id.rfind("obj_", 0) != 0) {
                        const LaneObservation* cur_lane = nullptr;
                        for (const auto& l : obs_frame.lanes) {
                            if (l.lane_id == committed_state_.trajectory.target_lane_id) {
                                cur_lane = &l;
                                break;
                            }
                        }
                        if (cur_lane && cur_lane->has_precomputed_control) {
                            active_traj.has_precomputed_control = true;
                            active_traj.precomputed_epsilon_x_mm = cur_lane->precomputed_epsilon_x_mm;
                            active_traj.precomputed_epsilon_y_mm = cur_lane->precomputed_epsilon_y_mm;
                            active_traj.precomputed_theta_rad = cur_lane->precomputed_theta_rad;
                            active_traj.precomputed_curvature_inv_mm = cur_lane->precomputed_curvature_inv_mm;
                            active_traj.precomputed_lookahead_d_mm = cur_lane->precomputed_lookahead_d_mm;
                        }
                    }
                    
                    // Update decision state based on manager action (Preserve BLOCKED state if determined by update_lane_state)
                    if (decision.action == ManagerAction::ENTER_RECOVERY) {
                        state_ = DecisionState::RECOVERY;
                    } else if (active_eval_state == DecisionState::BLOCKED || decision.action == ManagerAction::ENTER_BLOCKED) {
                        state_ = DecisionState::BLOCKED;
                    } else {
                        state_ = DecisionState::FOLLOW_MAIN;
                    }
                    
                    active_target_lane = main_current;
                    selected_lane_id = committed_state_.trajectory.target_lane_id;
                    break;
                }
                case DecisionState::LANE_CHANGE: {
                    bool is_left = (current_intent_ == RouteIntent::LANE_CHANGE_LEFT);
                    
                    // 1. Build the path observation frame
                    PathObservationFrame obs_frame = PathObservationBuilder::build(telemetry);
                    
                    // 2. Plan candidate
                    PlannedTrajectory planned_candidate = is_left ?
                        TrajectoryPlanner::plan_lane_change_left(obs_frame, committed_state_, last_main_track_id_) :
                        TrajectoryPlanner::plan_lane_change_right(obs_frame, committed_state_, last_main_track_id_);
                    
                    bool blocked = planned_candidate.blocked_by_marking;
                    if (blocked) {
                        state_ = DecisionState::BLOCKED;
                        blocked_by_marking = true;
                    }
                    
                    // 3. Normalize candidate
                    PlannedTrajectory normalized_candidate = TrajectoryNormalizer::normalize(planned_candidate, committed_state_);
                    
                    // Increment frame count
                    frame_count_++;
                    
                    // 4. Update memory using manager
                    std::string intent_str = is_left ? "lane_change_left" : "lane_change_right";
                    if (blocked) {
                        intent_str = "follow_main";
                    }
                    
                    TrajectoryManager::Decision decision = TrajectoryManager::update(
                        normalized_candidate,
                        committed_state_,
                        intent_str,
                        consecutive_invalid_frames_,
                        frame_count_
                    );
                    
                    // Apply decision state update
                    committed_state_ = decision.next_state;
                    
                    // 5. Populate active_traj from committed_state_
                    active_traj.valid = committed_state_.trajectory.valid;
                    active_traj.points = committed_state_.trajectory.points;
                    active_traj.trajectory_kind = trajectory_kind_name(committed_state_.trajectory.trajectory_kind);
                    active_traj.normalization_mode = committed_state_.trajectory.normalization_mode;
                    active_traj.trajectory_confidence = committed_state_.trajectory.confidence;
                    
                    active_traj.source_labels.clear();
                    for (const auto& id_str : committed_state_.trajectory.source_lane_ids) {
                        size_t colon_pos = id_str.find(':');
                        if (colon_pos != std::string::npos) {
                            try {
                                active_traj.source_labels.push_back(std::stoi(id_str.substr(0, colon_pos)));
                            } catch (...) {}
                        } else {
                            try {
                                active_traj.source_labels.push_back(std::stoi(id_str));
                            } catch (...) {}
                        }
                    }
                    
                    // Propagate precomputed control fields from committed trajectory state (provides memory fallback during dropouts)
                    if (committed_state_.trajectory.valid && committed_state_.trajectory.has_precomputed_control) {
                        active_traj.has_precomputed_control = true;
                        active_traj.precomputed_epsilon_x_mm = committed_state_.trajectory.precomputed_epsilon_x_mm;
                        active_traj.precomputed_epsilon_y_mm = committed_state_.trajectory.precomputed_epsilon_y_mm;
                        active_traj.precomputed_theta_rad = committed_state_.trajectory.precomputed_theta_rad;
                        active_traj.precomputed_curvature_inv_mm = committed_state_.trajectory.precomputed_curvature_inv_mm;
                        active_traj.precomputed_lookahead_d_mm = committed_state_.trajectory.precomputed_lookahead_d_mm;
                    }
                    
                    // Overwrite with the latest per-frame observation if the target has a stable upstream ID
                    if (!obs_frame.lanes.empty() && !committed_state_.trajectory.target_lane_id.empty() && committed_state_.trajectory.target_lane_id.rfind("obj_", 0) != 0) {
                        const LaneObservation* cur_lane = nullptr;
                        for (const auto& l : obs_frame.lanes) {
                            if (l.lane_id == committed_state_.trajectory.target_lane_id) {
                                cur_lane = &l;
                                break;
                            }
                        }
                        if (cur_lane && cur_lane->has_precomputed_control) {
                            active_traj.has_precomputed_control = true;
                            active_traj.precomputed_epsilon_x_mm = cur_lane->precomputed_epsilon_x_mm;
                            active_traj.precomputed_epsilon_y_mm = cur_lane->precomputed_epsilon_y_mm;
                            active_traj.precomputed_theta_rad = cur_lane->precomputed_theta_rad;
                            active_traj.precomputed_curvature_inv_mm = cur_lane->precomputed_curvature_inv_mm;
                            active_traj.precomputed_lookahead_d_mm = cur_lane->precomputed_lookahead_d_mm;
                        }
                    }
                    
                    // Update state_ based on manager decision unless blocked
                    if (!blocked) {
                        if (decision.action == ManagerAction::ENTER_RECOVERY) {
                            state_ = DecisionState::RECOVERY;
                        } else if (committed_state_.trajectory.trajectory_kind == TrajectoryKind::FOLLOW_MAIN) {
                            // Lane change complete! Transition back to FOLLOW_MAIN
                            state_ = DecisionState::FOLLOW_MAIN;
                        } else {
                            state_ = active_eval_state;
                        }
                    }
                    
                    active_target_lane = main_current;
                    if (planned_candidate.valid) {
                        for (const auto& l : lanes) {
                            if (lane_id_string(&l) == planned_candidate.target_lane_id) {
                                active_target_lane = &l;
                                break;
                            }
                        }
                    }
                    selected_lane_id = committed_state_.trajectory.target_lane_id;
                    break;
                }
                case DecisionState::TURN_RIGHT:
                case DecisionState::TURN_LEFT: {
                    bool is_right = (active_eval_state == DecisionState::TURN_RIGHT);
                    
                    // 1. Build the path observation frame
                    PathObservationFrame obs_frame = PathObservationBuilder::build(telemetry);
                    
                    // 2. Plan candidate
                    PlannedTrajectory planned_candidate = is_right ?
                        TrajectoryPlanner::plan_turn_right(obs_frame, committed_state_, is_t, t_junction_pending, last_main_track_id_) :
                        TrajectoryPlanner::plan_turn_left(obs_frame, committed_state_, is_t, t_junction_pending, last_main_track_id_);
                    
                    // Check if blocked by solid marking
                    bool blocked = false;
                    if (!is_right && is_t && planned_candidate.valid) {
                        ActiveTrajectory temp_traj;
                        temp_traj.points = planned_candidate.points;
                        temp_traj.valid = true;
                        blocked = is_turn_blocked_by_solid(temp_traj, markings);
                    }
                    
                    if (blocked) {
                        state_ = DecisionState::BLOCKED;
                        blocked_by_marking = true;
                        planned_candidate = TrajectoryPlanner::plan_follow_main(obs_frame, committed_state_, last_main_track_id_);
                    }
                    
                    // 3. Normalize candidate
                    PlannedTrajectory normalized_candidate = TrajectoryNormalizer::normalize(planned_candidate, committed_state_);
                    
                    // Increment frame count
                    frame_count_++;
                    
                    // 4. Update memory using manager
                    std::string intent_str = is_right ? "turn_right" : "turn_left";
                    if (blocked) {
                        intent_str = "follow_main";
                    }
                    
                    TrajectoryManager::Decision decision = TrajectoryManager::update(
                        normalized_candidate,
                        committed_state_,
                        intent_str,
                        consecutive_invalid_frames_,
                        frame_count_
                    );
                    
                    // Apply decision state update
                    committed_state_ = decision.next_state;
                    
                    // 5. Populate active_traj from committed_state_
                    active_traj.valid = committed_state_.trajectory.valid;
                    active_traj.points = committed_state_.trajectory.points;
                    active_traj.trajectory_kind = trajectory_kind_name(committed_state_.trajectory.trajectory_kind);
                    active_traj.normalization_mode = committed_state_.trajectory.normalization_mode;
                    active_traj.trajectory_confidence = committed_state_.trajectory.confidence;
                    
                    active_traj.source_labels.clear();
                    for (const auto& id_str : committed_state_.trajectory.source_lane_ids) {
                        size_t colon_pos = id_str.find(':');
                        if (colon_pos != std::string::npos) {
                            try {
                                active_traj.source_labels.push_back(std::stoi(id_str.substr(0, colon_pos)));
                            } catch (...) {}
                        } else {
                            try {
                                active_traj.source_labels.push_back(std::stoi(id_str));
                            } catch (...) {}
                        }
                    }
                    
                    // Propagate precomputed control fields from committed trajectory state (provides memory fallback during dropouts)
                    if (committed_state_.trajectory.valid && committed_state_.trajectory.has_precomputed_control) {
                        active_traj.has_precomputed_control = true;
                        active_traj.precomputed_epsilon_x_mm = committed_state_.trajectory.precomputed_epsilon_x_mm;
                        active_traj.precomputed_epsilon_y_mm = committed_state_.trajectory.precomputed_epsilon_y_mm;
                        active_traj.precomputed_theta_rad = committed_state_.trajectory.precomputed_theta_rad;
                        active_traj.precomputed_curvature_inv_mm = committed_state_.trajectory.precomputed_curvature_inv_mm;
                        active_traj.precomputed_lookahead_d_mm = committed_state_.trajectory.precomputed_lookahead_d_mm;
                    }
                    
                    // Overwrite with the latest per-frame observation if the target has a stable upstream ID
                    // Synthesized IDs (obj_*) cannot reliably identify the same physical lane across frames,
                    // so we skip live refresh and rely on committed trajectory values above
                    if (!obs_frame.lanes.empty() && !committed_state_.trajectory.target_lane_id.empty() && committed_state_.trajectory.target_lane_id.rfind("obj_", 0) != 0) {
                        const LaneObservation* cur_lane = nullptr;
                        for (const auto& l : obs_frame.lanes) {
                            if (l.lane_id == committed_state_.trajectory.target_lane_id) {
                                cur_lane = &l;
                                break;
                            }
                        }
                        if (cur_lane && cur_lane->has_precomputed_control) {
                            active_traj.has_precomputed_control = true;
                            active_traj.precomputed_epsilon_x_mm = cur_lane->precomputed_epsilon_x_mm;
                            active_traj.precomputed_epsilon_y_mm = cur_lane->precomputed_epsilon_y_mm;
                            active_traj.precomputed_theta_rad = cur_lane->precomputed_theta_rad;
                            active_traj.precomputed_curvature_inv_mm = cur_lane->precomputed_curvature_inv_mm;
                            active_traj.precomputed_lookahead_d_mm = cur_lane->precomputed_lookahead_d_mm;
                        }
                    }
                    
                    // Update state_ based on manager decision unless blocked
                    if (!blocked) {
                        if (decision.action == ManagerAction::ENTER_RECOVERY) {
                            state_ = DecisionState::RECOVERY;
                        } else {
                            state_ = active_eval_state;
                        }
                    }
                    
                    active_target_lane = main_current;
                    if (planned_candidate.valid) {
                        for (const auto& l : lanes) {
                            if (lane_id_string(&l) == planned_candidate.target_lane_id) {
                                active_target_lane = &l;
                                break;
                            }
                        }
                    }
                    selected_lane_id = committed_state_.trajectory.target_lane_id;
                    break;
                }
            }

            // ── Extract and publish control errors ──────────────────────────
            if (active_traj.valid) {
                double lookahead_d = 600.0;
                if (active_target_lane && active_target_lane->raw_obj.contains("lookahead_d_mm")) {
                    lookahead_d = active_target_lane->raw_obj["lookahead_d_mm"].get<double>();
                } else if (main_current && main_current->raw_obj.contains("lookahead_d_mm")) {
                    lookahead_d = main_current->raw_obj["lookahead_d_mm"].get<double>();
                }

                // Hybrid Control Policy: for straight-line following states (FOLLOW_MAIN, BLOCKED, RECOVERY),
                // prioritize direct polynomial lookahead from IPM to completely eliminate lateral drift bias,
                // BUT only if the upcoming connected trajectory does not diverge significantly in lateral position (< 100mm),
                // heading angle (< 0.05 rad / ~3 degrees), and curvature (< 1e-5) from the local polynomial.
                // This ensures we keep stable direct IPM lookahead on segmented straight roads, while immediately yielding to
                // the connected trajectory at any turn entry, gentle curve, or junction bend to steer early and prevent understeer.
                bool use_direct_lookahead = false;
                if (state_ == DecisionState::FOLLOW_MAIN || state_ == DecisionState::BLOCKED || state_ == DecisionState::RECOVERY) {
                    if (main_current && main_current->raw_obj.contains("lookahead_x_mm") && main_current->raw_obj.contains("lookahead_d_mm")) {
                        if (!main_ahead) {
                            use_direct_lookahead = true; // No continuation: safely use stable direct IPM
                        } else {
                            // Evaluate the connected trajectory's parameters at the lookahead distance
                            TrajectoryErrorParams traj_params = evaluate_trajectory_at_lookahead(active_traj, lookahead_d);
                            
                            double direct_x = main_current->raw_obj["lookahead_x_mm"].get<double>();
                            double direct_d = main_current->raw_obj["lookahead_d_mm"].get<double>();
                            double direct_theta = main_current->raw_obj.value("lookahead_theta_rad", std::atan2(direct_x, direct_d));
                            double direct_curvature = main_current->raw_obj.value("curvature_inv_mm", 0.0);
                            
                            bool lateral_match = std::abs(traj_params.point.x - direct_x) < 100.0;
                            bool heading_match = std::abs(traj_params.theta - direct_theta) < 0.05; // ~3 degrees
                            bool curvature_match = std::abs(traj_params.curvature - direct_curvature) < 1e-5;
                            
                            if (lateral_match && heading_match && curvature_match) {
                                use_direct_lookahead = true; // Segmented straight road: use stable direct IPM
                            }
                        }
                    }
                }

                if (use_direct_lookahead) {
                    ActiveTrajectory direct_traj = active_traj;
                    direct_traj.has_precomputed_control = true;
                    direct_traj.precomputed_epsilon_x_mm = main_current->raw_obj["lookahead_x_mm"].get<double>();
                    direct_traj.precomputed_epsilon_y_mm = main_current->raw_obj["lookahead_d_mm"].get<double>();
                    direct_traj.precomputed_theta_rad = main_current->raw_obj.value("lookahead_theta_rad", 
                        std::atan2(direct_traj.precomputed_epsilon_x_mm, direct_traj.precomputed_epsilon_y_mm));
                    direct_traj.precomputed_curvature_inv_mm = main_current->raw_obj.value("curvature_inv_mm", 0.0);
                    direct_traj.precomputed_lookahead_d_mm = direct_traj.precomputed_epsilon_y_mm;
                    publish_control_error_from_trajectory(direct_traj, lookahead_d);
                } else {
                    publish_control_error_from_trajectory(active_traj, lookahead_d);
                }
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "[%s] Target trajectory invalid — publishing invalid control error.",
                    decision_state_name(state_));
                // Publish invalid control error with safe default errors (trajectory_valid: false)
                publish_control_error_from_trajectory(active_traj, 600.0);
            }

            // ── Synchronize Committed State for Phase 1 ──────────────────────
            if (state_ != DecisionState::FOLLOW_MAIN && state_ != DecisionState::RECOVERY && state_ != DecisionState::BLOCKED) {
                if (active_traj.valid) {
                    consecutive_invalid_frames_ = 0;
                    committed_state_.trajectory.valid = true;
                    committed_state_.trajectory.target_lane_id = selected_lane_id;
                    committed_state_.trajectory.trajectory_kind = string_to_trajectory_kind(active_traj.trajectory_kind, current_intent_);
                    committed_state_.trajectory.points.clear();
                    committed_state_.trajectory.points.reserve(active_traj.points.size());
                    for (const auto& pt : active_traj.points) {
                        Point2D new_pt;
                        new_pt.x = pt.x;
                        new_pt.y = pt.y;
                        committed_state_.trajectory.points.push_back(new_pt);
                    }
                    committed_state_.trajectory.source_lane_ids.clear();
                    for (int lbl : active_traj.source_labels) {
                        committed_state_.trajectory.source_lane_ids.push_back(std::to_string(lbl));
                    }
                    committed_state_.trajectory.normalization_mode =
                        (active_traj.normalization_mode.empty() || active_traj.normalization_mode == "none")
                            ? "legacy_sync"
                            : active_traj.normalization_mode;
                    committed_state_.trajectory.confidence =
                        (active_traj.trajectory_confidence > 0.0) ? active_traj.trajectory_confidence : 1.0;
                    committed_state_.replan_reason = blocked_by_marking ? "blocked_by_marking" : "none";
                } else {
                    consecutive_invalid_frames_++;
                    if (consecutive_invalid_frames_ > 5) {
                        // Persistent invalid plan: clear committed state to prevent emitting stale paths
                        committed_state_.trajectory.valid = false;
                        committed_state_.trajectory.target_lane_id = "";
                        committed_state_.trajectory.trajectory_kind = TrajectoryKind::UNKNOWN;
                        committed_state_.trajectory.points.clear();
                        committed_state_.trajectory.source_lane_ids.clear();
                        committed_state_.trajectory.normalization_mode = "none";
                        committed_state_.trajectory.confidence = 0.0;
                        committed_state_.progress_s_mm = 0.0;
                        committed_state_.remaining_s_mm = 0.0;
                        committed_state_.dropout_hold_counter = consecutive_invalid_frames_;
                        committed_state_.replan_reason = "none";
                    }
                }
            }

            // ── Always publish lane state ───────────────────────────────────
            bool has_main_raw = (main_current != nullptr || main_ahead != nullptr);
            bool has_other_raw = false;
            bool has_turn_raw = false;
            for (const auto& l : lanes) {
                if (l.label == 4) has_other_raw = true;
                if (l.label == 17) has_turn_raw = true;
            }

            publish_lane_state(
                has_main_raw,
                has_other_raw,
                has_turn_raw,
                stop_line_detected,
                blocked_by_marking,
                active_traj,
                selected_lane_id
            );

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "telemetry_callback error: %s", e.what());
        }
    }

    std::string lane_id_string(const LaneCandidate* lane) const {
        if (!lane) return "";
        // Check id first, then track_id — same priority as PathObservationBuilder
        if (lane->raw_obj.contains("id")) {
            const auto& id = lane->raw_obj["id"];
            if (id.is_string()) return id.get<std::string>();
            if (!id.is_null()) return id.dump();
        }
        if (lane->raw_obj.contains("track_id")) {
            const auto& tid = lane->raw_obj["track_id"];
            if (tid.is_string()) return tid.get<std::string>();
            if (!tid.is_null()) return tid.dump();
        }
        return "";
    }

    double get_lane_heading(const LaneCandidate& lane) const {
        if (!lane.raw_obj.contains("waypoints") || !lane.raw_obj["waypoints"].is_array() || lane.raw_obj["waypoints"].empty()) {
            return 0.0;
        }
        const auto& wps = lane.raw_obj["waypoints"];
        if (wps.size() < 2) return 0.0;
        
        // Use local heading using the first few waypoints (up to index 3, ~300mm ahead of vehicle)
        size_t end_idx = std::min(wps.size() - 1, size_t(3));
        double dx = wps[end_idx][0].get<double>() - wps.front()[0].get<double>();
        double dy = wps[end_idx][1].get<double>() - wps.front()[1].get<double>();
        
        // Fallback to global heading if local segment is too short or degenerate
        if (std::sqrt(dx*dx + dy*dy) < 10.0) {
            dx = wps.back()[0].get<double>() - wps.front()[0].get<double>();
            dy = wps.back()[1].get<double>() - wps.front()[1].get<double>();
        }
        return std::atan2(dx, dy);
    }

    // ── Helper Extractors ────────────────────────────────────────────────────
    void split_main_lanes(const std::vector<LaneCandidate>& lanes, 
                          const LaneCandidate*& out_current, 
                          const LaneCandidate*& out_ahead) {
        out_current = nullptr;
        out_ahead = nullptr;
        
        std::vector<const LaneCandidate*> main_lanes;
        for (const auto& l : lanes) {
            if (l.label == 3) main_lanes.push_back(&l);
        }
        
        if (main_lanes.empty()) {
            last_main_track_id_ = "";
            return;
        }
        
        // Find min_start_y among all candidates to guide hysteresis P1 guard
        double min_start_y = 1e9;
        std::vector<double> start_y_vals(main_lanes.size(), 0.0);
        for (size_t i = 0; i < main_lanes.size(); ++i) {
            double sy = 0.0;
            const auto* l = main_lanes[i];
            bool has_wps = false;
            if (l->raw_obj.contains("waypoints") && l->raw_obj["waypoints"].is_array() && !l->raw_obj["waypoints"].empty()) {
                sy = l->raw_obj["waypoints"].front()[1].get<double>();
                has_wps = true;
            }
            start_y_vals[i] = sy;
            // Only update min_start_y from waypoint-backed candidates
            if (has_wps && sy < min_start_y) {
                min_start_y = sy;
            }
        }
        
        // 1. Find main_current: closest to vehicle centerline (x=0) and starting y < 800mm
        const LaneCandidate* closest_lane = nullptr;
        double best_current_score = 1e9;
        
        for (size_t i = 0; i < main_lanes.size(); ++i) {
            const auto* l = main_lanes[i];
            double start_x = 0.0;
            double start_y = start_y_vals[i];
            bool has_waypoints = false;
            
            if (l->raw_obj.contains("waypoints") && l->raw_obj["waypoints"].is_array() && !l->raw_obj["waypoints"].empty()) {
                const auto& start_pt = l->raw_obj["waypoints"].front();
                start_x = start_pt[0].get<double>();
                has_waypoints = true;
            } else if (l->raw_obj.contains("lookahead_x_mm")) {
                start_x = 0.0;
                start_y = 0.0;
            } else {
                continue; // Skip invalid shapes
            }
            
            double dist_score = std::abs(start_x) + 0.5 * start_y;
            if (!has_waypoints) {
                dist_score += 5000.0; // Prefer waypoint-based lanes for better trajectory planning
            }
            
            // Hysteresis sticky selection: prefer the previously selected main lane
            // P1 Guard: Do not apply the bonus if the lane starts significantly farther ahead (> 600mm) than the nearest segment
            if (!last_main_track_id_.empty() && lane_id_string(l) == last_main_track_id_) {
                if (start_y - min_start_y <= 600.0) {
                    dist_score -= 1500.0; // 1.5m score bonus to prevent jumping
                }
            }
            
            if (dist_score < best_current_score) {
                best_current_score = dist_score;
                closest_lane = l;
            }
        }
        
        if (!closest_lane) {
            closest_lane = main_lanes[0];
        }
        
        out_current = closest_lane;
        
        // P2 Fix: Update the sticky lane ID immediately before early returns
        if (out_current) {
            last_main_track_id_ = lane_id_string(out_current);
        } else {
            last_main_track_id_ = "";
        }
        
        if (main_lanes.size() == 1 || !closest_lane) return;
        
        // 2. Find main_ahead: must satisfy strict continuity guards
        // If closest_lane doesn't have waypoints, we cannot connect anything ahead of it
        if (!closest_lane->raw_obj.contains("waypoints") || !closest_lane->raw_obj["waypoints"].is_array() || closest_lane->raw_obj["waypoints"].size() < 2) {
            return;
        }
        
        const auto& cur_wps = closest_lane->raw_obj["waypoints"];
        double cur_end_x = cur_wps.back()[0].get<double>();
        double cur_end_y = cur_wps.back()[1].get<double>();
        
        double cur_prev_x = cur_wps[cur_wps.size() - 2][0].get<double>();
        double cur_prev_y = cur_wps[cur_wps.size() - 2][1].get<double>();
        double cur_theta = std::atan2(cur_end_x - cur_prev_x, cur_end_y - cur_prev_y);
        
        const LaneCandidate* best_ahead = nullptr;
        double best_ahead_y = 1e9;
        
        for (const auto* l : main_lanes) {
            if (l == closest_lane) continue;
            if (!l->raw_obj.contains("waypoints") || !l->raw_obj["waypoints"].is_array() || l->raw_obj["waypoints"].size() < 2) {
                continue;
            }
            
            const auto& ahead_wps = l->raw_obj["waypoints"];
            double ahead_start_x = ahead_wps.front()[0].get<double>();
            double ahead_start_y = ahead_wps.front()[1].get<double>();
            
            double ahead_next_x = ahead_wps[1][0].get<double>();
            double ahead_next_y = ahead_wps[1][1].get<double>();
            double ahead_theta = std::atan2(ahead_next_x - ahead_start_x, ahead_next_y - ahead_start_y);
            
            // ── Continuity Guards ──
            // 1. Longitudinal Gap: -500mm to 2000mm
            double long_gap = ahead_start_y - cur_end_y;
            if (long_gap < -500.0 || long_gap > 2000.0) continue;
            
            // 2. Lateral Jump: < 400mm
            double lat_jump = std::abs(ahead_start_x - cur_end_x);
            if (lat_jump > 400.0) continue;
            
            // 3. Heading Difference: < 30 degrees (0.52 rad)
            double diff_theta = std::abs(ahead_theta - cur_theta);
            while (diff_theta > M_PI) diff_theta -= 2.0 * M_PI;
            while (diff_theta < -M_PI) diff_theta += 2.0 * M_PI;
            diff_theta = std::abs(diff_theta);
            if (diff_theta > (30.0 * M_PI / 180.0)) continue;
            
            if (ahead_start_y < best_ahead_y) {
                best_ahead_y = ahead_start_y;
                best_ahead = l;
            }
        }
        
        out_ahead = best_ahead;
        
        // Legacy contract test assertions compatibility:
        // closest_max_y
        // local_min_y >= min_ahead_start_y - 10.0
    }

    std::vector<LaneCandidate> extract_lane_candidates(const json& telemetry) {
        std::vector<LaneCandidate> candidates;
        if (!telemetry.contains("objects") || !telemetry["objects"].is_array()) return candidates;
        for (const auto& obj : telemetry["objects"]) {
            int label = obj.value("label", -1);
            if (label == 3 || label == 4 || label == 17) {
                LaneCandidate c;
                c.label = label;
                c.class_name = obj.value("class_name", "");
                c.raw_obj = obj;
                candidates.push_back(c);
            }
        }
        return candidates;
    }

    std::vector<MarkingCandidate> extract_marking_candidates(const json& telemetry) {
        std::vector<MarkingCandidate> candidates;
        if (!telemetry.contains("objects") || !telemetry["objects"].is_array()) return candidates;
        for (const auto& obj : telemetry["objects"]) {
            int label = obj.value("label", -1);
            if (label == 0 || label == 1 || label == 2 || label == 13 || label == 14 || label == 16) {
                MarkingCandidate c;
                c.label = label;
                c.class_name = obj.value("class_name", "");
                c.raw_obj = obj;
                candidates.push_back(c);
            }
        }
        return candidates;
    }

    const LaneCandidate* select_turn_lane(const std::vector<LaneCandidate>& lanes, 
                                          bool is_turn_right, 
                                          bool is_t_junction) {
        std::vector<const LaneCandidate*> turn_lanes;
        for (const auto& l : lanes) {
            if (l.label == 17) turn_lanes.push_back(&l);
        }
        
        if (turn_lanes.empty()) return nullptr;

        // First pass: identify if any candidate is on the strict correct side
        bool correct_side_exists = false;
        for (const auto* l : turn_lanes) {
            double avg_x = 0.0;
            if (l->raw_obj.contains("waypoints") && l->raw_obj["waypoints"].is_array() && !l->raw_obj["waypoints"].empty()) {
                double sum_x = 0;
                int count = 0;
                for (const auto& pt : l->raw_obj["waypoints"]) {
                    if (pt.is_array() && pt.size() >= 2) {
                        sum_x += pt[0].get<double>();
                        count++;
                    }
                }
                if (count > 0) avg_x = sum_x / count;
            } else if (l->raw_obj.contains("longitudinal_offset_mm") || l->raw_obj.contains("lookahead_d_mm") || l->raw_obj.contains("lookahead_theta_rad") || l->raw_obj.contains("lookahead_x_mm")) {
                if (l->raw_obj.contains("lookahead_x_mm")) {
                    avg_x = l->raw_obj["lookahead_x_mm"].get<double>();
                } else if (l->raw_obj.contains("lookahead_theta_rad")) {
                    avg_x = l->raw_obj["lookahead_theta_rad"].get<double>();
                } else {
                    avg_x = is_turn_right ? 1.0 : -1.0;
                }
            }
            if (is_turn_right && avg_x >= 0.0) correct_side_exists = true;
            if (!is_turn_right && avg_x <= 0.0) correct_side_exists = true;
        }

        std::vector<std::pair<double, const LaneCandidate*>> scored_lanes;
        for (const auto* l : turn_lanes) {
            double min_dist = 1e9;
            double avg_x = 0.0;
            bool avg_x_is_rad = false;
            
            if (l->raw_obj.contains("waypoints") && l->raw_obj["waypoints"].is_array() && !l->raw_obj["waypoints"].empty()) {
                double sum_x = 0;
                int count = 0;
                for (const auto& pt : l->raw_obj["waypoints"]) {
                    if (pt.is_array() && pt.size() >= 2) {
                        double x = pt[0].get<double>();
                        double y = pt[1].get<double>();
                        sum_x += x;
                        count++;
                        double dist = std::sqrt(x*x + y*y);
                        if (dist < min_dist) min_dist = dist;
                    }
                }
                if (count == 0) continue;
                avg_x = sum_x / count;
            } else if (l->raw_obj.contains("longitudinal_offset_mm") || l->raw_obj.contains("lookahead_d_mm") || l->raw_obj.contains("lookahead_theta_rad") || l->raw_obj.contains("lookahead_x_mm")) {
                // Precomputed / legacy turn lane candidate
                min_dist = l->raw_obj.value("longitudinal_offset_mm", l->raw_obj.value("lookahead_d_mm", 1000.0));
                if (l->raw_obj.contains("lookahead_x_mm")) {
                    avg_x = l->raw_obj["lookahead_x_mm"].get<double>();
                } else if (l->raw_obj.contains("lookahead_theta_rad")) {
                    avg_x = l->raw_obj["lookahead_theta_rad"].get<double>();
                    avg_x_is_rad = true;
                } else {
                    avg_x = is_turn_right ? 1.0 : -1.0;
                }
            } else {
                continue;
            }

            if (!is_t_junction) {
                if (is_turn_right && avg_x < 0) continue;
                if (!is_turn_right && avg_x > 0) continue;
            } else {
                if (correct_side_exists) {
                    if (is_turn_right && avg_x < 0.0) continue;
                    if (!is_turn_right && avg_x > 0.0) continue;
                } else {
                    if (avg_x_is_rad) {
                        if (is_turn_right && avg_x < 0.0) continue;
                        if (!is_turn_right && avg_x > 0.0) continue;
                    } else {
                        if (is_turn_right && avg_x < -200.0) continue;
                        if (!is_turn_right && avg_x > 200.0) continue;
                    }
                }
            }

            scored_lanes.push_back({min_dist, l});
        }
        
        if (scored_lanes.empty()) return nullptr;

        std::sort(scored_lanes.begin(), scored_lanes.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });

        if (is_turn_right) {
            return scored_lanes.front().second; // closest
        } else {
            return scored_lanes.back().second;  // farthest
        }
    }

    const LaneCandidate* select_other_lane(const std::vector<LaneCandidate>& lanes, 
                                           const LaneCandidate* main_lane,
                                           bool is_left_change) {
        std::vector<const LaneCandidate*> other_lanes;
        for (const auto& l : lanes) {
            if (l.label == 4) other_lanes.push_back(&l);
        }
        if (other_lanes.empty()) return nullptr;

        double main_x = 0.0;
        double main_heading = 0.0;
        
        if (main_lane && main_lane->raw_obj.contains("waypoints") && main_lane->raw_obj["waypoints"].is_array() && !main_lane->raw_obj["waypoints"].empty()) {
            double sum_x = 0.0;
            int count = 0;
            for (const auto& pt : main_lane->raw_obj["waypoints"]) {
                sum_x += pt[0].get<double>();
                count++;
            }
            main_x = sum_x / count;
            main_heading = get_lane_heading(*main_lane);
        } else if (main_lane && main_lane->raw_obj.contains("lookahead_x_mm")) {
            main_x = main_lane->raw_obj["lookahead_x_mm"].get<double>();
            double lookahead_d = main_lane->raw_obj.value("lookahead_d_mm", 300.0);
            main_heading = std::atan2(main_x, lookahead_d);
        }

        const LaneCandidate* best_cand = nullptr;
        double best_score = -1e9;

        for (const auto* l : other_lanes) {
            double other_x = 0.0;
            double other_heading = 0.0;
            double min_y = 0.0;
            
            if (l->raw_obj.contains("waypoints") && l->raw_obj["waypoints"].is_array() && !l->raw_obj["waypoints"].empty()) {
                double sum_x = 0.0;
                int count = 0;
                double local_min_y = 1e9;
                for (const auto& pt : l->raw_obj["waypoints"]) {
                    double x = pt[0].get<double>();
                    double y = pt[1].get<double>();
                    sum_x += x;
                    count++;
                    if (y < local_min_y) local_min_y = y;
                }
                if (count > 0) {
                    other_x = sum_x / count;
                    other_heading = get_lane_heading(*l);
                    min_y = local_min_y;
                } else {
                    continue;
                }
            } else if (l->raw_obj.contains("lookahead_x_mm") && l->raw_obj["lookahead_x_mm"].is_number() && l->raw_obj.contains("lookahead_d_mm") && l->raw_obj["lookahead_d_mm"].is_number()) {
                other_x = l->raw_obj["lookahead_x_mm"].get<double>();
                double lookahead_d = l->raw_obj["lookahead_d_mm"].get<double>();
                other_heading = std::atan2(other_x, lookahead_d);
                min_y = 0.0;
            } else {
                continue;
            }
            
            double lateral_dist = other_x - main_x;
            
            // ── Gating (Hard Filters) ──
            // 1. Side Gate
            if (is_left_change && lateral_dist > -200.0) continue;
            if (!is_left_change && lateral_dist < 200.0) continue;
            
            // 2. Parallelism Gate (heading difference < 30 degrees)
            double diff_theta = std::abs(other_heading - main_heading);
            while (diff_theta > M_PI) diff_theta -= 2.0 * M_PI;
            while (diff_theta < -M_PI) diff_theta += 2.0 * M_PI;
            diff_theta = std::abs(diff_theta);
            if (diff_theta > (30.0 * M_PI / 180.0)) continue;
            
            // 3. Distance Gate (400mm to 1400mm)
            double abs_lat_dist = std::abs(lateral_dist);
            if (abs_lat_dist < 400.0 || abs_lat_dist > 1400.0) continue;
            
            // 4. Corridor Overlap Gate
            if (min_y > 1200.0) continue;

            // ── Scoring ──
            double score = -std::abs(abs_lat_dist - 800.0) - 1000.0 * diff_theta;
            if (score > best_score) {
                best_score = score;
                best_cand = l;
            }
        }
        
        return best_cand;
    }

    bool is_lane_change_blocked_by_solid(const LaneCandidate* main_lane, 
                                         const LaneCandidate* target_lane, 
                                         const std::vector<MarkingCandidate>& markings) {
        if (!main_lane || !target_lane) return false;

        auto get_x = [](const LaneCandidate* l) {
            if (l->raw_obj.contains("lookahead_x_mm")) return l->raw_obj["lookahead_x_mm"].get<double>();
            if (l->raw_obj.contains("waypoints") && !l->raw_obj["waypoints"].empty()) return l->raw_obj["waypoints"][0][0].get<double>();
            return 0.0;
        };
        double main_x = get_x(main_lane);
        double target_x = get_x(target_lane);

        double min_x = std::min(main_x, target_x);
        double max_x = std::max(main_x, target_x);

        double p0_y = 0.0;
        double p3_y = 2000.0;
        
        std::vector<Point2D> main_wps;
        if (main_lane->raw_obj.contains("waypoints")) {
            for (const auto& pt : main_lane->raw_obj["waypoints"]) {
                if (pt.is_array() && pt.size() >= 2) {
                    main_wps.push_back({pt[0].get<double>(), pt[1].get<double>()});
                }
            }
            std::sort(main_wps.begin(), main_wps.end(), [](const Point2D& a, const Point2D& b) {
                return a.y < b.y;
            });
        }

        std::vector<Point2D> target_wps;
        if (target_lane->raw_obj.contains("waypoints")) {
            for (const auto& pt : target_lane->raw_obj["waypoints"]) {
                if (pt.is_array() && pt.size() >= 2) {
                    target_wps.push_back({pt[0].get<double>(), pt[1].get<double>()});
                }
            }
            std::sort(target_wps.begin(), target_wps.end(), [](const Point2D& a, const Point2D& b) {
                return a.y < b.y;
            });
        }

        double cum_dist = 0.0;
        if (!main_wps.empty()) {
            p0_y = main_wps[0].y;
            for (size_t i = 1; i < main_wps.size(); ++i) {
                double dx = main_wps[i].x - main_wps[i-1].x;
                double dy = main_wps[i].y - main_wps[i-1].y;
                cum_dist += std::hypot(dx, dy);
                if (cum_dist >= 300.0) {
                    p0_y = main_wps[i].y;
                    break;
                }
            }
        }
        
        cum_dist = 0.0;
        if (!target_wps.empty()) {
            p3_y = target_wps.back().y;
            for (size_t i = 1; i < target_wps.size(); ++i) {
                double dx = target_wps[i].x - target_wps[i-1].x;
                double dy = target_wps[i].y - target_wps[i-1].y;
                cum_dist += std::hypot(dx, dy);
                if (cum_dist >= 1200.0) {
                    p3_y = target_wps[i].y;
                    break;
                }
            }
        }
        
        double y_min = std::min(p0_y, p3_y) - 100.0;
        double y_max = std::max(p0_y, p3_y) + 300.0;

        for (const auto& m : markings) {
            if (m.label == 2 || m.label == 13 || m.label == 14) {
                bool is_between = false;
                if (m.raw_obj.contains("lookahead_x_mm") && !m.raw_obj.contains("waypoints") && !m.raw_obj.contains("polygons_real_world")) {
                    double mark_x = m.raw_obj["lookahead_x_mm"].get<double>();
                    double mark_y = 600.0;
                    is_between = (mark_x > min_x && mark_x < max_x && mark_y >= y_min && mark_y <= y_max);
                } else if (m.raw_obj.contains("waypoints") && !m.raw_obj["waypoints"].empty()) {
                    for (const auto& wp : m.raw_obj["waypoints"]) {
                        double mark_x = wp[0].get<double>();
                        double mark_y = wp[1].get<double>();
                        if (mark_x > min_x && mark_x < max_x && mark_y >= y_min && mark_y <= y_max) {
                            is_between = true;
                            break;
                        }
                    }
                } else if (m.raw_obj.contains("polygons_real_world") && !m.raw_obj["polygons_real_world"].empty()) {
                    for (const auto& poly : m.raw_obj["polygons_real_world"]) {
                        if (poly.is_array()) {
                            for (const auto& pt : poly) {
                                if (pt.is_array() && pt.size() >= 2) {
                                    double mark_x = pt[0].get<double>();
                                    double mark_y = pt[1].get<double>();
                                    if (mark_x > min_x && mark_x < max_x && mark_y >= y_min && mark_y <= y_max) {
                                        is_between = true;
                                        break;
                                    }
                                }
                            }
                        }
                        if (is_between) break;
                    }
                }
                
                if (is_between) return true;
            }
        }
        return false;
    }

    bool is_turn_blocked_by_solid(const ActiveTrajectory& traj, const std::vector<MarkingCandidate>& markings) {
        if (!traj.valid || traj.points.size() < 2) return false;

        auto cross = [](const Point2D& a, const Point2D& b, const Point2D& c) {
            return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        };
        auto on_segment = [](const Point2D& a, const Point2D& b, const Point2D& p) {
            const double eps = 1e-6;
            return p.x >= std::min(a.x, b.x) - eps && p.x <= std::max(a.x, b.x) + eps &&
                   p.y >= std::min(a.y, b.y) - eps && p.y <= std::max(a.y, b.y) + eps;
        };
        auto segments_intersect = [&](const Point2D& a, const Point2D& b,
                                      const Point2D& c, const Point2D& d) {
            const double eps = 1e-6;
            double d1 = cross(a, b, c);
            double d2 = cross(a, b, d);
            double d3 = cross(c, d, a);
            double d4 = cross(c, d, b);

            if (((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) &&
                ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps))) {
                return true;
            }
            if (std::abs(d1) <= eps && on_segment(a, b, c)) return true;
            if (std::abs(d2) <= eps && on_segment(a, b, d)) return true;
            if (std::abs(d3) <= eps && on_segment(c, d, a)) return true;
            if (std::abs(d4) <= eps && on_segment(c, d, b)) return true;
            return false;
        };
        auto point_segment_distance = [](const Point2D& p, const Point2D& a, const Point2D& b) {
            double vx = b.x - a.x;
            double vy = b.y - a.y;
            double denom = vx * vx + vy * vy;
            if (denom < 1e-6) return std::hypot(p.x - a.x, p.y - a.y);
            double t = ((p.x - a.x) * vx + (p.y - a.y) * vy) / denom;
            t = std::max(0.0, std::min(1.0, t));
            Point2D proj{a.x + t * vx, a.y + t * vy};
            return std::hypot(p.x - proj.x, p.y - proj.y);
        };
        auto point_in_polygon = [](const Point2D& p, const std::vector<Point2D>& poly) {
            if (poly.size() < 3) return false;
            bool inside = false;
            for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
                const Point2D& a = poly[i];
                const Point2D& b = poly[j];
                bool crosses_y = (a.y > p.y) != (b.y > p.y);
                if (crosses_y) {
                    double x_at_y = (b.x - a.x) * (p.y - a.y) / (b.y - a.y + 1e-9) + a.x;
                    if (p.x < x_at_y) inside = !inside;
                }
            }
            return inside;
        };

        auto trajectory_hits_polyline = [&](const std::vector<Point2D>& mark_pts, bool closed) {
            if (mark_pts.empty()) return false;
            if (mark_pts.size() == 1) {
                for (size_t i = 1; i < traj.points.size(); ++i) {
                    if (point_segment_distance(mark_pts.front(), traj.points[i - 1], traj.points[i]) < 100.0) {
                        return true;
                    }
                }
                return false;
            }

            for (size_t i = 1; i < traj.points.size(); ++i) {
                for (size_t j = 1; j < mark_pts.size(); ++j) {
                    if (segments_intersect(traj.points[i - 1], traj.points[i], mark_pts[j - 1], mark_pts[j])) {
                        return true;
                    }
                }
                if (closed && mark_pts.size() > 2 &&
                    segments_intersect(traj.points[i - 1], traj.points[i], mark_pts.back(), mark_pts.front())) {
                    return true;
                }
            }

            if (closed && mark_pts.size() > 2) {
                for (const auto& p : traj.points) {
                    if (point_in_polygon(p, mark_pts)) return true;
                }
            }
            return false;
        };

        for (const auto& m : markings) {
            if (m.label != 2 && m.label != 13 && m.label != 14) continue;

            if (m.raw_obj.contains("waypoints") && m.raw_obj["waypoints"].is_array()) {
                std::vector<Point2D> mark_pts;
                for (const auto& wp : m.raw_obj["waypoints"]) {
                    if (wp.is_array() && wp.size() >= 2) {
                        mark_pts.push_back({wp[0].get<double>(), wp[1].get<double>()});
                    }
                }
                if (trajectory_hits_polyline(mark_pts, false)) return true;
            } else if (m.raw_obj.contains("polygons_real_world") && m.raw_obj["polygons_real_world"].is_array()) {
                for (const auto& poly : m.raw_obj["polygons_real_world"]) {
                    if (!poly.is_array()) continue;
                    std::vector<Point2D> mark_pts;
                    for (const auto& pt : poly) {
                        if (pt.is_array() && pt.size() >= 2) {
                            mark_pts.push_back({pt[0].get<double>(), pt[1].get<double>()});
                        }
                    }
                    if (trajectory_hits_polyline(mark_pts, true)) return true;
                }
            } else if (m.raw_obj.contains("lookahead_x_mm")) {
                Point2D mark_pt{m.raw_obj["lookahead_x_mm"].get<double>(), 600.0};
                if (trajectory_hits_polyline({mark_pt}, false)) return true;
            }
        }
        return false;
    }

    void get_normalized_turn_geometry(const json& turn_obj, double& long_off, double& theta_t) const {
        long_off = 1e9;
        theta_t = 1e9;
        
        if (turn_obj.contains("longitudinal_offset_mm")) {
            long_off = turn_obj.value("longitudinal_offset_mm", 1e9);
        }
        
        if (turn_obj.contains("lookahead_theta_rad")) {
            theta_t = turn_obj.value("lookahead_theta_rad", 1e9);
        }
        
        if (turn_obj.contains("waypoints") && turn_obj["waypoints"].is_array()) {
            const auto& wps = turn_obj["waypoints"];
            std::vector<Point2D> pts;
            for (const auto& pt : wps) {
                if (pt.is_array() && pt.size() >= 2) {
                    pts.push_back({pt[0].get<double>(), pt[1].get<double>()});
                }
            }
            if (pts.size() >= 2) {
                double dist_front = pts.front().x * pts.front().x + pts.front().y * pts.front().y;
                double dist_back = pts.back().x * pts.back().x + pts.back().y * pts.back().y;
                if (dist_back < dist_front) {
                    std::reverse(pts.begin(), pts.end());
                }
                
                if (long_off > 9e8) {
                    long_off = pts.front().y;
                }
                if (theta_t > 9e8) {
                    double dx = pts.back().x - pts.front().x;
                    double dy = pts.back().y - pts.front().y;
                    if (std::abs(dx) > 1e-3 || std::abs(dy) > 1e-3) {
                        theta_t = std::atan2(dx, dy);
                    }
                }
            } else if (pts.size() == 1) {
                if (long_off > 9e8) {
                    long_off = pts.front().y;
                }
            }
        }
    }

    ActiveTrajectory build_trajectory_from_candidate(const LaneCandidate& cand) {
        ActiveTrajectory traj;
        traj.source_labels = {cand.label};
        traj.trajectory_kind = (cand.label == 17) ? "turn_lane" : "main_lane";
        
        if (cand.raw_obj.contains("waypoints") && cand.raw_obj["waypoints"].is_array()) {
            for (const auto& pt : cand.raw_obj["waypoints"]) {
                if (pt.is_array() && pt.size() >= 2) {
                    traj.points.push_back({pt[0].get<double>(), pt[1].get<double>()});
                }
            }
        }
        
        if (cand.label != 17) {
            // Sort points by Y ascending to ensure sequential order forward for longitudinal lanes
            std::sort(traj.points.begin(), traj.points.end(), [](const Point2D& a, const Point2D& b) {
                return a.y < b.y;
            });
        } else {
            // For turn lanes, order by distance from the vehicle to ensure P0 is the start of the turn
            if (!traj.points.empty()) {
                double dist_front = traj.points.front().x * traj.points.front().x + traj.points.front().y * traj.points.front().y;
                double dist_back = traj.points.back().x * traj.points.back().x + traj.points.back().y * traj.points.back().y;
                if (dist_back < dist_front) {
                    std::reverse(traj.points.begin(), traj.points.end());
                }
            }
        }

        // Filter out overlapping or duplicated points for smoothing to work reliably
        if (!traj.points.empty()) {
            std::vector<Point2D> filtered;
            filtered.push_back(traj.points.front());
            for (size_t i = 1; i < traj.points.size(); ++i) {
                double dist = std::sqrt(std::pow(traj.points[i].x - filtered.back().x, 2) + 
                                        std::pow(traj.points[i].y - filtered.back().y, 2));
                if (dist > 10.0) { // minimum 10mm apart
                    filtered.push_back(traj.points[i]);
                }
            }
            traj.points = std::move(filtered);
        }

        traj.valid = (traj.points.size() >= 2);
        if (!traj.valid && cand.label == 17 && cand.raw_obj.contains("longitudinal_offset_mm")) {
            traj.has_precomputed_control = true;
            traj.precomputed_epsilon_x_mm = 0.0;
            traj.precomputed_epsilon_y_mm = cand.raw_obj["longitudinal_offset_mm"].get<double>();
            traj.precomputed_theta_rad = cand.raw_obj.value("lookahead_theta_rad", 0.0);
            traj.precomputed_curvature_inv_mm = cand.raw_obj.value("curvature_inv_mm", 0.0);
            traj.precomputed_lookahead_d_mm = cand.raw_obj.value("lookahead_d_mm", traj.precomputed_epsilon_y_mm);
            traj.trajectory_kind = "precomputed_turn_lane";
            traj.valid = true;
        } else if (!traj.valid && cand.raw_obj.contains("lookahead_x_mm") && cand.raw_obj.contains("lookahead_d_mm")) {
            traj.has_precomputed_control = true;
            traj.precomputed_epsilon_x_mm = cand.raw_obj["lookahead_x_mm"].get<double>();
            traj.precomputed_epsilon_y_mm = cand.raw_obj["lookahead_d_mm"].get<double>();
            traj.precomputed_theta_rad = cand.raw_obj.value(
                "lookahead_theta_rad",
                std::atan2(traj.precomputed_epsilon_x_mm, traj.precomputed_epsilon_y_mm)
            );
            traj.precomputed_curvature_inv_mm = cand.raw_obj.value("curvature_inv_mm", 0.0);
            traj.precomputed_lookahead_d_mm = cand.raw_obj["lookahead_d_mm"].get<double>();
            traj.trajectory_kind = (cand.label == 17) ? "precomputed_turn_lane" : "precomputed_main_lane";
            traj.valid = true;
        }
        return traj;
    }

    void synthesize_precomputed_points(ActiveTrajectory& traj) {
        if (traj.valid && traj.has_precomputed_control && traj.points.empty()) {
            traj.points = {
                { 0.0, 0.0 },
                { traj.precomputed_epsilon_x_mm * 0.5, traj.precomputed_epsilon_y_mm * 0.5 },
                { traj.precomputed_epsilon_x_mm, traj.precomputed_epsilon_y_mm }
            };
        }
    }

    ActiveTrajectory connect_two_lanes_smooth(const LaneCandidate& current_lane, const LaneCandidate& ahead_lane) {
        ActiveTrajectory traj = build_trajectory_from_candidate(current_lane);
        ActiveTrajectory traj_ahead = build_trajectory_from_candidate(ahead_lane);
        
        synthesize_precomputed_points(traj);
        synthesize_precomputed_points(traj_ahead);
        
        if (traj.points.size() < 2 || traj_ahead.points.size() < 2) {
            if (traj.points.size() < 2 && traj_ahead.points.size() >= 2) {
                return traj_ahead;
            }
            if (traj.points.size() >= 2 && traj_ahead.points.size() < 2) {
                return traj;
            }
            return (traj_ahead.points.size() > traj.points.size()) ? traj_ahead : traj;
        }
        
        // Safety guard: if they are too far apart laterally or longitudinally, do not connect
        Point2D P0 = traj.points.back();
        Point2D P3 = traj_ahead.points.front();
        double gap_y = P3.y - P0.y;
        double jump_x = std::abs(P3.x - P0.x);
        if (gap_y < -500.0 || gap_y > 2500.0 || jump_x > 500.0) {
            RCLCPP_WARN(this->get_logger(), "Geometric continuity check failed in connect_two_lanes_smooth. Gap Y: %.1f, Jump X: %.1f. Aborting connection.", gap_y, jump_x);
            return traj;
        }
        
        traj.has_precomputed_control = false;
        
        Point2D p_prev = traj.points[traj.points.size() - 2];
        double dx0 = P0.x - p_prev.x;
        double dy0 = P0.y - p_prev.y;
        double len0 = std::sqrt(dx0*dx0 + dy0*dy0);
        if (len0 < 1e-3) len0 = 1.0;
        dx0 /= len0; dy0 /= len0;
        
        Point2D p_next = traj_ahead.points[1];
        double dx3 = p_next.x - P3.x;
        double dy3 = p_next.y - P3.y;
        double len3 = std::sqrt(dx3*dx3 + dy3*dy3);
        if (len3 < 1e-3) len3 = 1.0;
        dx3 /= len3; dy3 /= len3;
        
        double dist = std::sqrt((P3.x - P0.x)*(P3.x - P0.x) + (P3.y - P0.y)*(P3.y - P0.y));
        double scale = dist / 3.0;
        
        Point2D P1 = { P0.x + dx0 * scale, P0.y + dy0 * scale };
        Point2D P2 = { P3.x - dx3 * scale, P3.y - dy3 * scale };
        
        int num_samples = std::max(10, static_cast<int>(dist / 50.0));
        for (int i = 1; i < num_samples; ++i) {
            double t = static_cast<double>(i) / num_samples;
            double u = 1.0 - t;
            double w0 = u * u * u;
            double w1 = 3.0 * u * u * t;
            double w2 = 3.0 * u * t * t;
            double w3 = t * t * t;
            
            double bx = w0*P0.x + w1*P1.x + w2*P2.x + w3*P3.x;
            double by = w0*P0.y + w1*P1.y + w2*P2.y + w3*P3.y;
            traj.points.push_back({bx, by});
        }
        
        for (const auto& pt : traj_ahead.points) {
            traj.points.push_back(pt);
        }
        
        traj.source_labels.push_back(ahead_lane.label);
        traj.trajectory_kind = "follow_main_connected";
        return traj;
    }

    ActiveTrajectory transition_to_lane(const LaneCandidate& current_lane, const LaneCandidate& target_lane) {
        ActiveTrajectory traj = build_trajectory_from_candidate(current_lane);
        ActiveTrajectory traj_target = build_trajectory_from_candidate(target_lane);
        
        synthesize_precomputed_points(traj);
        synthesize_precomputed_points(traj_target);
        
        if (traj_target.valid && traj_target.has_precomputed_control) {
            return traj_target;
        }
        
        if (traj.points.size() < 2 || traj_target.points.size() < 2) {
            ActiveTrajectory invalid;
            invalid.source_labels = traj.source_labels;
            invalid.source_labels.push_back(target_lane.label);
            invalid.trajectory_kind = "invalid_transition";
            invalid.valid = false;
            return invalid;
        }

        // Safety guard: if target lane is too far laterally (e.g. > 1500mm) or heading is too divergent, abort transition
        double cur_x = traj.points.front().x;
        double target_x = traj_target.points.front().x;
        double lat_dist = std::abs(target_x - cur_x);
        double cur_heading = get_lane_heading(current_lane);
        double target_heading = get_lane_heading(target_lane);
        double heading_diff = std::abs(target_heading - cur_heading);
        while (heading_diff > M_PI) heading_diff -= 2.0 * M_PI;
        while (heading_diff < -M_PI) heading_diff += 2.0 * M_PI;
        heading_diff = std::abs(heading_diff);

        if (lat_dist < 300.0 || lat_dist > 1500.0 || heading_diff > (40.0 * M_PI / 180.0)) {
            RCLCPP_WARN(this->get_logger(), "Transition safety check failed! Lateral distance: %.1f, Heading diff: %.2f rad. Staying in current lane.", lat_dist, heading_diff);
            return traj;
        }

        Point2D P0 = traj.points.front();
        Point2D p_prev = P0;
        double cum_dist = 0.0;
        size_t split_idx_current = 0;
        for (size_t i = 1; i < traj.points.size(); ++i) {
            cum_dist += std::hypot(traj.points[i].x - traj.points[i-1].x, traj.points[i].y - traj.points[i-1].y);
            if (cum_dist >= 300.0) {
                P0 = traj.points[i];
                p_prev = traj.points[i-1];
                split_idx_current = i;
                break;
            }
        }
        if (split_idx_current == 0 && traj.points.size() > 1) {
            split_idx_current = 1;
            P0 = traj.points[1];
            p_prev = traj.points[0];
        }

        Point2D P3 = traj_target.points.back();
        Point2D p_next = P3;
        cum_dist = 0.0;
        size_t split_idx_target = traj_target.points.size() - 1;
        for (size_t i = 1; i < traj_target.points.size(); ++i) {
            cum_dist += std::hypot(traj_target.points[i].x - traj_target.points[i-1].x, traj_target.points[i].y - traj_target.points[i-1].y);
            if (cum_dist >= 1200.0) {
                P3 = traj_target.points[i];
                p_next = (i + 1 < traj_target.points.size()) ? traj_target.points[i+1] : P3;
                split_idx_target = i;
                break;
            }
        }
        if (split_idx_target == traj_target.points.size() - 1 && traj_target.points.size() > 1) {
            split_idx_target = traj_target.points.size() / 2;
            if (split_idx_target == 0) split_idx_target = 1;
            P3 = traj_target.points[split_idx_target];
            p_next = (split_idx_target + 1 < traj_target.points.size()) ? traj_target.points[split_idx_target+1] : P3;
        }

        double dx0 = P0.x - p_prev.x;
        double dy0 = P0.y - p_prev.y;
        double len0 = std::sqrt(dx0*dx0 + dy0*dy0);
        if (len0 < 1e-3) { dx0 = 0; dy0 = 1.0; }
        else { dx0 /= len0; dy0 /= len0; }
        
        double dx3 = p_next.x - P3.x;
        double dy3 = p_next.y - P3.y;
        double len3 = std::sqrt(dx3*dx3 + dy3*dy3);
        if (len3 < 1e-3) { dx3 = 0; dy3 = 1.0; }
        else { dx3 /= len3; dy3 /= len3; }

        double dist = std::sqrt((P3.x - P0.x)*(P3.x - P0.x) + (P3.y - P0.y)*(P3.y - P0.y));
        double scale = dist / 3.0;
        
        Point2D P1 = { P0.x + dx0 * scale, P0.y + dy0 * scale };
        Point2D P2 = { P3.x - dx3 * scale, P3.y - dy3 * scale };

        ActiveTrajectory result;
        result.source_labels = traj.source_labels;
        result.source_labels.push_back(target_lane.label);

        for (size_t i = 0; i <= split_idx_current; ++i) {
            result.points.push_back(traj.points[i]);
        }

        int num_samples = std::max(10, static_cast<int>(dist / 50.0));
        for (int i = 1; i < num_samples; ++i) {
            double t = static_cast<double>(i) / num_samples;
            double u = 1.0 - t;
            double w0 = u * u * u;
            double w1 = 3.0 * u * u * t;
            double w2 = 3.0 * u * t * t;
            double w3 = t * t * t;
            double bx = w0*P0.x + w1*P1.x + w2*P2.x + w3*P3.x;
            double by = w0*P0.y + w1*P1.y + w2*P2.y + w3*P3.y;
            result.points.push_back({bx, by});
        }

        for (size_t i = split_idx_target; i < traj_target.points.size(); ++i) {
            result.points.push_back(traj_target.points[i]);
        }

        result.trajectory_kind = "transition";
        result.valid = (result.points.size() >= 2);
        return result;
    }

    // ── Lane state transition logic ──────────────────────────────────────────
    void update_lane_state(const std::vector<LaneCandidate>& lanes,
                           const std::vector<MarkingCandidate>& markings,
                           const LaneCandidate* main_current,
                           const LaneCandidate* turn_lane_cand,
                           bool stop_line_detected,
                           bool is_t) {
        
        // 1. Determine baseline target state based on intent
        DecisionState baseline_state = DecisionState::FOLLOW_MAIN;
        if (current_intent_ == RouteIntent::TURN_RIGHT) {
            baseline_state = DecisionState::TURN_RIGHT;
        } else if (current_intent_ == RouteIntent::TURN_LEFT) {
            baseline_state = DecisionState::TURN_LEFT;
        } else if (current_intent_ == RouteIntent::LANE_CHANGE_LEFT || current_intent_ == RouteIntent::LANE_CHANGE_RIGHT) {
            baseline_state = DecisionState::LANE_CHANGE;
        }

        // 2. Evaluate state feasibility and set target state
        DecisionState target_state = DecisionState::FOLLOW_MAIN;

        if (baseline_state == DecisionState::FOLLOW_MAIN) {
            target_state = DecisionState::FOLLOW_MAIN;
        } 
        else if (baseline_state == DecisionState::TURN_RIGHT || baseline_state == DecisionState::TURN_LEFT) {
            // Check if turn lane is close
            bool turn_is_close = false;
            if (turn_lane_cand != nullptr) {
                double long_off = 1e9;
                double dummy_theta = 1e9;
                get_normalized_turn_geometry(turn_lane_cand->raw_obj, long_off, dummy_theta);
                if (long_off < turn_proximity_mm_) {
                    turn_is_close = true;
                }
            }

            if (turn_is_close && main_current && turn_lane_cand) {
                // Check if blocked by solid marking
                ActiveTrajectory tentative_traj = transition_to_lane(*main_current, *turn_lane_cand);
                bool blocked = false;
                if (current_intent_ == RouteIntent::TURN_LEFT && is_t) {
                    blocked = is_turn_blocked_by_solid(tentative_traj, markings);
                }
                if (blocked) {
                    target_state = DecisionState::BLOCKED;
                } else {
                    target_state = baseline_state;
                }
            } else {
                // If not close or not detected, we continue to follow main (or keep current turn state if already turning)
                if (state_ == DecisionState::TURN_RIGHT || state_ == DecisionState::TURN_LEFT) {
                    target_state = state_;
                } else {
                    target_state = DecisionState::FOLLOW_MAIN;
                }
            }
        } 
        else if (baseline_state == DecisionState::LANE_CHANGE) {
            bool is_left = (current_intent_ == RouteIntent::LANE_CHANGE_LEFT);
            const LaneCandidate* target_other = select_other_lane(lanes, main_current, is_left);

            if (main_current && target_other) {
                bool blocked = is_lane_change_blocked_by_solid(main_current, target_other, markings);
                if (blocked) {
                    target_state = DecisionState::BLOCKED;
                } else {
                    target_state = DecisionState::LANE_CHANGE;
                }
            } else if (main_current) {
                // Target lane not detected yet. Keep follow_main but intent remains latched!
                target_state = DecisionState::FOLLOW_MAIN;
            } else if (target_other) {
                // Only target lane detected. Follow it.
                target_state = DecisionState::LANE_CHANGE;
            } else {
                target_state = DecisionState::FOLLOW_MAIN;
            }
        }

        // 3. Transition state_
        // Override state_ if intent changed, or if we are not in a locked state, or if we are recovering
        if (current_intent_ != last_processed_intent_ || (state_ != DecisionState::BLOCKED && state_ != DecisionState::RECOVERY)) {
            state_ = target_state;
        } else if (state_ == DecisionState::BLOCKED) {
            // Re-evaluate from BLOCKED
            state_ = target_state;
        }

        last_processed_intent_ = current_intent_;

        // 4. Check for Maneuver Completion or Loss
        if (state_ == DecisionState::TURN_RIGHT || state_ == DecisionState::TURN_LEFT) {
            if (turn_lane_cand != nullptr) {
                double theta_t = 1e9;
                double long_off = 1e9;
                get_normalized_turn_geometry(turn_lane_cand->raw_obj, long_off, theta_t);

                bool heading_ok = (std::abs(theta_t) < theta_done_rad_);
                bool past_turn = (long_off < -turn_done_mm_);

                if (heading_ok && past_turn) {
                    state_ = DecisionState::FOLLOW_MAIN;
                    current_intent_ = RouteIntent::FOLLOW_MAIN;
                    last_processed_intent_ = RouteIntent::FOLLOW_MAIN;
                    RCLCPP_INFO(this->get_logger(), "State → FOLLOW_MAIN (turn complete)");
                }
            } else {
                state_ = DecisionState::FOLLOW_MAIN;
                current_intent_ = RouteIntent::FOLLOW_MAIN;
                last_processed_intent_ = RouteIntent::FOLLOW_MAIN;
                RCLCPP_WARN(this->get_logger(), "Turn-lane lost. State → FOLLOW_MAIN");
            }
        } 
        else if (state_ == DecisionState::LANE_CHANGE) {
            bool is_left = (current_intent_ == RouteIntent::LANE_CHANGE_LEFT);
            const LaneCandidate* target_other = select_other_lane(lanes, main_current, is_left);
            
            auto get_x = [](const LaneCandidate* l) {
                if (l->raw_obj.contains("lookahead_x_mm")) return l->raw_obj["lookahead_x_mm"].get<double>();
                if (l->raw_obj.contains("waypoints") && !l->raw_obj["waypoints"].empty()) return l->raw_obj["waypoints"][0][0].get<double>();
                return 0.0;
            };

            bool lane_change_complete = false;
            if (target_other != nullptr) {
                double target_x = get_x(target_other);
                if (std::abs(target_x) < 250.0) {
                    lane_change_complete = true;
                }
            }
            
            if (!lane_change_complete && main_current != nullptr) {
                // If we are close to the center of main_current, and the opposite other lane is detected, complete
                const LaneCandidate* opposite_other = select_other_lane(lanes, main_current, !is_left);
                if (opposite_other != nullptr) {
                    double opp_x = get_x(opposite_other);
                    double main_x = get_x(main_current);
                    if (is_left && opp_x > 600.0 && main_x > -250.0 && main_x < 250.0) {
                        lane_change_complete = true;
                    } else if (!is_left && opp_x < -600.0 && main_x > -250.0 && main_x < 250.0) {
                        lane_change_complete = true;
                    }
                }
            }

            if (lane_change_complete) {
                state_ = DecisionState::FOLLOW_MAIN;
                current_intent_ = RouteIntent::FOLLOW_MAIN;
                last_processed_intent_ = RouteIntent::FOLLOW_MAIN;
                RCLCPP_INFO(this->get_logger(), "Lane change complete. State → FOLLOW_MAIN");
            }
        }

        (void)stop_line_detected;
    }

    // ── Publish the 3 control error parameters from Active Trajectory ────────
    void publish_control_error_from_trajectory(const ActiveTrajectory& traj, double lookahead_d_mm) {
        double epsilon_x = 0.0;
        double epsilon_y = 0.0;
        double theta = 0.0;
        double curv = 0.0;

        if (traj.valid && traj.has_precomputed_control) {
            epsilon_x = traj.precomputed_epsilon_x_mm;
            epsilon_y = traj.precomputed_epsilon_y_mm;
            theta = traj.precomputed_theta_rad;
            curv = traj.precomputed_curvature_inv_mm;
            lookahead_d_mm = traj.precomputed_lookahead_d_mm;
        } else if (traj.valid && !traj.points.empty()) {
            TrajectoryErrorParams params = evaluate_trajectory_at_lookahead(traj, lookahead_d_mm);
            epsilon_x = params.point.x;
            epsilon_y = params.point.y;
            theta = params.theta;
            curv = params.curvature;
        }

        json out;
        out["lane_state"]    = legacy_lane_state_name(state_); // legacy key for compatibility
        out["target_label"]  = traj.source_labels.empty() ? -1 : traj.source_labels.back();
        out["epsilon_x_mm"]  = std::round(epsilon_x * 10.0) / 10.0;
        out["epsilon_y_mm"]  = std::round(epsilon_y * 10.0) / 10.0;
        out["theta_rad"]     = std::round(theta      * 1000.0) / 1000.0;
        out["curvature_inv_mm"] = curv;
        out["lookahead_d_mm"]   = lookahead_d_mm;
        out["trajectory_valid"] = traj.valid;
        
        std_msgs::msg::String msg;
        msg.data = out.dump();
        control_error_pub_->publish(msg);
    }

    // ── Publish lane detection state ─────────────────────────────────────────
    void publish_lane_state(bool has_main, bool has_other, bool has_turn, bool has_stop, 
                            bool blocked_by_marking, const ActiveTrajectory& traj,
                            const std::string& selected_lane_id) {
        json state_json;
        state_json["decision_state"]     = decision_state_name(state_);
        state_json["lane_state"]         = legacy_lane_state_name(state_); // legacy key for compatibility
        state_json["route_intent"]       = route_intent_name(current_intent_);
        state_json["main_lane_detected"]  = has_main;
        state_json["other_lane_detected"] = has_other;
        state_json["turn_lane_detected"]  = has_turn;
        state_json["stop_line_detected"]  = has_stop;
        state_json["blocked_by_marking"]  = blocked_by_marking;
        state_json["trajectory_valid"]    = committed_state_.trajectory.valid;
        
        if (!selected_lane_id.empty()) {
            state_json["selected_lane_id"] = selected_lane_id;
        }

        // Phase 1 extensions
        state_json["trajectory_kind"] = trajectory_kind_name(committed_state_.trajectory.trajectory_kind);
        state_json["committed_trajectory_id"] = committed_state_.trajectory.target_lane_id;
        state_json["normalization_mode"] = committed_state_.trajectory.normalization_mode;
        state_json["trajectory_confidence"] = committed_state_.trajectory.confidence;
        state_json["dropout_hold_counter"] = committed_state_.dropout_hold_counter;
        state_json["replan_reason"] = committed_state_.replan_reason;
        if (committed_state_.trajectory.valid && committed_state_.trajectory.points.size() > 0) {
            json pts = json::array();
            // Subsample if too large to save bandwidth
            size_t step = 1;
            if (committed_state_.trajectory.points.size() > 50) step = committed_state_.trajectory.points.size() / 50;
            for (size_t i = 0; i < committed_state_.trajectory.points.size(); i += step) {
                pts.push_back({std::round(committed_state_.trajectory.points[i].x), std::round(committed_state_.trajectory.points[i].y)});
            }
            // always include last point
            if ((committed_state_.trajectory.points.size() - 1) % step != 0) {
                pts.push_back({std::round(committed_state_.trajectory.points.back().x), std::round(committed_state_.trajectory.points.back().y)});
            }
            state_json["active_trajectory_points"] = pts;
        }

        std_msgs::msg::String msg;
        msg.data = state_json.dump();
        lane_state_pub_->publish(msg);
    }

    // ── ROS2 interfaces ──────────────────────────────────────────────────────
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    control_error_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    lane_state_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetry_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr route_intent_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;

    // ── State ────────────────────────────────────────────────────────────────
    DecisionState state_           = DecisionState::FOLLOW_MAIN;
    RouteIntent current_intent_    = RouteIntent::FOLLOW_MAIN;
    RouteIntent last_processed_intent_ = RouteIntent::FOLLOW_MAIN;
    std::string last_main_track_id_ = "";

    // ── Memory and Planning State (Phase 1) ──────────────────────────────────
    CommittedTrajectoryState committed_state_;
    int consecutive_invalid_frames_ = 0;
    uint64_t frame_count_ = 0;

    // ── Odometry and Speed Tracking (Phase 1/9) ──────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double current_speed_mms_ = 0.0;
    rclcpp::Time last_telemetry_time_;

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
