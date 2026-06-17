#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <algorithm>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class IPMTransformNode : public rclcpp::Node {
public:
    IPMTransformNode() : Node("ipm_transform_node") {
        // Declare ROS2 parameters
        this->declare_parameter<std::string>("calibration_file_path", "/workspace/config/calibration.json");
        calibration_file_path_ = this->get_parameter("calibration_file_path").as_string();

        RCLCPP_INFO(this->get_logger(), "Starting IPM Transform Node.");
        RCLCPP_INFO(this->get_logger(), "Calibration file path: %s", calibration_file_path_.c_str());

        // Attempt initial calibration load
        load_calibration();

        // Create publisher for real-world telemetry
        telemetry_realworld_pub_ = this->create_publisher<std_msgs::msg::String>("/avs/telemetry_realworld", 10);

        // Create subscription to telemetry JSON data
        telemetry_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/avs/telemetry", 10,
            std::bind(&IPMTransformNode::telemetry_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Subscribed to /avs/telemetry, publishing to /avs/telemetry_realworld");
    }

private:
    void load_calibration() {
        try {
            if (!std::filesystem::exists(calibration_file_path_)) {
                RCLCPP_WARN(this->get_logger(), "Calibration file does not exist at: %s. IPM transformation will be skipped.", calibration_file_path_.c_str());
                has_calibration_ = false;
                return;
            }

            std::ifstream f(calibration_file_path_);
            if (!f.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open calibration file: %s", calibration_file_path_.c_str());
                has_calibration_ = false;
                return;
            }

            json data;
            f >> data;
            f.close();

            if (data.contains("homography_matrix")) {
                auto h_mat = data["homography_matrix"];
                if (h_mat.size() == 3 && h_mat[0].size() == 3) {
                    for (int i = 0; i < 3; ++i) {
                        for (int j = 0; j < 3; ++j) {
                            H_[i][j] = h_mat[i][j].get<double>();
                        }
                    }
                    has_calibration_ = true;
                    last_write_time_ = std::filesystem::last_write_time(calibration_file_path_);
                    RCLCPP_INFO(this->get_logger(), "Successfully loaded homography matrix H from calibration file.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Invalid homography matrix dimensions in calibration file.");
                    has_calibration_ = false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Calibration file does not contain 'homography_matrix'.");
                has_calibration_ = false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during load_calibration: %s", e.what());
            has_calibration_ = false;
        }
    }

    void check_calibration_update() {
        try {
            if (std::filesystem::exists(calibration_file_path_)) {
                auto current_write_time = std::filesystem::last_write_time(calibration_file_path_);
                if (!has_calibration_ || current_write_time != last_write_time_) {
                    RCLCPP_INFO(this->get_logger(), "Calibration file change detected. Reloading...");
                    load_calibration();
                }
            } else if (has_calibration_) {
                RCLCPP_WARN(this->get_logger(), "Calibration file was removed. Skipping IPM transformations.");
                has_calibration_ = false;
            }
        } catch (const std::filesystem::filesystem_error& e) {
            // Log warning but don't crash, since filesystem operations can occasionally block or fail during writes
            RCLCPP_WARN(this->get_logger(), "Filesystem error while checking calibration update: %s", e.what());
        }
    }

    struct Waypoint {
        double x;
        double y;
    };

    std::vector<Waypoint> extract_centerline_waypoints_y(const std::vector<std::vector<double>>& poly_real_world, double step_mm = 100.0) {
        std::vector<Waypoint> waypoints;
        if (poly_real_world.empty()) return waypoints;

        double min_y = 1e9;
        double max_y = -1e9;
        for (const auto& pt : poly_real_world) {
            if (pt.size() >= 2) {
                double y = pt[1];
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
            }
        }

        if (min_y >= max_y) return waypoints;

        double start_y = std::ceil(min_y / step_mm) * step_mm;
        for (double y = start_y; y <= max_y; y += step_mm) {
            std::vector<double> x_intersections;
            size_t n = poly_real_world.size();
            for (size_t i = 0; i < n; ++i) {
                const auto& p1 = poly_real_world[i];
                const auto& p2 = poly_real_world[(i + 1) % n];
                if (p1.size() < 2 || p2.size() < 2) continue;

                double x1 = p1[0], y1 = p1[1];
                double x2 = p2[0], y2 = p2[1];

                if ((y1 <= y && y2 >= y) || (y2 <= y && y1 >= y)) {
                    if (std::abs(y2 - y1) > 1e-5) {
                        double x_int = x1 + (y - y1) * (x2 - x1) / (y2 - y1);
                        x_intersections.push_back(x_int);
                    }
                }
            }

            if (!x_intersections.empty()) {
                auto minmax = std::minmax_element(x_intersections.begin(), x_intersections.end());
                double x_left = *minmax.first;
                double x_right = *minmax.second;
                double x_mid = (x_left + x_right) / 2.0;
                waypoints.push_back({x_mid, y});
            }
        }

        std::sort(waypoints.begin(), waypoints.end(), [](const Waypoint& a, const Waypoint& b) {
            return a.y < b.y;
        });

        return waypoints;
    }

    std::vector<Waypoint> extract_centerline_waypoints_x(const std::vector<std::vector<double>>& poly_real_world, double step_mm = 100.0) {
        std::vector<Waypoint> waypoints;
        if (poly_real_world.empty()) return waypoints;

        double min_x = 1e9;
        double max_x = -1e9;
        for (const auto& pt : poly_real_world) {
            if (pt.size() >= 2) {
                double x = pt[0];
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
            }
        }

        if (min_x >= max_x) return waypoints;

        double start_x = std::ceil(min_x / step_mm) * step_mm;
        for (double x = start_x; x <= max_x; x += step_mm) {
            std::vector<double> y_intersections;
            size_t n = poly_real_world.size();
            for (size_t i = 0; i < n; ++i) {
                const auto& p1 = poly_real_world[i];
                const auto& p2 = poly_real_world[(i + 1) % n];
                if (p1.size() < 2 || p2.size() < 2) continue;

                double x1 = p1[0], y1 = p1[1];
                double x2 = p2[0], y2 = p2[1];

                if ((x1 <= x && x2 >= x) || (x2 <= x && x1 >= x)) {
                    if (std::abs(x2 - x1) > 1e-5) {
                        double y_int = y1 + (x - x1) * (y2 - y1) / (x2 - x1);
                        y_intersections.push_back(y_int);
                    }
                }
            }

            if (!y_intersections.empty()) {
                auto minmax = std::minmax_element(y_intersections.begin(), y_intersections.end());
                double y_min_val = *minmax.first;
                double y_max_val = *minmax.second;
                double y_mid = (y_min_val + y_max_val) / 2.0;
                waypoints.push_back({x, y_mid});
            }
        }

        std::sort(waypoints.begin(), waypoints.end(), [](const Waypoint& a, const Waypoint& b) {
            return a.x < b.x;
        });

        return waypoints;
    }

    std::vector<double> fit_polynomial_xy(const std::vector<Waypoint>& waypoints) {
        std::vector<double> coeffs(4, 0.0);
        size_t n = waypoints.size();
        if (n < 2) {
            return coeffs;
        }

        if (n >= 4) {
            cv::Mat A(n, 4, CV_64F);
            cv::Mat B(n, 1, CV_64F);
            for (size_t i = 0; i < n; ++i) {
                double y = waypoints[i].y;
                A.at<double>(i, 0) = y * y * y;
                A.at<double>(i, 1) = y * y;
                A.at<double>(i, 2) = y;
                A.at<double>(i, 3) = 1.0;
                B.at<double>(i, 0) = waypoints[i].x;
            }
            cv::Mat C;
            cv::solve(A, B, C, cv::DECOMP_SVD);
            coeffs[0] = C.at<double>(0, 0); // a3
            coeffs[1] = C.at<double>(1, 0); // a2
            coeffs[2] = C.at<double>(2, 0); // a1
            coeffs[3] = C.at<double>(3, 0); // a0
        } else {
            cv::Mat A(n, 2, CV_64F);
            cv::Mat B(n, 1, CV_64F);
            for (size_t i = 0; i < n; ++i) {
                double y = waypoints[i].y;
                A.at<double>(i, 0) = y;
                A.at<double>(i, 1) = 1.0;
                B.at<double>(i, 0) = waypoints[i].x;
            }
            cv::Mat C;
            cv::solve(A, B, C, cv::DECOMP_SVD);
            coeffs[0] = 0.0; // a3
            coeffs[1] = 0.0; // a2
            coeffs[2] = C.at<double>(0, 0); // a1
            coeffs[3] = C.at<double>(1, 0); // a0
        }

        return coeffs;
    }

    std::vector<double> fit_polynomial_yx(const std::vector<Waypoint>& waypoints) {
        std::vector<double> coeffs(4, 0.0);
        size_t n = waypoints.size();
        if (n < 2) {
            return coeffs;
        }

        if (n >= 4) {
            cv::Mat A(n, 4, CV_64F);
            cv::Mat B(n, 1, CV_64F);
            for (size_t i = 0; i < n; ++i) {
                double x = waypoints[i].x;
                A.at<double>(i, 0) = x * x * x;
                A.at<double>(i, 1) = x * x;
                A.at<double>(i, 2) = x;
                A.at<double>(i, 3) = 1.0;
                B.at<double>(i, 0) = waypoints[i].y;
            }
            cv::Mat C;
            cv::solve(A, B, C, cv::DECOMP_SVD);
            coeffs[0] = C.at<double>(0, 0); // a3
            coeffs[1] = C.at<double>(1, 0); // a2
            coeffs[2] = C.at<double>(2, 0); // a1
            coeffs[3] = C.at<double>(3, 0); // a0
        } else {
            cv::Mat A(n, 2, CV_64F);
            cv::Mat B(n, 1, CV_64F);
            for (size_t i = 0; i < n; ++i) {
                double x = waypoints[i].x;
                A.at<double>(i, 0) = x;
                A.at<double>(i, 1) = 1.0;
                B.at<double>(i, 0) = waypoints[i].y;
            }
            cv::Mat C;
            cv::solve(A, B, C, cv::DECOMP_SVD);
            coeffs[0] = 0.0; // a3
            coeffs[1] = 0.0; // a2
            coeffs[2] = C.at<double>(0, 0); // a1
            coeffs[3] = C.at<double>(1, 0); // a0
        }

        return coeffs;
    }

    void telemetry_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received telemetry message! Length: %zu bytes", msg->data.size());
        // Periodically check/reload calibration file
        check_calibration_update();

        if (!has_calibration_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "IPM calibration is missing or invalid. Telemetry publish blocked. Please calibrate via the Web Dashboard.");
            return;
        }

        try {
            json telemetry = json::parse(msg->data);

            // If we have calibration, perform the planar homography mapping
            if (telemetry.contains("objects") && telemetry["objects"].is_array()) {
                for (auto& obj : telemetry["objects"]) {
                    obj["polygons_real_world"] = json::array();

                    if (obj.contains("polygons") && obj["polygons"].is_array()) {
                        for (const auto& poly : obj["polygons"]) {
                            json poly_real = json::array();

                            if (poly.is_array()) {
                                for (const auto& pt : poly) {
                                    if (pt.is_array() && pt.size() >= 2) {
                                        double u = pt[0].get<double>();
                                        double v = pt[1].get<double>();

                                        // Apply perspective transform equations:
                                        // X_world = (h00*u + h01*v + h02) / (h20*u + h21*v + h22)
                                        // Y_world = (h10*u + h11*v + h12) / (h20*u + h21*v + h22)
                                        double w = H_[2][0] * u + H_[2][1] * v + H_[2][2];
                                        
                                        if (std::abs(w) > 1e-6) {
                                            double X = (H_[0][0] * u + H_[0][1] * v + H_[0][2]) / w;
                                            double Y = (H_[1][0] * u + H_[1][1] * v + H_[1][2]) / w;
                                            
                                            // Rounding for clean representation in JSON (optional, but keeps strings readable)
                                            X = std::round(X * 10.0) / 10.0;
                                            Y = std::round(Y * 10.0) / 10.0;

                                            poly_real.push_back({X, Y});
                                        }
                                    }
                                }
                            }
                            obj["polygons_real_world"].push_back(poly_real);
                        }
                    }

                    // Extract centerline waypoints and fit polynomial for lane objects
                    int label = obj.contains("label") ? obj["label"].get<int>() : -1;
                    bool is_lane = (label == 3 || label == 4 || label == 10);
                    if (is_lane) {
                        obj["waypoints"] = json::array();
                        obj["polynomial"] = {{"a3", 0.0}, {"a2", 0.0}, {"a1", 0.0}, {"a0", 0.0}};
                        obj["lateral_offset_mm"] = 0.0;
                        obj["longitudinal_offset_mm"] = 0.0;
                        obj["heading_angle_rad"] = 0.0;
                        obj["curvature_inv_mm"] = 0.0;

                        if (obj.contains("polygons_real_world") && obj["polygons_real_world"].is_array()) {
                            std::vector<Waypoint> all_waypoints;
                            
                            // Check if turn-lane (label 10) -> sweep along X and fit y(x)
                            if (label == 10) {
                                for (const auto& poly_json : obj["polygons_real_world"]) {
                                    if (poly_json.is_array()) {
                                        std::vector<std::vector<double>> poly_pts;
                                        for (const auto& pt : poly_json) {
                                            if (pt.is_array() && pt.size() >= 2) {
                                                poly_pts.push_back({pt[0].get<double>(), pt[1].get<double>()});
                                            }
                                        }
                                        auto wps = extract_centerline_waypoints_x(poly_pts, 100.0);
                                        all_waypoints.insert(all_waypoints.end(), wps.begin(), wps.end());
                                    }
                                }

                                if (all_waypoints.size() >= 2) {
                                    std::sort(all_waypoints.begin(), all_waypoints.end(), [](const Waypoint& a, const Waypoint& b) {
                                        return a.x < b.x;
                                    });

                                    std::vector<Waypoint> unique_waypoints;
                                    for (const auto& wp : all_waypoints) {
                                        if (unique_waypoints.empty() || std::abs(unique_waypoints.back().x - wp.x) > 1e-3) {
                                            unique_waypoints.push_back(wp);
                                        } else {
                                            unique_waypoints.back().y = (unique_waypoints.back().y + wp.y) / 2.0;
                                        }
                                    }

                                    // 1. Spatial smoothing of raw waypoints
                                    if (unique_waypoints.size() >= 3) {
                                        std::vector<Waypoint> smoothed = unique_waypoints;
                                        for (size_t i = 1; i < unique_waypoints.size() - 1; ++i) {
                                            smoothed[i].y = (unique_waypoints[i-1].y + unique_waypoints[i].y + unique_waypoints[i+1].y) / 3.0;
                                        }
                                        unique_waypoints = smoothed;
                                    }

                                    // 2. Fit raw polynomial
                                    std::vector<double> coeffs = fit_polynomial_yx(unique_waypoints);

                                    // 3. Temporal smoothing of coefficients
                                    double alpha = 0.25;
                                    if (has_prev_[label]) {
                                        for (size_t c = 0; c < 4; ++c) {
                                            coeffs[c] = alpha * coeffs[c] + (1.0 - alpha) * prev_coeffs_[label][c];
                                        }
                                    }
                                    prev_coeffs_[label] = coeffs;
                                    has_prev_[label] = true;

                                    // 4. Regenerate smooth waypoints from the smoothed polynomial
                                    double x_min = unique_waypoints.front().x;
                                    double x_max = unique_waypoints.back().x;
                                    std::vector<Waypoint> smooth_wps;
                                    for (double x_val = x_min; x_val <= x_max; x_val += 100.0) {
                                        double y_val = coeffs[0] * std::pow(x_val, 3) + coeffs[1] * std::pow(x_val, 2) + coeffs[2] * x_val + coeffs[3];
                                        smooth_wps.push_back({x_val, y_val});
                                    }
                                    if (smooth_wps.empty() || std::abs(smooth_wps.back().x - x_max) > 1e-3) {
                                        double y_val = coeffs[0] * std::pow(x_max, 3) + coeffs[1] * std::pow(x_max, 2) + coeffs[2] * x_max + coeffs[3];
                                        smooth_wps.push_back({x_max, y_val});
                                    }

                                    // Write smooth waypoints to JSON
                                    for (const auto& wp : smooth_wps) {
                                        double rx = std::round(wp.x * 10.0) / 10.0;
                                        double ry = std::round(wp.y * 10.0) / 10.0;
                                        obj["waypoints"].push_back({rx, ry});
                                    }

                                    // Write smooth polynomial coefficients to JSON
                                    obj["polynomial"]["a3"] = coeffs[0];
                                    obj["polynomial"]["a2"] = coeffs[1];
                                    obj["polynomial"]["a1"] = coeffs[2];
                                    obj["polynomial"]["a0"] = coeffs[3];

                                    // 5. Smooth and write control outputs
                                    double longitudinal_offset = coeffs[3];
                                    double heading_angle = std::atan(coeffs[2]);
                                    double curvature = 2.0 * coeffs[1];

                                    if (has_prev_metrics_[label]) {
                                        longitudinal_offset = alpha * longitudinal_offset + (1.0 - alpha) * prev_longitudinal_offset_[label];
                                        heading_angle = alpha * heading_angle + (1.0 - alpha) * prev_heading_angle_[label];
                                        curvature = alpha * curvature + (1.0 - alpha) * prev_curvature_[label];
                                    }
                                    prev_longitudinal_offset_[label] = longitudinal_offset;
                                    prev_heading_angle_[label] = heading_angle;
                                    prev_curvature_[label] = curvature;
                                    has_prev_metrics_[label] = true;

                                    obj["lateral_offset_mm"] = 0.0;
                                    obj["longitudinal_offset_mm"] = std::round(longitudinal_offset * 10.0) / 10.0;
                                    obj["heading_angle_rad"] = std::round(heading_angle * 1000.0) / 1000.0;
                                    obj["curvature_inv_mm"] = std::round(curvature * 1e6) / 1e6;
                                }
                            } else {
                                // main-lane (label 3) or other-lane (label 4) -> sweep along Y and fit x(y)
                                for (const auto& poly_json : obj["polygons_real_world"]) {
                                    if (poly_json.is_array()) {
                                        std::vector<std::vector<double>> poly_pts;
                                        for (const auto& pt : poly_json) {
                                            if (pt.is_array() && pt.size() >= 2) {
                                                poly_pts.push_back({pt[0].get<double>(), pt[1].get<double>()});
                                            }
                                        }
                                        auto wps = extract_centerline_waypoints_y(poly_pts, 100.0);
                                        all_waypoints.insert(all_waypoints.end(), wps.begin(), wps.end());
                                    }
                                }

                                if (all_waypoints.size() >= 2) {
                                    std::sort(all_waypoints.begin(), all_waypoints.end(), [](const Waypoint& a, const Waypoint& b) {
                                        return a.y < b.y;
                                    });

                                    std::vector<Waypoint> unique_waypoints;
                                    for (const auto& wp : all_waypoints) {
                                        if (unique_waypoints.empty() || std::abs(unique_waypoints.back().y - wp.y) > 1e-3) {
                                            unique_waypoints.push_back(wp);
                                        } else {
                                            unique_waypoints.back().x = (unique_waypoints.back().x + wp.x) / 2.0;
                                        }
                                    }

                                    // 1. Spatial smoothing of raw waypoints
                                    if (unique_waypoints.size() >= 3) {
                                        std::vector<Waypoint> smoothed = unique_waypoints;
                                        for (size_t i = 1; i < unique_waypoints.size() - 1; ++i) {
                                            smoothed[i].x = (unique_waypoints[i-1].x + unique_waypoints[i].x + unique_waypoints[i+1].x) / 3.0;
                                        }
                                        unique_waypoints = smoothed;
                                    }

                                    // 2. Fit raw polynomial
                                    std::vector<double> coeffs = fit_polynomial_xy(unique_waypoints);

                                    // 3. Temporal smoothing of coefficients
                                    double alpha = 0.25;
                                    if (has_prev_[label]) {
                                        for (size_t c = 0; c < 4; ++c) {
                                            coeffs[c] = alpha * coeffs[c] + (1.0 - alpha) * prev_coeffs_[label][c];
                                        }
                                    }
                                    prev_coeffs_[label] = coeffs;
                                    has_prev_[label] = true;

                                    // 4. Regenerate smooth waypoints from the smoothed polynomial
                                    double y_min = unique_waypoints.front().y;
                                    double y_max = unique_waypoints.back().y;
                                    std::vector<Waypoint> smooth_wps;
                                    for (double y_val = y_min; y_val <= y_max; y_val += 100.0) {
                                        double x_val = coeffs[0] * std::pow(y_val, 3) + coeffs[1] * std::pow(y_val, 2) + coeffs[2] * y_val + coeffs[3];
                                        smooth_wps.push_back({x_val, y_val});
                                    }
                                    if (smooth_wps.empty() || std::abs(smooth_wps.back().y - y_max) > 1e-3) {
                                        double x_val = coeffs[0] * std::pow(y_max, 3) + coeffs[1] * std::pow(y_max, 2) + coeffs[2] * y_max + coeffs[3];
                                        smooth_wps.push_back({x_val, y_max});
                                    }

                                    // Write smooth waypoints to JSON
                                    for (const auto& wp : smooth_wps) {
                                        double rx = std::round(wp.x * 10.0) / 10.0;
                                        double ry = std::round(wp.y * 10.0) / 10.0;
                                        obj["waypoints"].push_back({rx, ry});
                                    }

                                    // Write smooth polynomial coefficients to JSON
                                    obj["polynomial"]["a3"] = coeffs[0];
                                    obj["polynomial"]["a2"] = coeffs[1];
                                    obj["polynomial"]["a1"] = coeffs[2];
                                    obj["polynomial"]["a0"] = coeffs[3];

                                    // 5. Smooth and write control outputs
                                    double lateral_offset = coeffs[3];
                                    double heading_angle = std::atan(coeffs[2]);
                                    double curvature = 2.0 * coeffs[1];

                                    if (has_prev_metrics_[label]) {
                                        lateral_offset = alpha * lateral_offset + (1.0 - alpha) * prev_lateral_offset_[label];
                                        heading_angle = alpha * heading_angle + (1.0 - alpha) * prev_heading_angle_[label];
                                        curvature = alpha * curvature + (1.0 - alpha) * prev_curvature_[label];
                                    }
                                    prev_lateral_offset_[label] = lateral_offset;
                                    prev_heading_angle_[label] = heading_angle;
                                    prev_curvature_[label] = curvature;
                                    has_prev_metrics_[label] = true;

                                    obj["lateral_offset_mm"] = std::round(lateral_offset * 10.0) / 10.0;
                                    obj["longitudinal_offset_mm"] = 0.0;
                                    obj["heading_angle_rad"] = std::round(heading_angle * 1000.0) / 1000.0;
                                    obj["curvature_inv_mm"] = std::round(curvature * 1e6) / 1e6;
                                }
                            }
                        }
                    }
                }
            }

            // Publish output
            std_msgs::msg::String out_msg;
            out_msg.data = telemetry.dump();
            telemetry_realworld_pub_->publish(out_msg);
            RCLCPP_INFO(this->get_logger(), "Published real-world telemetry! Size: %zu bytes", out_msg.data.size());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing telemetry: %s", e.what());
            
            // Fallback: publish original telemetry message on error
            telemetry_realworld_pub_->publish(*msg);
        }
    }

    std::string calibration_file_path_;
    bool has_calibration_ = false;
    double H_[3][3] = {0};
    std::filesystem::file_time_type last_write_time_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_realworld_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetry_sub_;

    // Temporal smoothing memory (Label -> previous values)
    std::map<int, std::vector<double>> prev_coeffs_;
    std::map<int, double> prev_lateral_offset_;
    std::map<int, double> prev_longitudinal_offset_;
    std::map<int, double> prev_heading_angle_;
    std::map<int, double> prev_curvature_;
    std::map<int, bool> has_prev_;
    std::map<int, bool> has_prev_metrics_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IPMTransformNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
