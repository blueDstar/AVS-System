#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
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

    void telemetry_callback(const std_msgs::msg::String::SharedPtr msg) {
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
                }
            }

            // Publish output
            std_msgs::msg::String out_msg;
            out_msg.data = telemetry.dump();
            telemetry_realworld_pub_->publish(out_msg);

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
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IPMTransformNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
