#include <memory>
#include <string>
#include <chrono>
#include <mutex>
#include <unistd.h>
#include <limits.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

// Helper to check if string is a camera device path and return its index
bool try_parse_camera_device(const std::string& path, int& camera_index) {
    if (path.rfind("/dev/video", 0) != 0) {
        return false;
    }
    
    // Resolve symlinks if present
    char resolved_path[PATH_MAX];
    if (realpath(path.c_str(), resolved_path) != nullptr) {
        std::string resolved(resolved_path);
        // Find trailing digits
        size_t last_non_digit = resolved.find_last_not_of("0123456789");
        if (last_non_digit != std::string::npos && last_non_digit < resolved.length() - 1) {
            std::string digit_str = resolved.substr(last_non_digit + 1);
            try {
                camera_index = std::stoi(digit_str);
                return true;
            } catch (...) {
                // Ignore parsing errors
            }
        }
    }
    return false;
}

class VideoPublisherNode : public rclcpp::Node {
public:
    VideoPublisherNode() : Node("video_publisher_node") {
        // Declare parameters
        this->declare_parameter<std::string>("video_path", "/workspace/test/test_video/video_test1.mp4");
        this->declare_parameter<std::string>("publish_topic", "/camera/image_raw");
        this->declare_parameter<bool>("loop", true);
        this->declare_parameter<double>("fps_override", 0.0); // 0.0 means use video's native FPS

        // Retrieve parameters
        std::string video_path = this->get_parameter("video_path").as_string();
        std::string publish_topic = this->get_parameter("publish_topic").as_string();
        loop_ = this->get_parameter("loop").as_bool();
        fps_override_ = this->get_parameter("fps_override").as_double();

        // Open video source (camera or file)
        int camera_index = 0;
        if (try_parse_camera_device(video_path, camera_index)) {
            RCLCPP_INFO(this->get_logger(), "Opening camera device: %s (index: %d)", video_path.c_str(), camera_index);
            cap_.open(camera_index, cv::CAP_V4L2);
        } else {
            RCLCPP_INFO(this->get_logger(), "Loading video file: %s", video_path.c_str());
            cap_.open(video_path);
        }

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video source: %s", video_path.c_str());
            throw std::runtime_error("Could not open video source");
        }

        // Get video properties
        double native_fps = cap_.get(cv::CAP_PROP_FPS);
        if (native_fps <= 0.0 || std::isnan(native_fps)) {
            native_fps = 30.0;
        }
        double fps = (fps_override_ > 0.0) ? fps_override_ : native_fps;
        int width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        int height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);

        RCLCPP_INFO(this->get_logger(), "Video source loaded. Resolution: %dx%d. FPS: %.2f (native: %.2f)", 
                    width, height, fps, native_fps);

        // Create image publishers
        image_pub = this->create_publisher<sensor_msgs::msg::Image>(publish_topic, 10);
        compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(publish_topic + "/compressed", 10);

        // Calculate timer period in milliseconds
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps));

        // Create timer
        timer_ = this->create_wall_timer(
            period,
            std::bind(&VideoPublisherNode::timer_callback, this)
        );

        // Setup dynamic parameter change callback
        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&VideoPublisherNode::on_set_parameters, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Publisher started. Publishing to topic: %s", publish_topic.c_str());
    }

private:
    rcl_interfaces::msg::SetParametersResult on_set_parameters(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        std::lock_guard<std::mutex> lock(cap_mutex_);
        
        for (const auto &param : parameters) {
            if (param.get_name() == "video_path") {
                std::string new_path = param.as_string();
                cv::VideoCapture new_cap;
                
                int camera_index = 0;
                if (try_parse_camera_device(new_path, camera_index)) {
                    RCLCPP_INFO(this->get_logger(), "Opening camera device: %s (index: %d)", new_path.c_str(), camera_index);
                    new_cap.open(camera_index, cv::CAP_V4L2);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Loading video file: %s", new_path.c_str());
                    new_cap.open(new_path);
                }

                if (new_cap.isOpened()) {
                    cap_ = new_cap;
                    RCLCPP_INFO(this->get_logger(), "Dynamically switched source to: %s", new_path.c_str());
                    
                    // Reset timer based on new video FPS if not overridden
                    if (fps_override_ <= 0.0) {
                        double native_fps = cap_.get(cv::CAP_PROP_FPS);
                        if (native_fps <= 0.0 || std::isnan(native_fps)) {
                            native_fps = 30.0;
                        }
                        timer_->cancel();
                        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / native_fps));
                        timer_ = this->create_wall_timer(
                            period,
                            std::bind(&VideoPublisherNode::timer_callback, this)
                        );
                        RCLCPP_INFO(this->get_logger(), "Adjusted timer interval to match new video native FPS: %.2f", native_fps);
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open new source: %s", new_path.c_str());
                    result.successful = false;
                    result.reason = "Failed to open new source";
                }
            } else if (param.get_name() == "loop") {
                loop_ = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "Loop setting updated to: %s", loop_ ? "true" : "false");
            }
        }
        return result;
    }

    void timer_callback() {
        std::lock_guard<std::mutex> lock(cap_mutex_);
        
        cv::Mat frame;
        if (!cap_.read(frame)) {
            if (loop_) {
                RCLCPP_INFO(this->get_logger(), "Video end reached. Looping back to start.");
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
                if (!cap_.read(frame)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to read frame after resetting video!");
                    return;
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "Video end reached. Stopping timer.");
                timer_->cancel();
                return;
            }
        }

        if (frame.empty()) {
            return;
        }

        auto timestamp = this->now();

        // 1. Publish raw image (Image message)
        if (image_pub->get_subscription_count() > 0) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            msg->header.stamp = timestamp;
            msg->header.frame_id = "camera_frame";
            image_pub->publish(*msg);
        }

        // 2. Publish compressed image (CompressedImage message - JPEG format)
        if (compressed_pub_->get_subscription_count() > 0) {
            sensor_msgs::msg::CompressedImage compressed_msg;
            compressed_msg.header.stamp = timestamp;
            compressed_msg.header.frame_id = "camera_frame";
            compressed_msg.format = "jpeg";

            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
            cv::imencode(".jpg", frame, compressed_msg.data, params);

            compressed_pub_->publish(compressed_msg);
        }
    }

    cv::VideoCapture cap_;
    bool loop_;
    double fps_override_;
    std::mutex cap_mutex_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
