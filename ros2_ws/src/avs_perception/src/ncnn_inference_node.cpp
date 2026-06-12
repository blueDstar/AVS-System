#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

#include "avs_perception/yolo26_seg.hpp"

class NCNNInferenceNode : public rclcpp::Node {
public:
    NCNNInferenceNode() : Node("ncnn_inference_node") {
        // Declare ROS2 parameters
        this->declare_parameter<std::string>("model_param_path", "/workspace/models/yolo26-best_ncnn_model/model.ncnn.param");
        this->declare_parameter<std::string>("model_bin_path", "/workspace/models/yolo26-best_ncnn_model/model.ncnn.bin");
        this->declare_parameter<float>("prob_threshold", 0.25f);
        this->declare_parameter<float>("nms_threshold", 0.45f);
        this->declare_parameter<std::string>("input_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("output_topic", "/camera/segmented_image/compressed");

        // Retrieve parameters
        std::string param_path = this->get_parameter("model_param_path").as_string();
        std::string bin_path = this->get_parameter("model_bin_path").as_string();
        prob_threshold_ = this->get_parameter("prob_threshold").as_double();
        nms_threshold_ = this->get_parameter("nms_threshold").as_double();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "Loading NCNN model from: \n  Param: %s\n  Bin: %s", param_path.c_str(), bin_path.c_str());
        
        // Initialize inference engine
        yolo_ = std::make_unique<YOLO26Seg>();
        if (yolo_->load(param_path, bin_path) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load model files!");
            throw std::runtime_error("Failed to load NCNN model");
        }
        RCLCPP_INFO(this->get_logger(), "Model loaded successfully.");

        // Create publisher for telemetry data
        telemetry_pub_ = this->create_publisher<std_msgs::msg::String>("/avs/telemetry", 10);

        // Create subscription to raw camera images
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, 10,
            std::bind(&NCNNInferenceNode::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Subscribed to %s, publishing telemetry to /avs/telemetry", input_topic.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto start_time = std::chrono::high_resolution_clock::now();

        // Update parameters dynamically from standard parameter service
        prob_threshold_ = static_cast<float>(this->get_parameter("prob_threshold").as_double());
        nms_threshold_ = static_cast<float>(this->get_parameter("nms_threshold").as_double());

        // Convert ROS2 image to cv::Mat
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Run inference
        auto inference_start = std::chrono::high_resolution_clock::now();
        std::vector<Object> objects;
        if (yolo_->detect(cv_ptr->image, objects, prob_threshold_, nms_threshold_) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Inference detection failed");
            return;
        }
        auto inference_end = std::chrono::high_resolution_clock::now();
        double inference_latency = std::chrono::duration<double, std::milli>(inference_end - inference_start).count();

        // Measure full latency (inference + contour extraction + JSON serialization)
        auto end_time = std::chrono::high_resolution_clock::now();
        double full_latency = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        double fps = 1000.0 / full_latency;

        RCLCPP_DEBUG(this->get_logger(), "Inference Latency: %.2f ms (FPS: %.1f)", full_latency, fps);

        // Publish JSON telemetry
        const std::vector<std::string> class_names = {
            "dashed-white", "dashed-yellow", "double-solid-white", "main-lane",
            "other-lane", "parking-zone", "solid-white", "solid-yellow",
            "start", "stop-line", "turn-lane", "vehicle"
        };

        std::string json_str = "{";
        json_str += "\"inference_latency_ms\":" + std::to_string(inference_latency) + ",";
        json_str += "\"full_latency_ms\":" + std::to_string(full_latency) + ",";
        json_str += "\"fps\":" + std::to_string(fps) + ",";
        json_str += "\"streaming\":true,";
        
        json_str += "\"detections\":{";
        for (size_t i = 0; i < class_names.size(); i++) {
            int count = 0;
            for (const auto& obj : objects) {
                if (obj.label == static_cast<int>(i)) {
                    count++;
                }
            }
            json_str += "\"" + class_names[i] + "\":" + std::to_string(count);
            if (i < class_names.size() - 1) {
                json_str += ",";
            }
        }
        json_str += "},";

        json_str += "\"objects\":[";
        for (size_t i = 0; i < objects.size(); i++) {
            const auto& obj = objects[i];
            json_str += "{";
            json_str += "\"label\":" + std::to_string(obj.label) + ",";
            json_str += "\"prob\":" + std::to_string(obj.prob) + ",";
            json_str += "\"box\":[" + std::to_string(obj.rect.x) + "," 
                                   + std::to_string(obj.rect.y) + "," 
                                   + std::to_string(obj.rect.width) + "," 
                                   + std::to_string(obj.rect.height) + "],";
            
            // Extract contours of the mask for this object to represent polygons
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            if (!obj.mask.empty()) {
                cv::findContours(obj.mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            }
            
            json_str += "\"polygons\":[";
            for (size_t c = 0; c < contours.size(); c++) {
                json_str += "[";
                for (size_t p = 0; p < contours[c].size(); p++) {
                    json_str += "[" + std::to_string(contours[c][p].x) + "," + std::to_string(contours[c][p].y) + "]";
                    if (p < contours[c].size() - 1) json_str += ",";
                }
                json_str += "]";
                if (c < contours.size() - 1) json_str += ",";
            }
            json_str += "]";
            json_str += "}";
            if (i < objects.size() - 1) json_str += ",";
        }
        json_str += "]";
        json_str += "}";

        auto telemetry_msg = std_msgs::msg::String();
        telemetry_msg.data = json_str;
        telemetry_pub_->publish(telemetry_msg);
    }

    std::unique_ptr<YOLO26Seg> yolo_;
    float prob_threshold_;
    float nms_threshold_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NCNNInferenceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
