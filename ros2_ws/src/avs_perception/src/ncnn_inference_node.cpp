#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
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

        // Create publisher for compressed images
        compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(output_topic, 10);

        // Create subscription to raw camera images
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, 10,
            std::bind(&NCNNInferenceNode::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Subscribed to %s, publishing segmentation to %s", input_topic.c_str(), output_topic.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto start_time = std::chrono::high_resolution_clock::now();
        bool is_someone_subscribed = (compressed_pub_->get_subscription_count() > 0);

        // Convert ROS2 image to cv::Mat
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Run inference
        std::vector<Object> objects;
        if (yolo_->detect(cv_ptr->image, objects, prob_threshold_, nms_threshold_) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Inference detection failed");
            return;
        }

        if (is_someone_subscribed) {
            // Draw bounding boxes and segmentation masks
            yolo_->draw(cv_ptr->image, objects);

            // Publish as CompressedImage (JPEG format)
            sensor_msgs::msg::CompressedImage compressed_msg;
            compressed_msg.header = msg->header;
            compressed_msg.format = "jpeg";

            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80}; // Good trade-off between bandwidth & quality
            cv::imencode(".jpg", cv_ptr->image, compressed_msg.data, params);

            compressed_pub_->publish(compressed_msg);

            // Measure full latency (inference + visualization + compression)
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
            double fps = 1000.0 / elapsed.count();
            RCLCPP_DEBUG(this->get_logger(), "Full Pipeline Latency (with streaming): %.2f ms (FPS: %.1f)", elapsed.count(), fps);
        } else {
            // Measure pure inference latency only
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
            double fps = 1000.0 / elapsed.count();
            RCLCPP_DEBUG(this->get_logger(), "Pure Inference Latency: %.2f ms (FPS: %.1f) [Streaming: IDLE]", elapsed.count(), fps);
        }
    }

    std::unique_ptr<YOLO26Seg> yolo_;
    float prob_threshold_;
    float nms_threshold_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NCNNInferenceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
