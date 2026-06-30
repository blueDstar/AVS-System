#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "avs_perception/yolo26_seg.hpp"

class VideoTestNode : public rclcpp::Node {
public:
    VideoTestNode() : Node("video_test_node") {
        // Declare parameters
        this->declare_parameter<std::string>("model_param_path", "/workspace/models/best_ncnn_model_int8/model.ncnn.param");
        this->declare_parameter<std::string>("model_bin_path", "/workspace/models/best_ncnn_model_int8/model.ncnn.bin");
        this->declare_parameter<std::string>("video_path", "/workspace/test/test_video/video_test1.mp4");
        this->declare_parameter<std::string>("output_path", "/workspace/test/test_video_output/output_video_test1.mp4");
        this->declare_parameter<float>("prob_threshold", 0.25f);
        this->declare_parameter<float>("nms_threshold", 0.45f);

        // Retrieve parameters
        std::string param_path = this->get_parameter("model_param_path").as_string();
        std::string bin_path = this->get_parameter("model_bin_path").as_string();
        std::string video_path = this->get_parameter("video_path").as_string();
        std::string output_path = this->get_parameter("output_path").as_string();
        float prob_threshold = this->get_parameter("prob_threshold").as_double();
        float nms_threshold = this->get_parameter("nms_threshold").as_double();

        RCLCPP_INFO(this->get_logger(), "Initializing Video Profiler...");
        RCLCPP_INFO(this->get_logger(), "Input Video: %s", video_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Output Video: %s", output_path.c_str());

        // Initialize YOLO engine
        auto yolo = std::make_unique<YOLO26Seg>();
        if (yolo->load(param_path, bin_path) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load NCNN model!");
            return;
        }

        // Open input video
        cv::VideoCapture cap(video_path);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open input video file: %s", video_path.c_str());
            return;
        }

        int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        double fps = cap.get(cv::CAP_PROP_FPS);
        int total_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);

        RCLCPP_INFO(this->get_logger(), "Video Resolution: %dx%d, Native FPS: %.2f, Total Frames: %d", 
                    width, height, fps, total_frames);

        // Open output video writer
        cv::VideoWriter writer(
            output_path,
            cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
            fps,
            cv::Size(width, height)
        );

        if (!writer.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open output video file for writing: %s", output_path.c_str());
            return;
        }

        cv::Mat frame;
        int frame_count = 0;
        double total_latency = 0.0;
        double max_latency = 0.0;
        double min_latency = 1e9;

        RCLCPP_INFO(this->get_logger(), "Processing frames...");

        while (cap.read(frame)) {
            if (frame.empty()) break;

            auto start = std::chrono::high_resolution_clock::now();

            // Run detection & segmentation
            std::vector<Object> objects;
            yolo->detect(frame, objects, prob_threshold, nms_threshold);

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end - start;
            double latency = duration.count();

            total_latency += latency;
            max_latency = std::max(max_latency, latency);
            min_latency = std::min(min_latency, latency);
            
            if (frame_count < 5) {
                std::cout << "[CPP DIAGNOSTIC] Frame " << frame_count << " Detections: ";
                if (objects.empty()) {
                    std::cout << "None";
                } else {
                    for (const auto& obj : objects) {
                        // Class names: 0: dashed-white, 1: dashed-yellow, 2: double-solid-white, 3: main-lane, 4: other-lane, etc.
                        std::cout << obj.label << " (" << obj.prob << "), ";
                    }
                }
                std::cout << std::endl;
            }
            
            frame_count++;

            // Draw results
            yolo->draw(frame, objects);

            // Write drawn frame to output video
            writer.write(frame);

            if (frame_count % 30 == 0 || frame_count == total_frames) {
                RCLCPP_INFO(this->get_logger(), "Processed %d/%d frames (%.1f%%). Current Latency: %.2f ms", 
                            frame_count, total_frames, (float)frame_count / total_frames * 100.f, latency);
            }
        }

        cap.release();
        writer.release();

        if (frame_count > 0) {
            double avg_latency = total_latency / frame_count;
            double avg_fps = 1000.0 / avg_latency;
            RCLCPP_INFO(this->get_logger(), "=========================================");
            RCLCPP_INFO(this->get_logger(), "             PROFILING REPORT            ");
            RCLCPP_INFO(this->get_logger(), "=========================================");
            RCLCPP_INFO(this->get_logger(), "Processed Frames: %d", frame_count);
            RCLCPP_INFO(this->get_logger(), "Inference Latency stats (CPU):");
            RCLCPP_INFO(this->get_logger(), "  - Average:  %.2f ms", avg_latency);
            RCLCPP_INFO(this->get_logger(), "  - Min:      %.2f ms", min_latency);
            RCLCPP_INFO(this->get_logger(), "  - Max:      %.2f ms", max_latency);
            RCLCPP_INFO(this->get_logger(), "Inference Performance: %.2f FPS", avg_fps);
            RCLCPP_INFO(this->get_logger(), "=========================================");
        } else {
            RCLCPP_ERROR(this->get_logger(), "No frames were processed!");
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoTestNode>();
    rclcpp::shutdown();
    return 0;
}
