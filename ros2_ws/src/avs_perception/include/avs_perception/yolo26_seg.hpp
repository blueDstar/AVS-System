#ifndef YOLO26_SEG_HPP
#define YOLO26_SEG_HPP

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ncnn/net.h>

struct Object {
    cv::Rect_<float> rect;
    int label;
    float prob;
    std::vector<float> mask_feats; // 32 mask coefficients
    cv::Mat mask;                  // CV_8UC1 binary mask at full image size
};

class YOLO26Seg {
public:
    YOLO26Seg();
    ~YOLO26Seg();

    int load(const std::string& param_path, const std::string& bin_path);
    int detect(const cv::Mat& bgr, std::vector<Object>& objects, float prob_threshold = 0.25f, float nms_threshold = 0.45f);
    void draw(cv::Mat& image, const std::vector<Object>& objects);

private:
    ncnn::Net net;
    int target_size = 320;
    std::vector<std::string> class_names = {
        "dashed-white", "dashed-yellow", "double-solid-white", "main-lane",
        "other-lane", "parking-zone", "solid-white", "solid-yellow",
        "start", "stop-line", "turn-lane", "vehicle"
    };

    // Color palette for segmentation overlay (BGR format)
    std::vector<cv::Scalar> class_colors = {
        cv::Scalar(255, 0, 0),     // dashed-white: Blue
        cv::Scalar(0, 165, 255),   // dashed-yellow: Orange
        cv::Scalar(255, 127, 0),   // double-solid-white: Light Blue
        cv::Scalar(0, 255, 0),     // main-lane: Green
        cv::Scalar(0, 0, 255),     // other-lane: Red
        cv::Scalar(128, 128, 128), // parking-zone: Gray
        cv::Scalar(255, 255, 0),   // solid-white: Cyan
        cv::Scalar(0, 255, 255),   // solid-yellow: Yellow
        cv::Scalar(0, 255, 127),   // start: Spring Green
        cv::Scalar(0, 0, 128),     // stop-line: Navy
        cv::Scalar(127, 0, 255),   // turn-lane: Purple
        cv::Scalar(255, 0, 255)    // vehicle: Magenta
    };

    void qsort_descent_inplace(std::vector<Object>& objects, int left, int right);
    void qsort_descent_inplace(std::vector<Object>& objects);
    void nms_sorted_bboxes(const std::vector<Object>& objects, std::vector<int>& picked, float nms_threshold);
    float intersection_area(const Object& a, const Object& b);
    void decode_mask(const ncnn::Mat& proto, const std::vector<float>& mask_feats, const cv::Rect& rect, cv::Mat& dest_mask, const cv::Size& img_size);
};

#endif // YOLO26_SEG_HPP
