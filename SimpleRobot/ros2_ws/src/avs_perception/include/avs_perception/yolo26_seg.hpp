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
    void set_num_threads(int num_threads);

private:
    ncnn::Net net;
    int num_threads_ = 3;
    int target_size = 320;
    std::vector<std::string> class_names = {
        "dashed-white", "dashed-yellow", "double-solid-white", "main-lane",
        "other-lane", "parking-zone", "sign-no-left", "sign-no-parking",
        "sign-no-right", "sign-parking", "sign-stop", "sign-turn-left",
        "sign-turn-right", "solid-white", "solid-yellow", "start",
        "stop-line", "turn-lane", "vehicle"
    };

    // Color palette for segmentation overlay (BGR format)
    std::vector<cv::Scalar> class_colors = {
        cv::Scalar(255, 0, 0),      // dashed-white: Blue
        cv::Scalar(0, 165, 255),    // dashed-yellow: Orange
        cv::Scalar(255, 127, 0),    // double-solid-white: Light Blue
        cv::Scalar(0, 255, 0),      // main-lane: Green
        cv::Scalar(0, 0, 255),      // other-lane: Red
        cv::Scalar(128, 128, 128),  // parking-zone: Gray
        cv::Scalar(60, 20, 220),    // sign-no-left: Crimson
        cv::Scalar(0, 0, 180),      // sign-no-parking: Bright Crimson
        cv::Scalar(50, 50, 150),    // sign-no-right: Dark Red
        cv::Scalar(230, 100, 50),   // sign-parking: Royal Blue
        cv::Scalar(0, 0, 255),      // sign-stop: Stop Red
        cv::Scalar(235, 206, 135),  // sign-turn-left: Sky Blue
        cv::Scalar(180, 130, 70),   // sign-turn-right: Steel Blue
        cv::Scalar(255, 255, 0),    // solid-white: Cyan
        cv::Scalar(0, 255, 255),    // solid-yellow: Yellow
        cv::Scalar(0, 255, 127),    // start: Spring Green
        cv::Scalar(0, 0, 128),      // stop-line: Navy
        cv::Scalar(127, 0, 255),    // turn-lane: Purple
        cv::Scalar(255, 0, 255)     // vehicle: Magenta
    };

    void qsort_descent_inplace(std::vector<Object>& objects, int left, int right);
    void qsort_descent_inplace(std::vector<Object>& objects);
    void nms_sorted_bboxes(const std::vector<Object>& objects, std::vector<int>& picked, float nms_threshold);
    float intersection_area(const Object& a, const Object& b);
    void decode_mask(const ncnn::Mat& proto, const std::vector<float>& mask_feats, const cv::Rect& rect, cv::Mat& dest_mask, const cv::Size& img_size);
};

#endif // YOLO26_SEG_HPP
