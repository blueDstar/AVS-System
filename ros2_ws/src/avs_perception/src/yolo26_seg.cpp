#include "avs_perception/yolo26_seg.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

YOLO26Seg::YOLO26Seg() {
    // Enable CPU optimizations in NCNN
    net.opt.use_vulkan_compute = false;
    net.opt.use_fp16_packed = true;
    net.opt.use_fp16_storage = true;
    net.opt.use_fp16_arithmetic = true;
    net.opt.use_packing_layout = true;
    net.opt.num_threads = 4; // Use all 4 Cortex-A76 cores of Pi 5
}

YOLO26Seg::~YOLO26Seg() {
    net.clear();
}

int YOLO26Seg::load(const std::string& param_path, const std::string& bin_path) {
    if (net.load_param(param_path.c_str()) != 0) {
        std::cerr << "Failed to load NCNN param file: " << param_path << std::endl;
        return -1;
    }
    if (net.load_model(bin_path.c_str()) != 0) {
        std::cerr << "Failed to load NCNN bin file: " << bin_path << std::endl;
        return -1;
    }
    return 0;
}

int YOLO26Seg::detect(const cv::Mat& bgr, std::vector<Object>& objects, float prob_threshold, float nms_threshold) {
    int img_w = bgr.cols;
    int img_h = bgr.rows;

    // 1. Preprocessing: resize BGR to 320x320 and convert to RGB, normalize pixels to [0, 1]
    ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR2RGB, img_w, img_h, target_size, target_size);
    const float mean_vals[3] = {0.f, 0.f, 0.f};
    const float norm_vals[3] = {1/255.f, 1/255.f, 1/255.f};
    in.substract_mean_normalize(mean_vals, norm_vals);

    // 2. Run Inference
    ncnn::Extractor ex = net.create_extractor();
    ex.input("in0", in);

    ncnn::Mat out0; // Detection & Mask Coefficients (44 x 2100)
    ncnn::Mat out1; // Prototype Masks (32 x 80 x 80)
    if (ex.extract("out0", out0) != 0 || ex.extract("out1", out1) != 0) {
        std::cerr << "Failed to extract output blobs from NCNN net" << std::endl;
        return -1;
    }

    int num_anchors = out0.w; // 2100
    int num_classes = 12;
    int feat_channels = 32;

    std::vector<Object> proposals;

    // 3. Decode boxes, class scores, and mask coefficients
    for (int i = 0; i < num_anchors; i++) {
        // Find best class
        float max_score = 0.f;
        int class_id = -1;
        for (int c = 0; c < num_classes; c++) {
            float score = out0.row(4 + c)[i];
            if (score > max_score) {
                max_score = score;
                class_id = c;
            }
        }

        if (max_score > prob_threshold) {
            float cx = out0.row(0)[i];
            float cy = out0.row(1)[i];
            float w = out0.row(2)[i];
            float h = out0.row(3)[i];

            float x = cx - w / 2.f;
            float y = cy - h / 2.f;

            Object obj;
            obj.rect.x = x;
            obj.rect.y = y;
            obj.rect.width = w;
            obj.rect.height = h;
            obj.label = class_id;
            obj.prob = max_score;

            obj.mask_feats.resize(feat_channels);
            for (int j = 0; j < feat_channels; j++) {
                obj.mask_feats[j] = out0.row(4 + num_classes + j)[i];
            }

            proposals.push_back(obj);
        }
    }

    // Sort proposals by probability score
    qsort_descent_inplace(proposals);

    // Apply Non-Maximum Suppression (NMS)
    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked, nms_threshold);

    // Scale factors to map box back to original image coordinates
    float scale_x = (float)img_w / target_size;
    float scale_y = (float)img_h / target_size;

    objects.clear();
    for (size_t i = 0; i < picked.size(); i++) {
        int idx = picked[i];
        Object obj = proposals[idx];

        // Map box back to original image
        obj.rect.x *= scale_x;
        obj.rect.y *= scale_y;
        obj.rect.width *= scale_x;
        obj.rect.height *= scale_y;

        // Clamp coordinates
        obj.rect.x = std::max(0.f, std::min(obj.rect.x, (float)(img_w - 1)));
        obj.rect.y = std::max(0.f, std::min(obj.rect.y, (float)(img_h - 1)));
        obj.rect.width = std::max(1.f, std::min(obj.rect.width, (float)(img_w - obj.rect.x)));
        obj.rect.height = std::max(1.f, std::min(obj.rect.height, (float)(img_h - obj.rect.y)));

        // Decode full-resolution binary mask
        decode_mask(out1, obj.mask_feats, obj.rect, obj.mask, cv::Size(img_w, img_h));

        objects.push_back(obj);
    }

    return 0;
}

void YOLO26Seg::decode_mask(const ncnn::Mat& proto, const std::vector<float>& mask_feats, const cv::Rect& rect, cv::Mat& dest_mask, const cv::Size& img_size) {
    int proto_w = proto.w;
    int proto_h = proto.h;
    int proto_c = proto.c;

    cv::Mat mask_80 = cv::Mat::zeros(proto_h, proto_w, CV_32FC1);

    // Linear combination of prototype masks
    for (int c = 0; c < proto_c; c++) {
        float coeff = mask_feats[c];
        const float* proto_ptr = proto.channel(c);

        for (int r = 0; r < proto_h; r++) {
            float* mask_ptr = mask_80.ptr<float>(r);
            for (int col = 0; col < proto_w; col++) {
                mask_ptr[col] += coeff * proto_ptr[r * proto_w + col];
            }
        }
    }

    // Sigmoid function
    for (int r = 0; r < proto_h; r++) {
        float* mask_ptr = mask_80.ptr<float>(r);
        for (int col = 0; col < proto_w; col++) {
            mask_ptr[col] = 1.0f / (1.0f + std::exp(-mask_ptr[col]));
        }
    }

    // Scale rect coordinates to prototype space (80x80)
    float x_scale = (float)proto_w / img_size.width;
    float y_scale = (float)proto_h / img_size.height;

    int rx = std::round(rect.x * x_scale);
    int ry = std::round(rect.y * y_scale);
    int rw = std::round(rect.width * x_scale);
    int rh = std::round(rect.height * y_scale);

    // Clamp inside prototype dimensions
    rx = std::max(0, std::min(rx, proto_w - 1));
    ry = std::max(0, std::min(ry, proto_h - 1));
    rw = std::max(1, std::min(rw, proto_w - rx));
    rh = std::max(1, std::min(rh, proto_h - ry));

    cv::Mat cropped_mask = mask_80(cv::Rect(rx, ry, rw, rh));

    // Resize back to original bounding box size
    cv::Mat resized_mask;
    cv::resize(cropped_mask, resized_mask, rect.size(), 0, 0, cv::INTER_LINEAR);

    dest_mask = cv::Mat::zeros(img_size, CV_8UC1);
    cv::Mat mask_roi = dest_mask(rect);

    for (int r = 0; r < rect.height; r++) {
        const float* res_ptr = resized_mask.ptr<float>(r);
        uchar* roi_ptr = mask_roi.ptr<uchar>(r);
        for (int col = 0; col < rect.width; col++) {
            if (res_ptr[col] > 0.5f) {
                roi_ptr[col] = 255;
            } else {
                roi_ptr[col] = 0;
            }
        }
    }
}

void YOLO26Seg::draw(cv::Mat& image, const std::vector<Object>& objects) {
    cv::Mat overlay = image.clone();

    for (size_t i = 0; i < objects.size(); i++) {
        const Object& obj = objects[i];
        cv::Scalar color = class_colors[obj.label % class_colors.size()];

        // Draw overlay transparent mask
        if (!obj.mask.empty()) {
            overlay.setTo(color, obj.mask);
        }

        // Draw bounding box
        cv::rectangle(image, obj.rect, color, 2);

        // Draw label text
        char text[256];
        sprintf(text, "%s %.1f%%", class_names[obj.label].c_str(), obj.prob * 100);
        
        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        int x = obj.rect.x;
        int y = obj.rect.y - label_size.height - 2;
        if (y < 0) y = 0;
        if (x + label_size.width > image.cols) x = image.cols - label_size.width;

        cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)), color, -1);
        cv::putText(image, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    // Blend overlay into the original image
    cv::addWeighted(overlay, 0.4, image, 0.6, 0, image);
}

void YOLO26Seg::qsort_descent_inplace(std::vector<Object>& objects, int left, int right) {
    int i = left;
    int j = right;
    float p = objects[(left + right) / 2].prob;

    while (i <= j) {
        while (objects[i].prob > p) i++;
        while (objects[j].prob < p) j--;
        if (i <= j) {
            std::swap(objects[i], objects[j]);
            i++;
            j--;
        }
    }

    if (left < j) qsort_descent_inplace(objects, left, j);
    if (i < right) qsort_descent_inplace(objects, i, right);
}

void YOLO26Seg::qsort_descent_inplace(std::vector<Object>& objects) {
    if (objects.empty()) return;
    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

void YOLO26Seg::nms_sorted_bboxes(const std::vector<Object>& objects, std::vector<int>& picked, float nms_threshold) {
    picked.clear();
    const int n = objects.size();
    std::vector<float> areas(n);
    for (int i = 0; i < n; i++) {
        areas[i] = objects[i].rect.area();
    }

    for (int i = 0; i < n; i++) {
        const Object& a = objects[i];
        bool keep = true;
        for (int j = 0; j < (int)picked.size(); j++) {
            const Object& b = objects[picked[j]];
            float inter = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter;
            if (inter / union_area > nms_threshold) {
                keep = false;
                break;
            }
        }
        if (keep) {
            picked.push_back(i);
        }
    }
}

float YOLO26Seg::intersection_area(const Object& a, const Object& b) {
    cv::Rect_<float> rect = a.rect & b.rect;
    return rect.area();
}
