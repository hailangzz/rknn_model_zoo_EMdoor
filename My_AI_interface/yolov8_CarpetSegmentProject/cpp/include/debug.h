#pragma once

#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <iostream>
#include <iomanip>

#include "image_utils.h"

#include <mutex>

class Debug {
public:
    Debug(const std::string& debug_image_save_path, bool is_save_debug_image = false);

    // 保存图像（检测到目标时调用）
    void saveIfDetected(const cv::Mat& image, const std::string& tag = "");

private:
    std::string generateFileName(const std::string& tag);

    std::mutex mutex_;
    std::string debug_image_save_path_;
    bool is_save_debug_image_=false;
};