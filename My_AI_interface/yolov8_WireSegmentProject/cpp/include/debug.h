#pragma once

#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <chrono>
#include <fstream>

#include "image_utils.h"

#include <mutex>

class Debug
{
public:
    Debug(const std::string &debug_image_save_path, bool is_save_debug_image = false, int fps_limit = 5);

    // 保存图像（检测到目标时调用）
    void saveIfDetected(const cv::Mat &image, const std::string &tag = "");
    // 保存图像+轮廓点图（检测到目标时调用）
    void saveIfDetectedAddContours(
        const cv::Mat &image,
        const std::vector<std::vector<cv::Point>> &contours,
        const std::vector<int> &cls_ids,
        const std::string &tag);

private:
    std::string generateFileName(const std::string &tag);

    std::mutex mutex_;
    std::string debug_image_save_path_;
    bool is_save_debug_image_ = false;

    int fps_limit_ = 5; // 每秒图像存储帧数
    std::atomic<int64_t> last_save_time_ms_{0};
    int save_interval_ms_; // 默认 5 FPS（每秒存储5帧）
};