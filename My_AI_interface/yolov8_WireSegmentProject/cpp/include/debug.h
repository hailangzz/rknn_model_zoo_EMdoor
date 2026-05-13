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
    void saveSegLabel(const cv::Mat &image, const std::vector<std::vector<cv::Point>> &contours, const std::vector<int> &cls_ids, const std::string &tag = "");

    // 更改图像保存路径
    void setDebugImageSavePath(const std::string &path);

    // 计算基于空间位置的图像保存的计数
    void updateSavedPoseImageCount(bool is_exist_target);
    int getSavedPoseImageCount(bool is_exist_target);

private:
    std::string generateFileName(const std::string &tag);

    std::mutex mutex_;
    std::string debug_image_save_path_;
    bool is_save_debug_image_ = false;

    int fps_limit_ = 5; // 每秒图像存储帧数
    std::atomic<int64_t> last_save_time_ms_{0};
    int save_interval_ms_; // 默认 5 FPS（每秒存储5帧）

    int saved_pose_exist_target_image_count_ = 0; // 基于空间位置，存在目标的图像数量
    int saved_pose_null_target_image_count_ = 0;  // 基于空间位置，不存在目标的图像数量
};