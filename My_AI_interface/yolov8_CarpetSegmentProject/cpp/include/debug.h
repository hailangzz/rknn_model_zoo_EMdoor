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
#include <dirent.h>
#include <unistd.h>
#include <sys/types.h>
#include <sstream>
#include <iomanip>

enum class TargetStatus
{
    EXISTS = 2, // 有目标（高置信）
    MIDDLE = 1, // 有一定置信，但不稳定
    NONE = 0    // 没目标
};

struct ai_capture_info
{
    // 设备sn码，唯一标识
    std::string device_id_sn;
    // AI任务类型
    std::string model_task_type;
    // 是否检测到目标
    std::string has_target = "";
    // 检测置信度
    float confidence = 0.0f;
};
// 将 TargetStatus 转换为字符串 内联函数
inline const char *TargetStatusToStr(TargetStatus status)
{
    switch (status)
    {
    case TargetStatus::EXISTS:
        return "exist";

    case TargetStatus::MIDDLE:
        return "middle";

    case TargetStatus::NONE:
        return "null";

    default:
        return "unknown";
    }
}

class Debug
{
public:
    Debug(const std::string &debug_image_save_path, bool is_save_debug_image = false, int fps_limit = 5);

    // 保存图像（检测到目标时调用）
    void saveIfDetected(const cv::Mat &image, const std::string &tag = "");
    void saveSegLabel(
        const cv::Mat &image,
        const std::vector<std::vector<cv::Point>> &contours,
        const std::vector<int> &cls_ids,
        const std::string &task_name,
        const std::string &save_sample_info_string,
        TargetStatus status);

    // 更改图像保存路径
    void setDebugImageSavePath(const std::string &path);

    // 计算基于空间位置的图像保存的计数
    void updateSavedPoseImageCount(TargetStatus status);
    int getSavedPoseImageCount(TargetStatus status);

    // 获取SN码
    std::string device_id_sn = ""; // 设备SN码
    std::string getDeviceSN(const std::string &file_path);
    // 设置AI自动化迭代的图像保存信息
    std::string set_ai_capture_save_info(
        const std::string &device_id_sn,
        const std::string &model_task_type,
        const std::string &target_status,
        float confidence);

    int64_t getCurrentTimestampMs();

    std::string buildSaveDirectory(
        const std::string &task_name,
        TargetStatus status);

    // 删除旧数据
    void removeExpiredDirectories();

private:
    std::string generateFileName(const std::string &tag);

    std::mutex mutex_;
    std::string debug_image_save_path_;
    bool is_save_debug_image_ = false;

    int fps_limit_ = 5; // 每秒图像存储帧数
    std::atomic<int64_t> last_save_time_ms_{0};
    int save_interval_ms_; // 默认 5 FPS（每秒存储5帧）

    int saved_pose_exist_target_image_count_ = 0;  // 基于空间位置，存在目标的图像数量
    int saved_pose_middle_target_image_count_ = 0; // 基于空间位置，存在目标的图像数量
    int saved_pose_null_target_image_count_ = 0;   // 基于空间位置，不存在目标的图像数量

    // 已下为AI自动化迭代相关功能

    const std::string produce_info_path_ = "/oem/produce.info"; // 设备信息文件路径

    ai_capture_info ai_capture_save_info_; // AI自动化迭代的图像保存信息
    // 图像存储路径结构设计：
    // 数据保留天数
    int keep_days_ = 5;
    // 获取当前日期 YYYYMMDD
    std::string getCurrentDate();

    // 递归创建目录
    bool createDirectoryRecursive(
        const std::string &path);
};