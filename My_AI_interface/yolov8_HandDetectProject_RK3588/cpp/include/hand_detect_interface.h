#pragma once

#include <memory>
#include <string>
#include <opencv2/opencv.hpp>

// ================= 检测结果 =================
enum class HandDetectResult
{
    Processing,  // 上一帧还未处理完成
    NoHand,      // 本帧处理完成，没有检测到手
    HandDetected // 本帧处理完成，检测到手
};

enum class HandDetectCameraInfo
{
    TopHandCamera,   // 货架上端摄像头，输入标志
    BottomHandCamera // 货架下端摄像头，输入标志
};

// ================= 配置接口 =================
void hand_detect_set_config_path(const std::string &path);

// ================= 推荐主接口（OpenCV Mat 共享指针） =================
HandDetectResult hand_detect_interface(std::shared_ptr<cv::Mat> image_object_input, HandDetectCameraInfo camera_info, bool is_save_images);