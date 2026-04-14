// yolov8_api.h
#pragma once
#include <opencv2/opencv.hpp>
#ifdef __cplusplus
extern "C" {
#endif


struct AndroidImageNV21 {
    uint8_t* image_input_nv21;   // NV21 数据
    int image_width;
    int image_height;
};

enum class HandDetectResult {
    Processing,   // 上一帧还未处理完成
    NoHand,       // 本帧处理完成，没有检测到手
    HandDetected  // 本帧处理完成，检测到手
};

void hand_detect_set_config_path(const std::string& path);
// 目标检测接口（使用指针，更兼容 C/JNI）
// bool hand_detect_interface(AndroidImageNV21* image_object_input);
// bool hand_detect_interface(AndroidImageNV21* image_object_input,bool is_save_images);
HandDetectResult hand_detect_interface(AndroidImageNV21* image_object_input, bool is_save_images);

#ifdef __cplusplus
}
#endif
