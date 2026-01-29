// yolov8_api.h
#pragma once
#include <opencv2/opencv.hpp>

#ifdef __cplusplus
extern "C" {
#endif

bool carpet_model_init(const char* model_path);
bool carpet_detect_infer(const cv::Mat& img);
void carpet_model_release();

#ifdef __cplusplus
}
#endif
