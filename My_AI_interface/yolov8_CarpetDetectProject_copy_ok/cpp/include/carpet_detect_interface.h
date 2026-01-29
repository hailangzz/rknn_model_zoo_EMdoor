// yolov8_api.h
#pragma once
#include <opencv2/opencv.hpp>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int left;
    int top;
    int right;
    int bottom;
} ObjectTargetBox;

// 定义单个 3D 坐标结构体
typedef struct {
    float X;
    float Y;
    float Z;
} CameraCoordinate;

// 定义检测结果结构体
typedef struct {    
    ObjectTargetBox target_box;
    float prop;                     // 置信度
    int cls_id;                      // 类别 ID
    CameraCoordinate coords[4];      // 最多存储 4 个 3D 坐标点                // 实际存储的坐标点数量

} ObjectCameraDetectResult;


bool carpet_model_init(const char* config_path);
bool carpet_detect_infer(const cv::Mat& img,std::vector<ObjectCameraDetectResult>& results);
void carpet_model_release();

#ifdef __cplusplus
}
#endif
