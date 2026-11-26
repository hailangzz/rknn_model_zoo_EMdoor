// yolov8_api.h
#pragma once
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

/* ================= box ================= */

typedef struct
{
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
// 玻璃语义建图约定：
// - add_edge_point_... : 玻璃底端接地线（或悬空底边投影线）的三维点，按从左到右排列
// - object_contours_mark_point : 对应的二维底边折线（调试/可视化）
// - coords[0]=左端点, coords[1]=右端点；coords[2]/coords[3] 预留兼容
// 注意：不要随意在本结构体尾部加字段，否则会与板端 ROS 节点旧 ABI 不兼容导致崩溃
typedef struct {
    ObjectTargetBox target_box;
    float prop;                      // 置信度
    int cls_id;                      // 类别 ID
    std::vector<CameraCoordinate> coords;
    std::vector<CameraCoordinate> add_edge_point_single_pixel_camera_coordinates;
    std::vector<std::vector<cv::Point>> object_contours_mark_point;
} ObjectCameraDetectResult;


bool base_model_init(
    const char *config_path);

/* ================= 普通推理 ================= */

bool real_detect_infer(
    const cv::Mat &img,
    std::vector<ObjectCameraDetectResult> &results);

/* ================= 带位姿采样推理 ================= */

bool base_detect_infer(
    const cv::Mat &img,
    std::vector<ObjectCameraDetectResult> &results,
    const Eigen::Vector3d &position,
    const Eigen::Quaterniond &q);

/* ================= 释放 ================= */

void base_model_release();
