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

/* ================= 单个3D点 ================= */

typedef struct
{
    float X;
    float Y;
    float Z;

} CameraCoordinate;

/* ================= 检测结果 ================= */

typedef struct
{
    ObjectTargetBox target_box;

    float prop;

    int cls_id;

    std::vector<CameraCoordinate> coords;

    std::vector<CameraCoordinate>
        add_edge_point_single_pixel_camera_coordinates;

    std::vector<std::vector<cv::Point>>
        object_contours_mark_point;

} ObjectCameraDetectResult;

/* ================= 初始化 ================= */

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