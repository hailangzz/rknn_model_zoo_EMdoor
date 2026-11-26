#ifndef _RKNN_YOLOV8_DEMO_POSTPROCESS_H_
#define _RKNN_YOLOV8_DEMO_POSTPROCESS_H_

#include <stdint.h>
#include <vector>
#include "rknn_api.h"
#include "common.h"
#include "image_utils.h"
#include "types.h"
#include <opencv2/opencv.hpp>
#include "rknn_matmul_api.h"
#include <sys/time.h>
#include "easy_timer.h"
#include "base_detect_interface.h"
#define OBJ_NAME_MAX_SIZE 64
#define OBJ_NUMB_MAX_SIZE 128
#define OBJ_CLASS_NUM 1
#define NMS_THRESH 0.45
#define BOX_THRESH 0.35
#define PROP_BOX_SIZE (5 + OBJ_CLASS_NUM)

#define PROTO_CHANNEL 32
// #define PROTO_HEIGHT 160
// #define PROTO_WEIGHT 160
#define PROTO_HEIGHT 240
#define PROTO_WEIGHT 240
// #define PROTO_HEIGHT 240
// #define PROTO_WEIGHT 144
// #define PROTO_HEIGHT 320
// #define PROTO_WEIGHT 320

#define N_CLASS_COLORS 20

// class rknn_app_context_t;

typedef struct
{
    image_rect_t box;
    float prop;
    int cls_id;
    box_camera_coordinates camera_coordinates; // 新增相机坐标信息

} object_detect_result;

typedef struct
{
    uint8_t *seg_mask;
} object_segment_result;

typedef struct
{
    int id;
    int count;
    object_detect_result results[OBJ_NUMB_MAX_SIZE];
    object_segment_result results_seg[OBJ_NUMB_MAX_SIZE];
} object_detect_result_list;

int init_post_process();
void deinit_post_process();
char *coco_cls_to_name(int cls_id);
int post_process(rknn_app_context_t *app_ctx, rknn_output *outputs, letterbox_t *letter_box, float conf_threshold, float nms_threshold, object_detect_result_list *od_results);
int clamp(float val, int min, int max);

void extract_seg_mask_contours(
    object_segment_result *seg, // 只操作单个目标的 seg_mask
    int width,
    int height,
    std::vector<std::vector<cv::Point>> &out_contours);

void smoothContour(
    const std::vector<cv::Point> &input,
    std::vector<cv::Point> &output,
    int win = 3);

// 局部凸包平滑
void smoothContourOuterOnly(
    const std::vector<cv::Point> &input,
    std::vector<cv::Point> &output,
    float smooth_factor = 0.7f);

void fillCameraDetectResult(const object_detect_result *det, ObjectCameraDetectResult &one, ConfigInfo &config);

// Z 轴多项式距离校正（与 fillCameraDetectResult 一致）
float applyDistancePolyfit(float raw_z, const ConfigInfo &config);

void filter_mask_contours(const std::vector<std::vector<cv::Point>> &input_contours, std::vector<std::vector<cv::Point>> &output_contours);

// 从分割轮廓提取玻璃底端接地线（旧方法，易抖动）
bool extractGlassBottomContactLine(
    const std::vector<std::vector<cv::Point>> &contours,
    int sample_step_px,
    std::vector<cv::Point> &bottom_line_px);

// 稳定提取：在检测框内从框底向上扫 mask，得到 mark 下边缘
// （即“框底与 mark 之间空隙区域”的上边界），再中值平滑 + 可选直线拟合
bool extractStableGlassBottomLineFromMask(
    const uint8_t *seg_mask,
    int width,
    int height,
    const image_rect_t &box,
    int sample_step_px,
    int median_k,
    bool fit_straight_line,
    int fit_sample_num,
    std::vector<cv::Point> &bottom_line_px);

// 根据目标框底部相对图像底部的空隙，判断是否接地
bool isGlassBottomGroundContact(const image_rect_t &box, int image_height, float floating_gap_ratio);

// 过滤近地平线异常点，并做 Z 多项式校正；宽度取左右端点直线距离
bool sanitizeGlassBottomLineXYZ(
    const std::vector<CameraCoordinate> &raw_xyz,
    const ConfigInfo &config,
    std::vector<CameraCoordinate> &out_xyz,
    float *line_width_m,
    float *nearest_distance_m);

// 取目标轮廓/角点中最近有效深度（米）；无效返回负数
float getNearestDetectDistance(const ObjectCameraDetectResult &one);

// 玻璃障碍物几何后处理过滤：距离/边缘点数/尺寸合法性
bool isGlassObstacleValid(const ObjectCameraDetectResult &one, const ConfigInfo &config, float *nearest_distance_m = nullptr);

// 释放 object_detect_result 中动态分配的边缘点
void releaseDetectEdgePoints(object_detect_result *det);

#endif //_RKNN_YOLOV8_DEMO_POSTPROCESS_H_
