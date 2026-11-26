#pragma once
#include <string>
#include "rknn_api.h"
#include "common.h"

#if defined(RV1106_1103)
typedef struct
{
    char *dma_buf_virt_addr;
    int dma_buf_fd;
    int size;
} rknn_dma_buf;
#endif

typedef struct
{
    rknn_context rknn_ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr *input_attrs;
    rknn_tensor_attr *output_attrs;
#if defined(RV1106_1103)
    rknn_tensor_mem *input_mems[1];
    rknn_tensor_mem *output_mems[9];
    rknn_dma_buf img_dma_buf;
#endif
#if defined(ZERO_COPY)
    rknn_tensor_mem *input_mems[1];
    rknn_tensor_mem *output_mems[9];
    rknn_tensor_attr *input_native_attrs;
    rknn_tensor_attr *output_native_attrs;
#endif
    int model_channel;
    int model_width;
    int model_height;
    int input_image_width;
    int input_image_height;
    bool is_quant;
} rknn_app_context_t;

struct ConfigInfo
{

    std::string task_name;
    std::string model_path;
    int input_width;
    int input_height;
    float score_threshold;
    int max_frame_threshold;
    int BASE_AREA; // 基础目标的classid，用于建图时，确认世界地图上障碍物构建，基础区域的重要程度

    float camera_z_axle_top_resize_rate; // 相机坐标系，Z轴远端缩放比例
    float camera_z_axle_polyfit_w0;      // 多项式拟合，二次项系数
    float camera_z_axle_polyfit_w1;      // 多项式拟合，一次项系数
    float camera_z_axle_polyfit_w2;      // 多项式拟合，常数项系数

    float camera_fx;
    float camera_fy;
    float camera_cx;
    float camera_cy;
    float camera_H;
    float camera_pitch;
    float camera_D_0;
    float camera_D_1;
    float camera_D_2;
    float camera_D_3;
    float camera_D_4;
    float camera_D_5;
    float camera_D_6;
    float camera_D_7;

    // 后处理几何过滤（单位：米）
    // 最近像素深度 > max_detect_distance_m 的目标直接丢弃
    float max_detect_distance_m;
    // 最近像素深度过近（无效地面交点/异常）也丢弃
    float min_detect_distance_m;
    // 有效边缘三维点最少数量
    int min_edge_points;
    // 目标地面投影宽度合法性范围
    float min_object_width_m;
    float max_object_width_m;

    // 玻璃底端接地线提取 / 悬空门窗
    // 按列取底部轮廓点时的水平采样步长（像素）
    int bottom_line_sample_step_px;
    // 底边 y 中值滤波窗口（奇数，越大越稳）
    int bottom_line_median_k;
    // 是否将底边拟合为直线（玻璃门底边近似直线，建图更稳）
    bool bottom_line_fit_straight;
    // 拟合直线上输出采样点数
    int bottom_line_fit_sample_num;
    // (img_h - box.bottom)/img_h 超过该比例，视为底边未接地（悬空窗）
    float floating_bottom_gap_ratio;
    // 悬空时假设玻璃底边离地高度（米），用于射线与水平面求交
    float glass_floating_assume_height_m;

    float debug_score_threshold;
    std::string save_debug_images_path;
    std::string save_images_path_spatial_location_val;
    bool is_save_debug_images;
    int fps_limit;
};
