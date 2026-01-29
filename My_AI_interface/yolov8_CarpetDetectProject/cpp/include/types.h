#pragma once
#include <string>
#include "rknn_api.h"
#include "common.h"

#if defined(RV1106_1103) 
    typedef struct {
        char *dma_buf_virt_addr;
        int dma_buf_fd;
        int size;
    }rknn_dma_buf;
#endif

typedef struct {
    rknn_context rknn_ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr* input_attrs;
    rknn_tensor_attr* output_attrs;
#if defined(RV1106_1103) 
    rknn_tensor_mem* input_mems[1];
    rknn_tensor_mem* output_mems[9];
    rknn_dma_buf img_dma_buf;
#endif
#if defined(ZERO_COPY)  
    rknn_tensor_mem* input_mems[1];
    rknn_tensor_mem* output_mems[9];
    rknn_tensor_attr* input_native_attrs;
    rknn_tensor_attr* output_native_attrs;
#endif
    int model_channel;
    int model_width;
    int model_height;
    bool is_quant;
} rknn_app_context_t;

struct ConfigInfo {

    std::string model_path;
    int input_width;
    int input_height;
    float score_threshold;
    int max_frame_threshold;
    int CARPET_AREA;                     // 地毯目标的classid，用于建图时，确认世界地图上障碍物构建，地毯区域的重要程度

    float camera_z_axle_top_resize_rate; // 相机坐标系，Z轴远端缩放比例
    float camera_z_axle_polyfit_w0; // 多项式拟合，二次项系数
    float camera_z_axle_polyfit_w1; // 多项式拟合，一次项系数
    float camera_z_axle_polyfit_w2; // 多项式拟合，常数项系数
    
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

};

