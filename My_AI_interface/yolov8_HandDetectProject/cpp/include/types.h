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
    std::string labels_info;
    int input_width;
    int input_height;
    float score_threshold;
    int max_frame_threshold;
    std::string debug_nv21_image_saver;
    int center_x;
    int center_y;
    int axes_w;
    int axes_h;
    float target_effective_area_iou_thread;
    float max_detect_duration_s;  
    float block_duration_s;    
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
