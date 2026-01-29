#ifndef _RKNN_YOLOV8_DEMO_POSTPROCESS_H_
#define _RKNN_YOLOV8_DEMO_POSTPROCESS_H_

#include <stdint.h>
#include <vector>
#include "rknn_api.h"
#include "common.h"
#include "image_utils.h"
#include "types.h"

#define OBJ_NAME_MAX_SIZE 64
#define OBJ_NUMB_MAX_SIZE 128
#define OBJ_CLASS_NUM 1
#define NMS_THRESH 0.45
#define BOX_THRESH 0.65

// class rknn_app_context_t;

typedef struct {
    image_rect_t box;
    float prop;
    int cls_id;
    box_camera_coordinates camera_coordinates;   //新增相机坐标信息

} object_detect_result;

typedef struct {
    int id;
    int count;
    object_detect_result results[OBJ_NUMB_MAX_SIZE];
} object_detect_result_list;

// int init_post_process();
int init_post_process(const char *labels_info);
void deinit_post_process();
char *coco_cls_to_name(int cls_id);
int post_process(rknn_app_context_t *app_ctx, void *outputs, letterbox_t *letter_box, float conf_threshold, float nms_threshold, object_detect_result_list *od_results);
void deinit_post_process();
inline bool pointInEllipse(float x, float y,float cx, float cy,float a, float b);
bool bboxEllipseOverlapRatio(int x1, int y1, int x2, int y2,int center_x, int center_y,int axes_w, int axes_h,float threshold,int samples = 20);   // samples采样密度，20~30 通常够用


#endif //_RKNN_YOLOV8_DEMO_POSTPROCESS_H_
