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
#include "carpet_detect_interface.h"
#define OBJ_NAME_MAX_SIZE 64
#define OBJ_NUMB_MAX_SIZE 128
#define OBJ_CLASS_NUM 1
#define NMS_THRESH 0.45
#define BOX_THRESH 0.55
#define PROP_BOX_SIZE (5 + OBJ_CLASS_NUM)

#define PROTO_CHANNEL 32
#define PROTO_HEIGHT 160
#define PROTO_WEIGHT 160

#define N_CLASS_COLORS 20


// class rknn_app_context_t;

typedef struct {
    image_rect_t box;
    float prop;
    int cls_id;
    box_camera_coordinates camera_coordinates;   //新增相机坐标信息

} object_detect_result;

typedef struct
{
    uint8_t *seg_mask;
} object_segment_result;

typedef struct {
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
void deinit_post_process();
void extract_seg_mask_contours(object_detect_result_list &od_results,
                               int target_index,
                               int width, int height,
                               std::vector<std::vector<cv::Point>> &out_contours);
void smoothContour(
    const std::vector<cv::Point>& input,
    std::vector<cv::Point>& output,
    int win = 3);

void fillCameraDetectResult(const object_detect_result* det, ObjectCameraDetectResult& one, ConfigInfo & config);


#endif //_RKNN_YOLOV8_DEMO_POSTPROCESS_H_
