// Copyright (c) 2021 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "yolov8_detect.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include <set>
#include <vector>
#define LABEL_NALE_TXT_PATH "./model/coco_80_labels_list.txt"

static char *labels[OBJ_CLASS_NUM];

int clamp(float val, int min, int max)
{
    return val > min ? (val < max ? val : max) : min;
}

static char *readLine(FILE *fp, char *buffer, int *len)
{
    int ch;
    int i = 0;
    size_t buff_len = 0;

    buffer = (char *)malloc(buff_len + 1);
    if (!buffer)
        return NULL; // Out of memory

    while ((ch = fgetc(fp)) != '\n' && ch != EOF)
    {
        buff_len++;
        void *tmp = realloc(buffer, buff_len + 1);
        if (tmp == NULL)
        {
            free(buffer);
            return NULL; // Out of memory
        }
        buffer = (char *)tmp;

        buffer[i] = (char)ch;
        i++;
    }
    buffer[i] = '\0';

    *len = buff_len;

    // Detect end
    if (ch == EOF && (i == 0 || ferror(fp)))
    {
        free(buffer);
        return NULL;
    }
    return buffer;
}

static int readLines(const char *fileName, char *lines[], int max_line)
{
    FILE *file = fopen(fileName, "r");
    char *s;
    int i = 0;
    int n = 0;

    if (file == NULL)
    {
        printf("Open %s fail!\n", fileName);
        return -1;
    }

    while ((s = readLine(file, s, &n)) != NULL)
    {
        lines[i++] = s;
        if (i >= max_line)
            break;
    }
    fclose(file);
    return i;
}

static int loadLabelName(const char *locationFilename, char *label[])
{
    printf("load lable %s\n", locationFilename);
    readLines(locationFilename, label, OBJ_CLASS_NUM);
    return 0;
}

static float CalculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0, float xmin1, float ymin1, float xmax1,
                              float ymax1)
{
    float w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0);
    float h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0);
    float i = w * h;
    float u = (xmax0 - xmin0 + 1.0) * (ymax0 - ymin0 + 1.0) + (xmax1 - xmin1 + 1.0) * (ymax1 - ymin1 + 1.0) - i;
    return u <= 0.f ? 0.f : (i / u);
}

static int nms(int validCount, std::vector<float> &outputLocations, std::vector<int> classIds, std::vector<int> &order,
               int filterId, float threshold)
{
    for (int i = 0; i < validCount; ++i)
    {
        int n = order[i];
        if (n == -1 || classIds[n] != filterId)
        {
            continue;
        }
        for (int j = i + 1; j < validCount; ++j)
        {
            int m = order[j];
            if (m == -1 || classIds[m] != filterId)
            {
                continue;
            }
            float xmin0 = outputLocations[n * 4 + 0];
            float ymin0 = outputLocations[n * 4 + 1];
            float xmax0 = outputLocations[n * 4 + 0] + outputLocations[n * 4 + 2];
            float ymax0 = outputLocations[n * 4 + 1] + outputLocations[n * 4 + 3];

            float xmin1 = outputLocations[m * 4 + 0];
            float ymin1 = outputLocations[m * 4 + 1];
            float xmax1 = outputLocations[m * 4 + 0] + outputLocations[m * 4 + 2];
            float ymax1 = outputLocations[m * 4 + 1] + outputLocations[m * 4 + 3];

            float iou = CalculateOverlap(xmin0, ymin0, xmax0, ymax0, xmin1, ymin1, xmax1, ymax1);

            if (iou > threshold)
            {
                order[j] = -1;
            }
        }
    }
    return 0;
}

static int quick_sort_indice_inverse(std::vector<float> &input, int left, int right, std::vector<int> &indices)
{
    float key;
    int key_index;
    int low = left;
    int high = right;
    if (left < right)
    {
        key_index = indices[left];
        key = input[left];
        while (low < high)
        {
            while (low < high && input[high] <= key)
            {
                high--;
            }
            input[low] = input[high];
            indices[low] = indices[high];
            while (low < high && input[low] >= key)
            {
                low++;
            }
            input[high] = input[low];
            indices[high] = indices[low];
        }
        input[low] = key;
        indices[low] = key_index;
        quick_sort_indice_inverse(input, left, low - 1, indices);
        quick_sort_indice_inverse(input, low + 1, right, indices);
    }
    return low;
}

void resize_by_opencv_fp(float *input_image, int input_width, int input_height, int boxes_num, float *output_image, int target_width, int target_height)
{
    for (int b = 0; b < boxes_num; b++)
    {
        cv::Mat src_image(input_height, input_width, CV_32F, &input_image[b * input_width * input_height]);
        cv::Mat dst_image;
        cv::resize(src_image, dst_image, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
        memcpy(&output_image[b * target_width * target_height], dst_image.data, target_width * target_height * sizeof(float));
    }
}

void resize_by_opencv_uint8(uint8_t *input_image, int input_width, int input_height, int boxes_num, uint8_t *output_image, int target_width, int target_height)
{
    for (int b = 0; b < boxes_num; b++)
    {
        cv::Mat src_image(input_height, input_width, CV_8U, &input_image[b * input_width * input_height]);
        cv::Mat dst_image;
        cv::resize(src_image, dst_image, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
        memcpy(&output_image[b * target_width * target_height], dst_image.data, target_width * target_height * sizeof(uint8_t));
    }
}

class DrmObject
{
public:
    int drm_buffer_fd;
    int drm_buffer_handle;
    size_t actual_size;
    uint8_t *drm_buf;
};

void crop_mask_fp(float *seg_mask, uint8_t *all_mask_in_one, float *boxes, int boxes_num, int *cls_id, int height, int width)
{
    for (int b = 0; b < boxes_num; b++)
    {
        float x1 = boxes[b * 4 + 0];
        float y1 = boxes[b * 4 + 1];
        float x2 = boxes[b * 4 + 2];
        float y2 = boxes[b * 4 + 3];

        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                if (j >= x1 && j < x2 && i >= y1 && i < y2)
                {
                    if (all_mask_in_one[i * width + j] == 0)
                    {
                        if (seg_mask[b * width * height + i * width + j] > 0)
                        {
                            all_mask_in_one[i * width + j] = (cls_id[b] + 1);
                        }
                        else
                        {
                            all_mask_in_one[i * width + j] = 0;
                        }
                    }
                }
            }
        }
    }
}

void crop_mask_uint8(uint8_t *seg_mask, uint8_t *all_mask_in_one, float *boxes, int boxes_num, int *cls_id, int height, int width)
{
    for (int b = 0; b < boxes_num; b++)
    {
        float x1 = boxes[b * 4 + 0];
        float y1 = boxes[b * 4 + 1];
        float x2 = boxes[b * 4 + 2];
        float y2 = boxes[b * 4 + 3];

        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                if (j >= x1 && j < x2 && i >= y1 && i < y2)
                {
                    if (all_mask_in_one[i * width + j] == 0)
                    {
                        if (seg_mask[b * width * height + i * width + j] > 0)
                        {
                            all_mask_in_one[i * width + j] = (cls_id[b] + 1);
                        }
                        else
                        {
                            all_mask_in_one[i * width + j] = 0;
                        }
                    }
                }
            }
        }
    }
}

void matmul_by_cpu_fp(std::vector<float> &A, float *B, float *C, int ROWS_A, int COLS_A, int COLS_B)
{

    float temp = 0;
    for (int i = 0; i < ROWS_A; i++)
    {
        for (int j = 0; j < COLS_B; j++)
        {
            temp = 0;
            for (int k = 0; k < COLS_A; k++)
            {
                temp += A[i * COLS_A + k] * B[k * COLS_B + j];
            }
            C[i * COLS_B + j] = temp;
        }
    }
}

void matmul_by_cpu_uint8(std::vector<float> &A, float *B, uint8_t *C, int ROWS_A, int COLS_A, int COLS_B)
{

    float temp = 0;
    for (int i = 0; i < ROWS_A; i++)
    {
        for (int j = 0; j < COLS_B; j++)
        {
            temp = 0;
            for (int k = 0; k < COLS_A; k++)
            {
                temp += A[i * COLS_A + k] * B[k * COLS_B + j];
            }
            if (temp > 0)
            {
                C[i * COLS_B + j] = 4;
            }
            else
            {
                C[i * COLS_B + j] = 0;
            }
        }
    }
}

// // 以下seg_reverse函数，作用是将模型输出的分割结果（seg_mask）进行反向处理，得到与原始输入图像尺寸相匹配的分割掩码（seg_mask_real）。
// // 注：此函数仅适用宽、高相等的输入输出情况（如640x640），如果输入输出尺寸不相等，需根据实际情况调整反向处理逻辑。
// void seg_reverse(uint8_t *seg_mask, uint8_t *cropped_seg, uint8_t *seg_mask_real,
//                  int model_in_height, int model_in_width, int cropped_height, int cropped_width, int ori_in_height, int ori_in_width, int y_pad, int x_pad)
// {

//     if (y_pad == 0 && x_pad == 0 && ori_in_height == model_in_height && ori_in_width == model_in_width)
//     {
//         memcpy(seg_mask_real, seg_mask, ori_in_height * ori_in_width);
//         return;
//     }

//     int cropped_index = 0;
//     for (int i = 0; i < model_in_height; i++)
//     {
//         for (int j = 0; j < model_in_width; j++)
//         {
//             if (i >= y_pad && i < model_in_height - y_pad && j >= x_pad && j < model_in_width - x_pad)
//             {
//                 int seg_index = i * model_in_width + j;
//                 cropped_seg[cropped_index] = seg_mask[seg_index];
//                 cropped_index++;
//             }
//         }
//     }
//     // Note: Here are different methods provided for implementing single-channel image scaling.
//     //       The method of using rga to resize the image requires that the image size is 2 aligned.
//     resize_by_opencv_uint8(cropped_seg, cropped_width, cropped_height, 1, seg_mask_real, ori_in_width, ori_in_height);
//     // resize_by_rga_rk356x(cropped_seg, cropped_width, cropped_height, seg_mask_real, ori_in_width, ori_in_height);
//     // resize_by_rga_rk3588(cropped_seg, cropped_width, cropped_height, seg_mask_real, ori_in_width, ori_in_height);
// }

void seg_reverse(uint8_t *seg_mask, uint8_t *cropped_seg, uint8_t *seg_mask_real,
                 int model_in_height, int model_in_width,
                 int cropped_height, int cropped_width,
                 int ori_in_height, int ori_in_width,
                 int y_pad, int x_pad)
{
    if (y_pad == 0 && x_pad == 0 && ori_in_height == model_in_height && ori_in_width == model_in_width)
    {
        memcpy(seg_mask_real, seg_mask, ori_in_height * ori_in_width);
        return;
    }

    // 1️⃣ 裁掉 padding
    int cropped_index = 0;
    for (int i = 0; i < model_in_height; i++)
    {
        for (int j = 0; j < model_in_width; j++)
        {
            if (i >= y_pad && i < model_in_height - y_pad && j >= x_pad && j < model_in_width - x_pad)
            {
                int seg_index = i * model_in_width + j;
                cropped_seg[cropped_index] = seg_mask[seg_index];
                cropped_index++;
            }
        }
    }

    // 2️⃣ 构造 cv::Mat
    cv::Mat tmp(cropped_height, cropped_width, CV_8UC1, cropped_seg);
    cv::Mat dst(ori_in_height, ori_in_width, CV_8UC1);

    // 3️⃣ resize 到原图尺寸（非方形输入也能正确）
    cv::resize(tmp, dst, cv::Size(ori_in_width, ori_in_height), 0, 0, cv::INTER_NEAREST);

    // 4️⃣ 复制数据回 seg_mask_real
    memcpy(seg_mask_real, dst.data, ori_in_height * ori_in_width);
}

static int box_reverse(int position, int boundary, int pad, float scale)
{
    return (int)((clamp(position, 0, boundary) - pad) / scale);
}

static float sigmoid(float x) { return 1.0 / (1.0 + expf(-x)); }

static float unsigmoid(float y) { return -1.0 * logf((1.0 / y) - 1.0); }

inline static int32_t __clip(float val, float min, float max)
{
    float f = val <= min ? min : (val >= max ? max : val);
    return f;
}

static int8_t qnt_f32_to_affine(float f32, int32_t zp, float scale)
{
    float dst_val = (f32 / scale) + zp;
    int8_t res = (int8_t)__clip(dst_val, -128, 127);
    return res;
}

static float deqnt_affine_to_f32(int8_t qnt, int32_t zp, float scale) { return ((float)qnt - (float)zp) * scale; }

static void compute_dfl(float *tensor, int dfl_len, float *box)
{
    for (int b = 0; b < 4; b++)
    {
        float exp_t[dfl_len];
        float exp_sum = 0;
        float acc_sum = 0;
        for (int i = 0; i < dfl_len; i++)
        {
            exp_t[i] = exp(tensor[i + b * dfl_len]);
            exp_sum += exp_t[i];
        }

        for (int i = 0; i < dfl_len; i++)
        {
            acc_sum += exp_t[i] / exp_sum * i;
        }
        box[b] = acc_sum;
    }
}

static int process_i8(rknn_output *all_input, int input_id, int grid_h, int grid_w, int height, int width, int stride, int dfl_len,
                      std::vector<float> &boxes, std::vector<float> &segments, float *proto, std::vector<float> &objProbs, std::vector<int> &classId, float threshold,
                      rknn_app_context_t *app_ctx)
{
    int validCount = 0;
    int grid_len = grid_h * grid_w;

    // Skip if input_id is not 0, 4, 8, or 12
    if (input_id % 4 != 0)
    {
        return validCount;
    }

    if (input_id == 12)
    {
        int8_t *input_proto = (int8_t *)all_input[input_id].buf;
        int32_t zp_proto = app_ctx->output_attrs[input_id].zp;
        float scale_proto = app_ctx->output_attrs[input_id].scale;
        for (int i = 0; i < PROTO_CHANNEL * PROTO_HEIGHT * PROTO_WEIGHT; i++)
        {
            proto[i] = deqnt_affine_to_f32(input_proto[i], zp_proto, scale_proto);
        }
        return validCount;
    }

    int8_t *box_tensor = (int8_t *)all_input[input_id].buf;
    int32_t box_zp = app_ctx->output_attrs[input_id].zp;
    float box_scale = app_ctx->output_attrs[input_id].scale;

    int8_t *score_tensor = (int8_t *)all_input[input_id + 1].buf;
    int32_t score_zp = app_ctx->output_attrs[input_id + 1].zp;
    float score_scale = app_ctx->output_attrs[input_id + 1].scale;

    int8_t *score_sum_tensor = nullptr;
    int32_t score_sum_zp = 0;
    float score_sum_scale = 1.0;
    score_sum_tensor = (int8_t *)all_input[input_id + 2].buf;
    score_sum_zp = app_ctx->output_attrs[input_id + 2].zp;
    score_sum_scale = app_ctx->output_attrs[input_id + 2].scale;

    int8_t *seg_tensor = (int8_t *)all_input[input_id + 3].buf;
    int32_t seg_zp = app_ctx->output_attrs[input_id + 3].zp;
    float seg_scale = app_ctx->output_attrs[input_id + 3].scale;

    int8_t score_thres_i8 = qnt_f32_to_affine(threshold, score_zp, score_scale);
    int8_t score_sum_thres_i8 = qnt_f32_to_affine(threshold, score_sum_zp, score_sum_scale);

    for (int i = 0; i < grid_h; i++)
    {
        for (int j = 0; j < grid_w; j++)
        {
            int offset = i * grid_w + j;
            int max_class_id = -1;

            int offset_seg = i * grid_w + j;
            int8_t *in_ptr_seg = seg_tensor + offset_seg;

            // for quick filtering through "score sum"
            if (score_sum_tensor != nullptr)
            {
                if (score_sum_tensor[offset] < score_sum_thres_i8)
                {
                    continue;
                }
            }

            int8_t max_score = -score_zp;
            for (int c = 0; c < OBJ_CLASS_NUM; c++)
            {
                if ((score_tensor[offset] > score_thres_i8) && (score_tensor[offset] > max_score))
                {
                    max_score = score_tensor[offset];
                    max_class_id = c;
                }
                offset += grid_len;
            }

            // compute box
            if (max_score > score_thres_i8)
            {

                for (int k = 0; k < PROTO_CHANNEL; k++)
                {
                    float seg_element_fp = deqnt_affine_to_f32(in_ptr_seg[(k)*grid_len], seg_zp, seg_scale);
                    segments.push_back(seg_element_fp);
                }

                offset = i * grid_w + j;
                float box[4];
                float before_dfl[dfl_len * 4];
                for (int k = 0; k < dfl_len * 4; k++)
                {
                    before_dfl[k] = deqnt_affine_to_f32(box_tensor[offset], box_zp, box_scale);
                    offset += grid_len;
                }
                compute_dfl(before_dfl, dfl_len, box);

                float x1, y1, x2, y2, w, h;
                x1 = (-box[0] + j + 0.5) * stride;
                y1 = (-box[1] + i + 0.5) * stride;
                x2 = (box[2] + j + 0.5) * stride;
                y2 = (box[3] + i + 0.5) * stride;
                w = x2 - x1;
                h = y2 - y1;
                boxes.push_back(x1);
                boxes.push_back(y1);
                boxes.push_back(w);
                boxes.push_back(h);

                objProbs.push_back(deqnt_affine_to_f32(max_score, score_zp, score_scale));
                classId.push_back(max_class_id);
                validCount++;
            }
        }
    }
    return validCount;
}

static int process_fp32(rknn_output *all_input, int input_id, int grid_h, int grid_w, int height, int width, int stride, int dfl_len,
                        std::vector<float> &boxes, std::vector<float> &segments, float *proto, std::vector<float> &objProbs, std::vector<int> &classId, float threshold)
{
    int validCount = 0;
    int grid_len = grid_h * grid_w;

    // Skip if input_id is not 0, 4, 8, or 12
    if (input_id % 4 != 0)
    {
        return validCount;
    }

    if (input_id == 12)
    {
        float *input_proto = (float *)all_input[input_id].buf;
        for (int i = 0; i < PROTO_CHANNEL * PROTO_HEIGHT * PROTO_WEIGHT; i++)
        {
            proto[i] = input_proto[i];
        }
        return validCount;
    }

    float *box_tensor = (float *)all_input[input_id].buf;
    float *score_tensor = (float *)all_input[input_id + 1].buf;
    float *score_sum_tensor = (float *)all_input[input_id + 2].buf;
    float *seg_tensor = (float *)all_input[input_id + 3].buf;

    for (int i = 0; i < grid_h; i++)
    {
        for (int j = 0; j < grid_w; j++)
        {
            int offset = i * grid_w + j;
            int max_class_id = -1;

            int offset_seg = i * grid_w + j;
            float *in_ptr_seg = seg_tensor + offset_seg;

            // for quick filtering through "score sum"
            if (score_sum_tensor != nullptr)
            {
                if (score_sum_tensor[offset] < threshold)
                {
                    continue;
                }
            }

            float max_score = 0;
            for (int c = 0; c < OBJ_CLASS_NUM; c++)
            {
                if ((score_tensor[offset] > threshold) && (score_tensor[offset] > max_score))
                {
                    max_score = score_tensor[offset];
                    max_class_id = c;
                }
                offset += grid_len;
            }

            // compute box
            if (max_score > threshold)
            {

                for (int k = 0; k < PROTO_CHANNEL; k++)
                {
                    float seg_element_f32 = in_ptr_seg[(k)*grid_len];
                    segments.push_back(seg_element_f32);
                }

                offset = i * grid_w + j;
                float box[4];
                float before_dfl[dfl_len * 4];
                for (int k = 0; k < dfl_len * 4; k++)
                {
                    before_dfl[k] = box_tensor[offset];
                    offset += grid_len;
                }
                compute_dfl(before_dfl, dfl_len, box);

                float x1, y1, x2, y2, w, h;
                x1 = (-box[0] + j + 0.5) * stride;
                y1 = (-box[1] + i + 0.5) * stride;
                x2 = (box[2] + j + 0.5) * stride;
                y2 = (box[3] + i + 0.5) * stride;
                w = x2 - x1;
                h = y2 - y1;
                boxes.push_back(x1);
                boxes.push_back(y1);
                boxes.push_back(w);
                boxes.push_back(h);

                objProbs.push_back(max_score);
                classId.push_back(max_class_id);
                validCount++;
            }
        }
    }
    return validCount;
}

// int post_process(rknn_app_context_t *app_ctx, rknn_output *outputs, letterbox_t *letter_box, float conf_threshold, float nms_threshold, object_detect_result_list *od_results)
// {

//     std::vector<float> filterBoxes;
//     std::vector<float> objProbs;
//     std::vector<int> classId;

//     std::vector<float> filterSegments;
//     float proto[PROTO_CHANNEL * PROTO_HEIGHT * PROTO_WEIGHT];
//     std::vector<float> filterSegments_by_nms;

//     int model_in_width = app_ctx->model_width;
//     int model_in_height = app_ctx->model_height;

//     int validCount = 0;
//     int stride = 0;
//     int grid_h = 0;
//     int grid_w = 0;

//     memset(od_results, 0, sizeof(object_detect_result_list));

//     int dfl_len = app_ctx->output_attrs[0].dims[1] / 4;
//     int output_per_branch = app_ctx->io_num.n_output / 3; // default 3 branch

//     // process the outputs of rknn
//     for (int i = 0; i < 13; i++)
//     {
//         grid_h = app_ctx->output_attrs[i].dims[2];
//         grid_w = app_ctx->output_attrs[i].dims[3];
//         stride = model_in_height / grid_h;

//         if (app_ctx->is_quant)
//         {
//             validCount += process_i8(outputs, i, grid_h, grid_w, model_in_height, model_in_width, stride, dfl_len, filterBoxes, filterSegments, proto, objProbs,
//                                      classId, conf_threshold, app_ctx);
//         }
//         else
//         {
//             validCount += process_fp32(outputs, i, grid_h, grid_w, model_in_height, model_in_width, stride, dfl_len, filterBoxes, filterSegments, proto, objProbs,
//                                        classId, conf_threshold);
//         }
//     }

//     // nms
//     if (validCount <= 0)
//     {
//         return 0;
//     }
//     std::vector<int> indexArray;
//     for (int i = 0; i < validCount; ++i)
//     {
//         indexArray.push_back(i);
//     }

//     quick_sort_indice_inverse(objProbs, 0, validCount - 1, indexArray);

//     std::set<int> class_set(std::begin(classId), std::end(classId));

//     for (auto c : class_set)
//     {
//         nms(validCount, filterBoxes, classId, indexArray, c, nms_threshold);
//     }

//     int last_count = 0;
//     od_results->count = 0;

//     for (int i = 0; i < validCount; ++i)
//     {
//         if (indexArray[i] == -1 || last_count >= OBJ_NUMB_MAX_SIZE)
//         {
//             continue;
//         }
//         int n = indexArray[i];

//         float x1 = filterBoxes[n * 4 + 0];
//         float y1 = filterBoxes[n * 4 + 1];
//         float x2 = x1 + filterBoxes[n * 4 + 2];
//         float y2 = y1 + filterBoxes[n * 4 + 3];
//         int id = classId[n];
//         float obj_conf = objProbs[i];

//         for (int k = 0; k < PROTO_CHANNEL; k++)
//         {
//             filterSegments_by_nms.push_back(filterSegments[n * PROTO_CHANNEL + k]);
//         }

//         od_results->results[last_count].box.left = x1;
//         od_results->results[last_count].box.top = y1;
//         od_results->results[last_count].box.right = x2;
//         od_results->results[last_count].box.bottom = y2;

//         od_results->results[last_count].prop = obj_conf;
//         od_results->results[last_count].cls_id = id;
//         last_count++;
//     }
//     od_results->count = last_count;
//     int boxes_num = od_results->count;

//     float filterBoxes_by_nms[boxes_num * 4];
//     int cls_id[boxes_num];
//     for (int i = 0; i < boxes_num; i++)
//     {
//         // for crop_mask
//         filterBoxes_by_nms[i * 4 + 0] = od_results->results[i].box.left;   // x1;
//         filterBoxes_by_nms[i * 4 + 1] = od_results->results[i].box.top;    // y1;
//         filterBoxes_by_nms[i * 4 + 2] = od_results->results[i].box.right;  // x2;
//         filterBoxes_by_nms[i * 4 + 3] = od_results->results[i].box.bottom; // y2;
//         cls_id[i] = od_results->results[i].cls_id;

//         // get real box
//         od_results->results[i].box.left = box_reverse(od_results->results[i].box.left, model_in_width, letter_box->x_pad, letter_box->scale);
//         od_results->results[i].box.top = box_reverse(od_results->results[i].box.top, model_in_height, letter_box->y_pad, letter_box->scale);
//         od_results->results[i].box.right = box_reverse(od_results->results[i].box.right, model_in_width, letter_box->x_pad, letter_box->scale);
//         od_results->results[i].box.bottom = box_reverse(od_results->results[i].box.bottom, model_in_height, letter_box->y_pad, letter_box->scale);
//     }

//     TIMER timer;
// #ifdef USE_FP_RESIZE
//     timer.tik();
//     // compute the mask through Matmul
//     int ROWS_A = boxes_num;
//     int COLS_A = PROTO_CHANNEL;
//     int COLS_B = PROTO_HEIGHT * PROTO_WEIGHT;
//     float *matmul_out = (float *)malloc(boxes_num * PROTO_HEIGHT * PROTO_WEIGHT * sizeof(float));
//     matmul_by_cpu_fp(filterSegments_by_nms, proto, matmul_out, ROWS_A, COLS_A, COLS_B);
//     // matmul_by_npu_fp(filterSegments_by_nms, proto, matmul_out, ROWS_A, COLS_A, COLS_B, app_ctx);
//     timer.tok();
//     timer.print_time("matmul_by_cpu_fp");

//     timer.tik();
//     // resize to (boxes_num, model_in_width, model_in_height)
//     float *seg_mask = (float *)malloc(boxes_num * model_in_height * model_in_width * sizeof(float));
//     resize_by_opencv_fp(matmul_out, PROTO_WEIGHT, PROTO_HEIGHT, boxes_num, seg_mask, model_in_width, model_in_height);
//     timer.tok();
//     timer.print_time("resize_by_opencv_fp");

//     timer.tik();
//     // crop mask
//     uint8_t *all_mask_in_one = (uint8_t *)malloc(model_in_height * model_in_width * sizeof(uint8_t));
//     memset(all_mask_in_one, 0, model_in_height * model_in_width * sizeof(uint8_t));
//     crop_mask_fp(seg_mask, all_mask_in_one, filterBoxes_by_nms, boxes_num, cls_id, model_in_height, model_in_width);
//     timer.tok();
//     timer.print_time("crop_mask_fp");
// #else
//     timer.tik();
//     // compute the mask through Matmul
//     int ROWS_A = boxes_num;
//     int COLS_A = PROTO_CHANNEL;
//     int COLS_B = PROTO_HEIGHT * PROTO_WEIGHT;
//     uint8_t *matmul_out = (uint8_t *)malloc(boxes_num * PROTO_HEIGHT * PROTO_WEIGHT * sizeof(uint8_t));
//     matmul_by_cpu_uint8(filterSegments_by_nms, proto, matmul_out, ROWS_A, COLS_A, COLS_B);

//     timer.tok();
//     timer.print_time("matmul_by_cpu_uint8");

//     timer.tik();
//     uint8_t *seg_mask = (uint8_t *)malloc(boxes_num * model_in_height * model_in_width * sizeof(uint8_t));
//     resize_by_opencv_uint8(matmul_out, PROTO_WEIGHT, PROTO_HEIGHT, boxes_num, seg_mask, model_in_width, model_in_height);
//     timer.tok();
//     timer.print_time("resize_by_opencv_uint8");

//     timer.tik();
//     // crop mask
//     uint8_t *all_mask_in_one = (uint8_t *)malloc(model_in_height * model_in_width * sizeof(uint8_t));
//     memset(all_mask_in_one, 0, model_in_height * model_in_width * sizeof(uint8_t));
//     crop_mask_uint8(seg_mask, all_mask_in_one, filterBoxes_by_nms, boxes_num, cls_id, model_in_height, model_in_width);
//     timer.tok();
//     timer.print_time("crop_mask_uint8");
// #endif

//     timer.tik();
//     // get real mask
//     int cropped_height = model_in_height - letter_box->y_pad * 2;
//     int cropped_width = model_in_width - letter_box->x_pad * 2;
//     int ori_in_height = app_ctx->input_image_height;
//     int ori_in_width = app_ctx->input_image_width;
//     int y_pad = letter_box->y_pad;
//     int x_pad = letter_box->x_pad;
//     uint8_t *cropped_seg_mask = (uint8_t *)malloc(cropped_height * cropped_width * sizeof(uint8_t));
//     uint8_t *real_seg_mask = (uint8_t *)malloc(ori_in_height * ori_in_width * sizeof(uint8_t));
//     seg_reverse(all_mask_in_one, cropped_seg_mask, real_seg_mask,
//                 model_in_height, model_in_width, cropped_height, cropped_width, ori_in_height, ori_in_width, y_pad, x_pad);

//     od_results->results_seg[0].seg_mask = real_seg_mask;

//     free(all_mask_in_one);
//     free(cropped_seg_mask);
//     free(seg_mask);
//     free(matmul_out);
//     timer.tok();
//     timer.print_time("seg_reverse");

//     return 0;
// }

/* 后处理流程说明：
模型输出
   ↓
process_xxx
   ↓
候选框 + mask系数 + proto
   ↓
NMS
   ↓
最终框 + mask系数
   ↓
matmul（生成160×160 mask）
   ↓
resize（放大到模型尺寸）
   ↓
crop（按框裁剪）
   ↓
seg_reverse（去padding + 映射原图）
   ↓
最终mask
*/

//  后处理函数 post_process 主要负责将模型的原始输出转换为最终的检测结果，包括以下几个步骤：

// 固定PROTO_HEIGHT * PROTO_WEIGHT，的后处理函数
/*
int post_process(rknn_app_context_t *app_ctx, rknn_output *outputs, letterbox_t *letter_box,
                 float conf_threshold, float nms_threshold, object_detect_result_list *od_results)
{
    std::vector<float> filterBoxes;
    std::vector<float> objProbs;
    std::vector<int> classId;

    std::vector<float> filterSegments;
    // float proto[PROTO_CHANNEL * PROTO_HEIGHT * PROTO_WEIGHT];
    int proto_size = PROTO_CHANNEL * PROTO_HEIGHT * PROTO_WEIGHT;
    float *proto = (float *)malloc(proto_size * sizeof(float));
    if (!proto)
    {
        printf("malloc proto failed\n");
        return -1;
    }
    memset(proto, 0, proto_size * sizeof(float));

    std::vector<float> filterSegments_by_nms;

    int model_in_width = app_ctx->model_width;
    int model_in_height = app_ctx->model_height;

    int validCount = 0;
    int stride = 0;
    int grid_h = 0;
    int grid_w = 0;

    memset(od_results, 0, sizeof(object_detect_result_list));

    int dfl_len = app_ctx->output_attrs[0].dims[1] / 4;

    // ---------------------------
    // 1️⃣ 解析模型输出，生成候选框
    // ---------------------------
    for (int i = 0; i < 13; i++)
    {
        grid_h = app_ctx->output_attrs[i].dims[2];
        grid_w = app_ctx->output_attrs[i].dims[3];
        stride = model_in_height / grid_h;

        if (app_ctx->is_quant)
        {
            validCount += process_i8(outputs, i, grid_h, grid_w, model_in_height, model_in_width,
                                     stride, dfl_len, filterBoxes, filterSegments, proto,
                                     objProbs, classId, conf_threshold, app_ctx);
        }
        else
        {
            validCount += process_fp32(outputs, i, grid_h, grid_w, model_in_height, model_in_width,
                                       stride, dfl_len, filterBoxes, filterSegments, proto,
                                       objProbs, classId, conf_threshold);
        }
    }

    if (validCount <= 0)
        return 0;

    // ---------------------------
    // 2️⃣ NMS 筛选
    // ---------------------------
    std::vector<int> indexArray(validCount);
    for (int i = 0; i < validCount; i++)
        indexArray[i] = i;

    quick_sort_indice_inverse(objProbs, 0, validCount - 1, indexArray);

    std::set<int> class_set(std::begin(classId), std::end(classId));
    for (auto c : class_set)
        nms(validCount, filterBoxes, classId, indexArray, c, nms_threshold);

    int last_count = 0;
    od_results->count = 0;

    for (int i = 0; i < validCount; ++i)
    {
        if (indexArray[i] == -1 || last_count >= OBJ_NUMB_MAX_SIZE)
            continue;

        int n = indexArray[i];

        float x1 = filterBoxes[n * 4 + 0];
        float y1 = filterBoxes[n * 4 + 1];
        float x2 = x1 + filterBoxes[n * 4 + 2];
        float y2 = y1 + filterBoxes[n * 4 + 3];
        int id = classId[n];
        float obj_conf = objProbs[i];

        for (int k = 0; k < PROTO_CHANNEL; k++)
            filterSegments_by_nms.push_back(filterSegments[n * PROTO_CHANNEL + k]);

        od_results->results[last_count].box.left = x1;
        od_results->results[last_count].box.top = y1;
        od_results->results[last_count].box.right = x2;
        od_results->results[last_count].box.bottom = y2;
        od_results->results[last_count].prop = obj_conf;
        od_results->results[last_count].cls_id = id;

        last_count++;
    }

    od_results->count = last_count;
    int boxes_num = od_results->count;

    // ---------------------------
    // 3️⃣ 准备裁剪 mask
    // ---------------------------
    float filterBoxes_by_nms[boxes_num * 4];
    int cls_id[boxes_num];
    for (int i = 0; i < boxes_num; i++)
    {
        filterBoxes_by_nms[i * 4 + 0] = od_results->results[i].box.left;
        filterBoxes_by_nms[i * 4 + 1] = od_results->results[i].box.top;
        filterBoxes_by_nms[i * 4 + 2] = od_results->results[i].box.right;
        filterBoxes_by_nms[i * 4 + 3] = od_results->results[i].box.bottom;
        cls_id[i] = od_results->results[i].cls_id;

        // 还原原图坐标
        od_results->results[i].box.left = box_reverse(od_results->results[i].box.left, model_in_width, letter_box->x_pad, letter_box->scale);
        od_results->results[i].box.top = box_reverse(od_results->results[i].box.top, model_in_height, letter_box->y_pad, letter_box->scale);
        od_results->results[i].box.right = box_reverse(od_results->results[i].box.right, model_in_width, letter_box->x_pad, letter_box->scale);
        od_results->results[i].box.bottom = box_reverse(od_results->results[i].box.bottom, model_in_height, letter_box->y_pad, letter_box->scale);
    }

    TIMER timer;
    uint8_t *seg_mask = (uint8_t *)malloc(boxes_num * model_in_height * model_in_width * sizeof(uint8_t));

    // ---------------------------
    // matmul + resize
    // ---------------------------
#ifdef USE_FP_RESIZE
    float *matmul_out = (float *)malloc(boxes_num * PROTO_HEIGHT * PROTO_WEIGHT * sizeof(float));
    matmul_by_cpu_fp(filterSegments_by_nms, proto, matmul_out, boxes_num, PROTO_CHANNEL, PROTO_HEIGHT * PROTO_WEIGHT);
    resize_by_opencv_fp(matmul_out, PROTO_WEIGHT, PROTO_HEIGHT, boxes_num, seg_mask, model_in_width, model_in_height);
    free(matmul_out);
#else
    uint8_t *matmul_out = (uint8_t *)malloc(boxes_num * PROTO_HEIGHT * PROTO_WEIGHT * sizeof(uint8_t));
    matmul_by_cpu_uint8(filterSegments_by_nms, proto, matmul_out, boxes_num, PROTO_CHANNEL, PROTO_HEIGHT * PROTO_WEIGHT);
    resize_by_opencv_uint8(matmul_out, PROTO_WEIGHT, PROTO_HEIGHT, boxes_num, seg_mask, model_in_width, model_in_height);


    free(matmul_out);
#endif

    // // ---------------------------
    // // 4️⃣ 按目标裁剪 mask 并存储
    // // ---------------------------
    // for (int i = 0; i < boxes_num; i++)
    // {
    //     uint8_t *single_mask = (uint8_t *)malloc(model_in_height * model_in_width * sizeof(uint8_t));
    //     memset(single_mask, 0, model_in_height * model_in_width * sizeof(uint8_t));

    //     #ifdef USE_FP_RESIZE
    //             crop_mask_fp(seg_mask, single_mask, &filterBoxes_by_nms[i * 4], 1, &cls_id[i], model_in_height, model_in_width);
    //     #else
    //             crop_mask_uint8(seg_mask, single_mask, &filterBoxes_by_nms[i * 4], 1, &cls_id[i], model_in_height, model_in_width);
    //     #endif

    //     // 得到原图尺寸 mask
    //     int cropped_height = model_in_height - letter_box->y_pad * 2;
    //     int cropped_width = model_in_width - letter_box->x_pad * 2;
    //     int ori_in_height = app_ctx->input_image_height;
    //     int ori_in_width = app_ctx->input_image_width;
    //     uint8_t *cropped_seg_mask = (uint8_t *)malloc(cropped_height * cropped_width * sizeof(uint8_t));
    //     uint8_t *real_seg_mask = (uint8_t *)malloc(ori_in_height * ori_in_width * sizeof(uint8_t));

    //     seg_reverse(single_mask, cropped_seg_mask, real_seg_mask,
    //                 model_in_height, model_in_width,
    //                 cropped_height, cropped_width,
    //                 ori_in_height, ori_in_width,
    //                 letter_box->y_pad, letter_box->x_pad);

    //     od_results->results_seg[i].seg_mask = real_seg_mask;

    //     free(single_mask);
    //     free(cropped_seg_mask);
    // }


    for (int i = 0; i < app_ctx->io_num.n_output; i++)
    {
        printf("[OUT %d] dims = [", i);

        for (int j = 0; j < app_ctx->output_attrs[i].n_dims; j++)
        {
            printf("%d", app_ctx->output_attrs[i].dims[j]);
            if (j != app_ctx->output_attrs[i].n_dims - 1)
                printf(", ");
        }

        printf("]\n");
    }

    // ---------------------------
    // 4️⃣ 按目标裁剪 mask 并存储（带调试）
    // ---------------------------
    for (int i = 0; i < boxes_num; i++)
    {
        printf("\n========== [MASK DEBUG] object %d ==========\n", i);

        // 👉 打印 box（模型坐标）
        printf("model box: [%.2f, %.2f, %.2f, %.2f]\n",
            filterBoxes_by_nms[i * 4 + 0],
            filterBoxes_by_nms[i * 4 + 1],
            filterBoxes_by_nms[i * 4 + 2],
            filterBoxes_by_nms[i * 4 + 3]);

        // 👉 打印原图 box
        printf("orig box:  [%d, %d, %d, %d]\n",
            od_results->results[i].box.left,
            od_results->results[i].box.top,
            od_results->results[i].box.right,
            od_results->results[i].box.bottom);

        uint8_t *single_mask = (uint8_t *)malloc(model_in_height * model_in_width);
        memset(single_mask, 0, model_in_height * model_in_width);


    #ifdef USE_FP_RESIZE
        crop_mask_fp(seg_mask, single_mask,
                    &filterBoxes_by_nms[i * 4],
                    1, &cls_id[i],
                    model_in_height, model_in_width);
    #else
        crop_mask_uint8(seg_mask, single_mask,
                        &filterBoxes_by_nms[i * 4],
                        1, &cls_id[i],
                        model_in_height, model_in_width);
    #endif

        // 👉 统计 mask 非零区域（判断有没有裁对）
        int non_zero = 0;
        for (int p = 0; p < model_in_height * model_in_width; p++)
        {
            if (single_mask[p] > 0)
                non_zero++;
        }
        printf("single_mask non-zero pixels: %d\n", non_zero);

        // ---------------------------
        // letterbox 参数打印
        // ---------------------------
        printf("letterbox: x_pad=%d, y_pad=%d, scale=%.4f\n",
            letter_box->x_pad,
            letter_box->y_pad,
            letter_box->scale);

        int cropped_height = model_in_height - letter_box->y_pad * 2;
        int cropped_width  = model_in_width  - letter_box->x_pad * 2;

        printf("cropped size: %d x %d\n", cropped_width, cropped_height);

        int ori_in_height = app_ctx->input_image_height;
        int ori_in_width  = app_ctx->input_image_width;

        printf("original image size: %d x %d\n", ori_in_width, ori_in_height);

        uint8_t *cropped_seg_mask = (uint8_t *)malloc(cropped_height * cropped_width);
        uint8_t *real_seg_mask    = (uint8_t *)malloc(ori_in_height * ori_in_width);

        // ---------------------------
        // seg_reverse
        // ---------------------------
        seg_reverse(single_mask, cropped_seg_mask, real_seg_mask,
                    model_in_height, model_in_width,
                    cropped_height, cropped_width,
                    ori_in_height, ori_in_width,
                    letter_box->y_pad, letter_box->x_pad);

        // 👉 检查最终 mask
        int final_non_zero = 0;
        for (int p = 0; p < ori_in_height * ori_in_width; p++)
        {
            if (real_seg_mask[p] > 0)
                final_non_zero++;
        }
        printf("final_mask non-zero pixels: %d\n", final_non_zero);

        // 👉 打印 mask 边界（粗略检测是否偏移）
        int min_x = ori_in_width, min_y = ori_in_height;
        int max_x = 0, max_y = 0;

        for (int y = 0; y < ori_in_height; y++)
        {
            for (int x = 0; x < ori_in_width; x++)
            {
                if (real_seg_mask[y * ori_in_width + x] > 0)
                {
                    if (x < min_x) min_x = x;
                    if (y < min_y) min_y = y;
                    if (x > max_x) max_x = x;
                    if (y > max_y) max_y = y;
                }
            }
        }

        printf("mask bbox in original image: [%d, %d, %d, %d]\n",
            min_x, min_y, max_x, max_y);

        od_results->results_seg[i].seg_mask = real_seg_mask;

        free(single_mask);
        free(cropped_seg_mask);
    }

    free(seg_mask);
    timer.tok();
    timer.print_time("seg_reverse");

    free(proto);

    return 0;
}
*/

int post_process(rknn_app_context_t *app_ctx,
                 rknn_output *outputs,
                 letterbox_t *letter_box,
                 float conf_threshold,
                 float nms_threshold,
                 object_detect_result_list *od_results)
{
    // ---------------------------
    // 静态内存池（避免反复 malloc）
    // ---------------------------
    static float *proto = NULL;
    static uint8_t *seg_mask = NULL;
    static uint8_t *single_mask = NULL;
    static uint8_t *cropped_seg_mask = NULL;

    int model_in_width = app_ctx->model_width;
    int model_in_height = app_ctx->model_height;

    int proto_size = PROTO_CHANNEL * PROTO_HEIGHT * PROTO_WEIGHT;
    int max_mask_size = model_in_height * model_in_width;

    // 初始化一次
    if (!proto)
    {
        proto = (float *)malloc(proto_size * sizeof(float));
        seg_mask = (uint8_t *)malloc(OBJ_NUMB_MAX_SIZE * max_mask_size);
        single_mask = (uint8_t *)malloc(max_mask_size);
        cropped_seg_mask = (uint8_t *)malloc(max_mask_size);

        if (!proto || !seg_mask || !single_mask || !cropped_seg_mask)
        {
            printf("malloc failed\n");
            return -1;
        }
    }

    std::vector<float> filterBoxes;
    std::vector<float> objProbs;
    std::vector<int> classId;
    std::vector<float> filterSegments;
    std::vector<float> filterSegments_by_nms;

    memset(od_results, 0, sizeof(object_detect_result_list));
    memset(proto, 0, proto_size * sizeof(float));

    int validCount = 0;
    int dfl_len = app_ctx->output_attrs[0].dims[1] / 4;

    // ---------------------------
    // 1️⃣ 解析输出
    // ---------------------------
    for (int i = 0; i < 13; i++)
    {
        int grid_h = app_ctx->output_attrs[i].dims[2];
        int grid_w = app_ctx->output_attrs[i].dims[3];
        int stride = model_in_height / grid_h;

        if (app_ctx->is_quant)
        {
            validCount += process_i8(outputs, i, grid_h, grid_w,
                                     model_in_height, model_in_width,
                                     stride, dfl_len,
                                     filterBoxes, filterSegments, proto,
                                     objProbs, classId,
                                     conf_threshold, app_ctx);
        }
        else
        {
            validCount += process_fp32(outputs, i, grid_h, grid_w,
                                       model_in_height, model_in_width,
                                       stride, dfl_len,
                                       filterBoxes, filterSegments, proto,
                                       objProbs, classId,
                                       conf_threshold);
        }
    }

    if (validCount <= 0)
        return 0;

    // ---------------------------
    // 2️⃣ NMS
    // ---------------------------
    std::vector<int> indexArray(validCount);
    for (int i = 0; i < validCount; i++)
        indexArray[i] = i;

    quick_sort_indice_inverse(objProbs, 0, validCount - 1, indexArray);

    std::set<int> class_set(std::begin(classId), std::end(classId));
    for (auto c : class_set)
        nms(validCount, filterBoxes, classId, indexArray, c, nms_threshold);

    int last_count = 0;

    std::vector<float> filterBoxes_by_nms;
    std::vector<int> cls_id;

    for (int i = 0; i < validCount; ++i)
    {
        if (indexArray[i] == -1 || last_count >= OBJ_NUMB_MAX_SIZE)
            continue;

        int n = indexArray[i];

        float x1 = filterBoxes[n * 4 + 0];
        float y1 = filterBoxes[n * 4 + 1];
        float x2 = x1 + filterBoxes[n * 4 + 2];
        float y2 = y1 + filterBoxes[n * 4 + 3];

        filterBoxes_by_nms.push_back(x1);
        filterBoxes_by_nms.push_back(y1);
        filterBoxes_by_nms.push_back(x2);
        filterBoxes_by_nms.push_back(y2);

        cls_id.push_back(classId[n]);

        for (int k = 0; k < PROTO_CHANNEL; k++)
            filterSegments_by_nms.push_back(filterSegments[n * PROTO_CHANNEL + k]);

        od_results->results[last_count].box.left = x1;
        od_results->results[last_count].box.top = y1;
        od_results->results[last_count].box.right = x2;
        od_results->results[last_count].box.bottom = y2;
        od_results->results[last_count].prop = objProbs[i];
        od_results->results[last_count].cls_id = classId[n];

        // 还原原图坐标
        od_results->results[last_count].box.left = box_reverse(od_results->results[last_count].box.left, model_in_width, letter_box->x_pad, letter_box->scale);
        od_results->results[last_count].box.top = box_reverse(od_results->results[last_count].box.top, model_in_height, letter_box->y_pad, letter_box->scale);
        od_results->results[last_count].box.right = box_reverse(od_results->results[last_count].box.right, model_in_width, letter_box->x_pad, letter_box->scale);
        od_results->results[last_count].box.bottom = box_reverse(od_results->results[last_count].box.bottom, model_in_height, letter_box->y_pad, letter_box->scale);

        last_count++;
    }

    od_results->count = last_count;

    int boxes_num = std::min(last_count, 5); // 👉 限制数量（防OOM）

    if (boxes_num <= 0)
        return 0;

    // ---------------------------
    // 3️⃣ matmul + resize
    // ---------------------------
    uint8_t *matmul_out = (uint8_t *)malloc(boxes_num * PROTO_HEIGHT * PROTO_WEIGHT);

    matmul_by_cpu_uint8(filterSegments_by_nms,
                        proto,
                        matmul_out,
                        boxes_num,
                        PROTO_CHANNEL,
                        PROTO_HEIGHT * PROTO_WEIGHT);

    resize_by_opencv_uint8(matmul_out,
                           PROTO_WEIGHT,
                           PROTO_HEIGHT,
                           boxes_num,
                           seg_mask,
                           model_in_width,
                           model_in_height);

    free(matmul_out);

    // ---------------------------
    // 4️⃣ mask处理（关键修复点）
    // ---------------------------
    for (int i = 0; i < boxes_num; i++)
    {
        int ori_size = app_ctx->input_image_width * app_ctx->input_image_height;

        // 👉 分配输出mask（避免段错误）
        if (!od_results->results_seg[i].seg_mask)
        {
            od_results->results_seg[i].seg_mask =
                (uint8_t *)malloc(ori_size);
        }

        memset(single_mask, 0, max_mask_size);

        crop_mask_uint8(seg_mask,
                        single_mask,
                        &filterBoxes_by_nms[i * 4],
                        1,
                        &cls_id[i],
                        model_in_height,
                        model_in_width);

        int cropped_height = model_in_height - letter_box->y_pad * 2;
        int cropped_width = model_in_width - letter_box->x_pad * 2;

        seg_reverse(single_mask,
                    cropped_seg_mask,
                    od_results->results_seg[i].seg_mask,
                    model_in_height,
                    model_in_width,
                    cropped_height,
                    cropped_width,
                    app_ctx->input_image_height,
                    app_ctx->input_image_width,
                    letter_box->y_pad,
                    letter_box->x_pad);
    }

    return 0;
}

int init_post_process()
{
    int ret = 0;
    ret = loadLabelName(LABEL_NALE_TXT_PATH, labels);
    if (ret < 0)
    {
        printf("Load %s failed!\n", LABEL_NALE_TXT_PATH);
        return -1;
    }
    return 0;
}

char *coco_cls_to_name(int cls_id)
{

    if (cls_id >= OBJ_CLASS_NUM)
    {
        return (char *)"null";
    }

    if (labels[cls_id])
    {
        return labels[cls_id];
    }

    return (char *)"null";
}

void deinit_post_process()
{
    for (int i = 0; i < OBJ_CLASS_NUM; i++)
    {
        {
            free(labels[i]);
            labels[i] = nullptr;
        }
    }
}

// void extract_seg_mask_contours(object_detect_result_list &od_results,
//                                int target_index,
//                                int width, int height,
//                                std::vector<std::vector<cv::Point>> &out_contours)
// {
//     if (od_results.count <= target_index)
//         return;

//     uint8_t *seg_mask = od_results.results_seg[target_index].seg_mask;
//     if (!seg_mask)
//         return;

//     // -----------------------------
//     // 1️⃣ seg_mask → cv::Mat
//     // -----------------------------
//     cv::Mat mask(height, width, CV_8UC1, seg_mask);

//     // 二值化（前景 = 非零）
//     cv::Mat bin_mask;
//     cv::threshold(mask, bin_mask, 0, 255, cv::THRESH_BINARY);

//     // -----------------------------
//     // 2️⃣ 提取轮廓
//     // -----------------------------
//     std::vector<std::vector<cv::Point>> contours;
//     std::vector<cv::Vec4i> hierarchy;
//     cv::findContours(
//         bin_mask,
//         contours,
//         hierarchy,
//         cv::RETR_EXTERNAL,      // 只提取最外层闭合轮廓
//         cv::CHAIN_APPROX_SIMPLE // 压缩连续点
//     );

//     // -----------------------------
//     // 3️⃣ 可选轮廓简化
//     // -----------------------------
//     out_contours.clear();
//     for (const auto &cnt : contours)
//     {
//         std::vector<cv::Point> approx;
//         cv::approxPolyDP(cnt, approx, 2.0, true); // 像素精度 2
//         out_contours.push_back(approx);
//     }

//     // -----------------------------
//     // 4️⃣ 释放 mask
//     // -----------------------------
//     free(seg_mask);
//     od_results.results_seg[target_index].seg_mask = nullptr;
// }

void extract_seg_mask_contours(
    object_segment_result *seg, // 只操作单个目标的 seg_mask
    int width,
    int height,
    std::vector<std::vector<cv::Point>> &out_contours)
{
    if (!seg || !seg->seg_mask)
        return;

    // -----------------------------
    // 1️⃣ seg_mask → cv::Mat
    // -----------------------------
    cv::Mat mask(height, width, CV_8UC1, seg->seg_mask);

    // 二值化（前景 = 非零）
    cv::Mat bin_mask;
    cv::threshold(mask, bin_mask, 0, 255, cv::THRESH_BINARY);

    // -----------------------------
    // 2️⃣ 提取轮廓
    // -----------------------------
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(
        bin_mask,
        contours,
        hierarchy,
        cv::RETR_EXTERNAL,      // 只提取最外层闭合轮廓
        cv::CHAIN_APPROX_SIMPLE // 压缩连续点
    );

    // -----------------------------
    // 3️⃣ 可选轮廓简化
    // -----------------------------
    out_contours.clear();
    for (const auto &cnt : contours)
    {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(cnt, approx, 2.0, true); // 像素精度 2
        out_contours.push_back(approx);
    }

    // 统计总点数并打印
    // -----------------------------
    int total_points = 0;
    for (const auto &cnt : out_contours)
    {
        total_points += static_cast<int>(cnt.size());
    }
    printf("extract_seg_mask_contours: total contours = %zu, total points = %d\n",
           out_contours.size(), total_points);

    // -----------------------------
    // 4️⃣ 释放 mask
    // -----------------------------
    free(seg->seg_mask);
    seg->seg_mask = nullptr;
}

// 对mark区域边缘，做移动平滑，减弱mark检测边缘的锯齿型抖动。
void smoothContour(
    const std::vector<cv::Point> &input,
    std::vector<cv::Point> &output,
    int win)
{
    output.clear();
    int n = input.size();
    if (n < win)
    {
        output = input;
        return;
    }

    for (int i = 0; i < n; ++i)
    {
        int count = 0;
        float sx = 0, sy = 0;
        for (int k = -win / 2; k <= win / 2; ++k)
        {
            int idx = (i + k + n) % n; // 闭合轮廓
            sx += input[idx].x;
            sy += input[idx].y;
            count++;
        }
        output.emplace_back(
            static_cast<int>(sx / count),
            static_cast<int>(sy / count));
    }
}

inline float estimateDistance(float x, ConfigInfo &config)
{
    float polyfit_result;
    polyfit_result = config.camera_z_axle_top_resize_rate * x;
    polyfit_result = config.camera_z_axle_polyfit_w0 * x * x + config.camera_z_axle_polyfit_w1 * x + config.camera_z_axle_polyfit_w2;
    // y = 0.2220 x^2 + 0.8531 x + 0.1568
    return polyfit_result;
}

// =========================
// 将 object_detect_result 填充到 ObjectCameraDetectResult
// =========================
void fillCameraDetectResult(
    const object_detect_result *det,
    ObjectCameraDetectResult &one,
    ConfigInfo &config)
{
    // printf("enter Filling camera detect result for one object...\n");
    // printf("edge ptr = %p, num = %d\n",
    //    det->camera_coordinates.add_edge_point_single_pixel_camera_coordinates,
    //    det->camera_coordinates.add_edge_point_num);

    // ---------- 1. 类别 & 置信度 ----------
    one.prop = det->prop;
    one.cls_id = config.BASE_AREA;

    // printf("det->box.top = %d\n", det->box.top);
    // printf("det->box.bottom = %d\n", det->box.bottom);
    // printf("det->box.left = %d\n", det->box.left);
    // printf("det->box.right = %d\n", det->box.right);
    // ---------- 2. 四个角点 ----------
    one.coords.clear();
    one.coords.resize(4);

    one.coords[0].X = det->camera_coordinates.left_top.X;
    one.coords[0].Y = det->camera_coordinates.left_top.Y;
    one.coords[0].Z = estimateDistance(det->camera_coordinates.left_top.Z, config);

    one.coords[1].X = det->camera_coordinates.right_top.X;
    one.coords[1].Y = det->camera_coordinates.right_top.Y;
    one.coords[1].Z = estimateDistance(det->camera_coordinates.right_top.Z, config);

    one.coords[2].X = det->camera_coordinates.right_bottom.X;
    one.coords[2].Y = det->camera_coordinates.right_bottom.Y;
    one.coords[2].Z = estimateDistance(det->camera_coordinates.right_bottom.Z, config);

    one.coords[3].X = det->camera_coordinates.left_bottom.X;
    one.coords[3].Y = det->camera_coordinates.left_bottom.Y;
    one.coords[3].Z = estimateDistance(det->camera_coordinates.left_bottom.Z, config);

    // ---------- 3. 目标框 ----------
    one.target_box.top = det->box.top;
    one.target_box.bottom = det->box.bottom;
    one.target_box.left = det->box.left;
    one.target_box.right = det->box.right;

    // printf("det->box.top = %d\n", det->box.top);
    // printf("det->box.bottom = %d\n", det->box.bottom);
    // printf("det->box.left = %d\n", det->box.left);
    // printf("det->box.right = %d\n", det->box.right);

    // printf("Target Box: left=%d, top=%d, right=%d, bottom=%d\n",
    //        one.target_box.left,
    //        one.target_box.top,
    //        one.target_box.right,
    //        one.target_box.bottom);

    // ---------- 4. 边缘采样点（按真实数量） ----------
    one.add_edge_point_single_pixel_camera_coordinates.clear();

    const int edge_num = det->camera_coordinates.add_edge_point_num;
    const auto *edge_src =
        det->camera_coordinates.add_edge_point_single_pixel_camera_coordinates;

    if (edge_num <= 0 || edge_src == nullptr)
    {
        printf("No edge points for this object (edge_num = %d)\n", edge_num);
        return;
    }

    one.add_edge_point_single_pixel_camera_coordinates.resize(edge_num);

    for (int i = 0; i < edge_num; ++i)
    {
        const single_pixel_camera_coordinates &src = edge_src[i];

        one.add_edge_point_single_pixel_camera_coordinates[i].X = src.X;
        one.add_edge_point_single_pixel_camera_coordinates[i].Y = src.Y;
        one.add_edge_point_single_pixel_camera_coordinates[i].Z =
            estimateDistance(src.Z, config);
    }
    printf("edge_num = %d\n", edge_num);
}

/*
void filter_mask_contours(
    const std::vector<std::vector<cv::Point>> &input_contours, // 输入：原始轮廓集合（来自mask）
    std::vector<std::vector<cv::Point>> &output_contours)      // 输出：过滤+平滑后的轮廓
{
    output_contours.clear(); // 先清空输出

    // 遍历每一个轮廓
    for (const auto &cnt : input_contours)
    {
        // -----------------------------
        // 1️⃣ 面积过滤（最关键）
        // -----------------------------
        // 作用：去除小噪点 / 小碎片（YOLOv8 seg常见问题）
        double area = cv::contourArea(cnt);
        if (area < 200)   // ⚠️ 可调参数：建议根据分辨率调 (100~1000)
            continue;

        // -----------------------------
        // 2️⃣ 外接矩形 + 长宽比过滤
        // -----------------------------
        // 获取最小外接矩形
        cv::Rect rect = cv::boundingRect(cnt);

        // 长宽比 = 宽 / 高
        float aspect_ratio = (float)rect.width / rect.height;

        // 作用：过滤异常细长的区域（如边缘噪声、电线误检等）
        if (aspect_ratio > 10.0 || aspect_ratio < 0.1)
            continue;

        // -----------------------------
        // 3️⃣ 周长过滤
        // -----------------------------
        // 计算轮廓周长（闭合）
        double perimeter = cv::arcLength(cnt, true);

        // 作用：进一步过滤过小或不规则轮廓
        if (perimeter < 50)   // ⚠️ 可调参数
            continue;

        // -----------------------------
        // 4️⃣ 填充率过滤（非常重要）
        // -----------------------------
        // 填充率 = 实际面积 / 外接矩形面积
        float fill_ratio = area / (rect.width * rect.height + 1e-5);

        // 作用：
        // - 过滤“很空”的区域（例如细线、噪声、破碎mask）
        // - 对扫地机器人识别电线特别有效
        if (fill_ratio < 0.2)   // ⚠️ 可调参数（0.1~0.5）
            continue;

        // -----------------------------
        // 5️⃣ 轮廓平滑（多边形逼近）
        // -----------------------------
        // 作用：
        // - 减少轮廓点数量
        // - 提高稳定性（便于后续路径规划/避障）
        std::vector<cv::Point> approx;
        cv::approxPolyDP(cnt, approx, 2.0, true); // 2.0 = 平滑精度（越大越平滑）

        // -----------------------------
        // 6️⃣ 保存结果
        // -----------------------------
        output_contours.push_back(approx);
    }
}*/

// void filter_mask_contours(
//     const std::vector<std::vector<cv::Point>> &input_contours,
//     std::vector<std::vector<cv::Point>> &output_contours)
// {
//     output_contours.clear();

//     for (const auto &cnt : input_contours)
//     {
//         // -----------------------------
//         // 1️⃣ 面积过滤（适配1920分辨率）
//         // -----------------------------
//         double area = cv::contourArea(cnt);

//         if (area < 50) // ⭐ 原来200 → 改小（避免误杀远处/细线）
//             continue;

//         // -----------------------------
//         // 2️⃣ 外接矩形
//         // -----------------------------
//         cv::Rect rect = cv::boundingRect(cnt);

//         if (rect.width <= 0 || rect.height <= 0)
//             continue;

//         float fill_ratio = area / (rect.width * rect.height + 1e-5);

//         float length = std::max(rect.width, rect.height);
//         float thickness = std::min(rect.width, rect.height);

//         // -----------------------------
//         // 4️⃣ 周长过滤（弱化）
//         // -----------------------------
//         double perimeter = cv::arcLength(cnt, true);
//         if (perimeter < 30) // ⭐ 原来50 → 放宽
//             continue;

//         // -----------------------------
//         // 5️⃣ 过滤逻辑（分情况）
//         // -----------------------------

//         if (fill_ratio < 0.05)
//             continue;

//         // -----------------------------
//         // 6️⃣ 多边形逼近（平滑）
//         // -----------------------------
//         std::vector<cv::Point> approx;

//         // 普通物体：正常平滑
//         cv::approxPolyDP(cnt, approx, 2.0, true);

//         // -----------------------------
//         // 7️⃣ 防止点太少（避免异常）
//         // -----------------------------
//         if (approx.size() < 3)
//             continue;

//         // -----------------------------
//         // 8️⃣ 输出
//         // -----------------------------
//         output_contours.push_back(approx);
//     }
// }

/**
 * @brief 过滤 contour 中的离群点（尖刺点 / 飞点）
 *
 * 原理：
 *   如果某个点与前后点距离都非常大，
 *   则认为该点是异常点。
 *
 * 适用于：
 *   - segmentation mask 毛刺
 *   - 轮廓尖刺
 *   - 飞点
 *   - 边缘异常跳变
 *
 * @param input
 *        输入 contour
 *
 * @param output
 *        输出过滤后的 contour
 *
 * @param max_jump_dist
 *        最大允许跳变距离（像素）
 *
 *        1280x720 推荐：
 *          10 ~ 15
 */
static void filterContourOutlierPoints(
    const std::vector<cv::Point> &input,
    std::vector<cv::Point> &output,
    float max_jump_dist)
{
    output.clear();

    // 点太少不处理
    if (input.size() < 5)
    {
        output = input;
        return;
    }

    const int n = static_cast<int>(input.size());

    for (int i = 0; i < n; i++)
    {
        // 前一个点（循环）
        const cv::Point &prev =
            input[(i - 1 + n) % n];

        // 当前点
        const cv::Point &cur =
            input[i];

        // 后一个点（循环）
        const cv::Point &next =
            input[(i + 1) % n];

        // 与前点距离
        float d1 =
            cv::norm(cur - prev);

        // 与后点距离
        float d2 =
            cv::norm(cur - next);

        // ------------------------------------------------
        // 离群点过滤
        // ------------------------------------------------
        //
        // 如果：
        //   当前点与前后点都距离过远
        //
        // 说明：
        //   当前点是异常飞点
        //
        if (d1 > max_jump_dist &&
            d2 > max_jump_dist)
        {
            continue;
        }

        output.push_back(cur);
    }
}

/**
 * @brief 过滤 segmentation mask 提取出的轮廓点集
 *
 * 主要用于：
 *  - 去除 segmentation 噪点
 *  - 去除远距离小目标误检
 *  - 去除图像边缘误检
 *  - 保留较可信的线材/障碍物轮廓
 *
 * 适用于：
 *  - 扫地机器人 RGB-only 视觉避障
 *  - YOLOv8-seg / instance segmentation 后处理
 *
 * @param input_contours
 *        输入轮廓数组（extract_seg_mask_contours 提取得到）
 *
 * @param output_contours
 *        输出过滤后的轮廓数组
 *
 * @param img
 *        原始 RGB 图像
 *        用于获取图像尺寸，进行空间位置过滤
 */
void filter_mask_contours(
    const std::vector<std::vector<cv::Point>> &input_contours,
    std::vector<std::vector<cv::Point>> &output_contours,
    const cv::Mat &img)
{
    output_contours.clear();

    const int img_width = img.cols;
    const int img_height = img.rows;

    for (const auto &cnt : input_contours)
    {
        // =====================================================
        // 1️⃣ contour点数量过滤
        // =====================================================

        if (cnt.size() < 8)
            continue;

        // =====================================================
        // 2️⃣ contour离群点过滤（新增）
        // =====================================================
        std::vector<cv::Point> filtered_cnt;
        filterContourOutlierPoints(
            cnt,
            filtered_cnt,
            15.0f);

        // 离群点过滤后点太少
        if (filtered_cnt.size() < 5)
        {
            continue;
        }

        // =====================================================
        // 2️⃣ 面积
        // =====================================================

        double area = cv::contourArea(cnt);

        // =====================================================
        // 3️⃣ 周长
        // =====================================================

        double perimeter =
            cv::arcLength(cnt, true);

        if (perimeter < 20)
            continue;

        // =====================================================
        // 4️⃣ 最小旋转矩形（比boundingRect稳定）
        // =====================================================

        cv::RotatedRect r =
            cv::minAreaRect(cnt);

        float w = r.size.width;
        float h = r.size.height;

        if (w < 1 || h < 1)
            continue;

        // =====================================================
        // 5️⃣ contour中心点
        // =====================================================

        float cx = r.center.x;
        float cy = r.center.y;

        // =====================================================
        // 6️⃣ 图像边缘过滤
        // =====================================================

        // 左右边缘容易畸变
        if (cx < img_width * 0.01f ||
            cx > img_width * 0.99f)
        {
            continue;
        }

        // =====================================================
        // 7️⃣ 动态区域过滤（关键）
        // =====================================================

        float min_area = 30;
        float min_perimeter = 20;

        // 图像上部（远距离）
        if (cy < img_height * 0.25f)
        {
            min_area = 120;
            min_perimeter = 60;
        }
        // 图像中部
        else if (cy < img_height * 0.5f)
        {
            min_area = 70;
            min_perimeter = 40;
        }

        if (area < min_area)
            continue;

        if (perimeter < min_perimeter)
            continue;

        // =====================================================
        // 8️⃣ contour平滑（轻量）
        // =====================================================

        std::vector<cv::Point> approx;

        cv::approxPolyDP(
            cnt,
            approx,
            1.0,
            true);

        if (approx.size() < 3)
            continue;

        // =====================================================
        // 9️⃣ 输出
        // =====================================================

        output_contours.push_back(approx);
    }
}

bool isEdgePointValid(
    const ObjectCameraDetectResult &one,
    float max_dist,   // 👉 最大距离阈值（Z轴）作为参数传入
    size_t min_points // 👉 最小边缘点数作为参数传入
)
{
    const auto &pts = one.add_edge_point_single_pixel_camera_coordinates;

    // printf("into isEdgePointValid, edge point num = %zu\n", pts.size());

    // 👉 1. 点数过滤
    if (pts.size() < min_points)
    {
        // printf("Invalid: too few edge points (%zu < %zu)\n", pts.size(), min_points);
        return false;
    }

    // 👉 2. 距离过滤
    for (const auto &pt : pts)
    {
        if (pt.Z > max_dist)
        {
            // printf("\nInvalid edge point found with Z = %f\n\n", pt.Z);
            return false;
        }
    }

    return true;
}
