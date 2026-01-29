// Copyright (c) 2023 by Rockchip Electronics Co., Ltd. All Rights Reserved.
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

/*-------------------------------------------
                Includes
-------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "yolov8_detect.h"
#include "image_utils.h"
#include "file_utils.h"
#include "image_drawing.h"
#include "transform_coordinates.h"
#include "hand_detect_interface.h"

#if defined(RV1106_1103) 
    #include "dma_alloc.hpp"
#endif

#include <sys/time.h>


double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }



// helper: RGB -> NV21
void RGBToNV21(const uint8_t* rgb, int width, int height, uint8_t* nv21)
{
    uint8_t* y_plane = nv21;
    uint8_t* vu_plane = nv21 + width * height;

    int frame_size = width * height;

    for (int j = 0; j < height; j++)
    {
        for (int i = 0; i < width; i++)
        {
            int rgb_index = (j * width + i) * 3;
            uint8_t r = rgb[rgb_index + 0];
            uint8_t g = rgb[rgb_index + 1];
            uint8_t b = rgb[rgb_index + 2];

            // 标准 YUV 转换公式
            uint8_t y = (uint8_t)(( 66*r + 129*g + 25*b + 128) >> 8) + 16;
            uint8_t u = (uint8_t)((-38*r - 74*g + 112*b + 128) >> 8) + 128;
            uint8_t v = (uint8_t)((112*r - 94*g - 18*b + 128) >> 8) + 128;

            y_plane[j*width + i] = y;

            // 只在偶数行偶数列采样 UV
            if ((j % 2 == 0) && (i % 2 == 0))
            {
                int vu_index = (j/2)*width + i;
                vu_plane[vu_index]     = v;
                vu_plane[vu_index + 1] = u;
            }
        }
    }
}

// helper: RGBA -> NV21
void RGBAToNV21(const uint8_t* rgba, int width, int height, uint8_t* nv21)
{
    uint8_t* rgb = new uint8_t[width * height * 3];
    for (int i = 0; i < width * height; i++)
    {
        rgb[i*3 + 0] = rgba[i*4 + 0];
        rgb[i*3 + 1] = rgba[i*4 + 1];
        rgb[i*3 + 2] = rgba[i*4 + 2];
    }
    RGBToNV21(rgb, width, height, nv21);
    delete[] rgb;
}

// helper: NV12 -> NV21 (只需要交换 U/V)
void NV12ToNV21(const uint8_t* nv12, int width, int height, uint8_t* nv21)
{
    int frame_size = width * height;
    memcpy(nv21, nv12, frame_size); // 拷贝 Y

    const uint8_t* uv = nv12 + frame_size;
    uint8_t* vu = nv21 + frame_size;

    for (int i = 0; i < frame_size / 2; i += 2)
    {
        vu[i]   = uv[i+1]; // V
        vu[i+1] = uv[i];   // U
    }
}


bool ConvertImageBufferToNV21(const image_buffer_t& src, AndroidImageNV21& dst)
{
    // 1. 检查输入指针
    if (!src.virt_addr || src.width <= 0 || src.height <= 0) {
        return false;
    }

    // 2. 保证宽高为偶数
    int width  = src.width & ~1;
    int height = src.height & ~1;
    int image_size = width * height * 3 / 2;

    // 3. 分配 NV21 缓冲区
    dst.image_input_nv21 = new uint8_t[image_size];
    if (!dst.image_input_nv21) return false;

    dst.image_width  = width;
    dst.image_height = height;

    // printf("[ConvertImageBufferToNV21] src.format = %d, w=%d, h=%d\n",
    //    src.format, src.width, src.height);

    // 4. 根据格式处理
    switch(src.format)
    {
        case IMAGE_FORMAT_GRAY8:
        {
            uint8_t* y_plane  = dst.image_input_nv21;
            uint8_t* vu_plane = dst.image_input_nv21 + width * height;

            // Y 分量
            for (int i = 0; i < width * height; i++) {
                y_plane[i] = src.virt_addr[i];
            }

            // UV 分量填充灰色 (V=128, U=128)
            for (int i = 0; i < width * height / 2; i++) {
                vu_plane[i] = 128;
            }
            break;
        }

        case IMAGE_FORMAT_RGB888:
            RGBToNV21(src.virt_addr, width, height, dst.image_input_nv21);
            break;

        case IMAGE_FORMAT_RGBA8888:
            RGBAToNV21(src.virt_addr, width, height, dst.image_input_nv21);
            break;

        case IMAGE_FORMAT_YUV420SP_NV21:
            memcpy(dst.image_input_nv21, src.virt_addr, image_size);
            break;

        case IMAGE_FORMAT_YUV420SP_NV12:
            NV12ToNV21(src.virt_addr, width, height, dst.image_input_nv21);
            break;

        default:
            // 不支持的格式
            delete[] dst.image_input_nv21;
            dst.image_input_nv21 = nullptr;
            return false;
    }

    return true;
}


void PrintHandDetectResult(HandDetectResult result)
{
    switch(result)
    {
        case HandDetectResult::Processing:  std::cout << "Processing"; break;
        case HandDetectResult::NoHand:      std::cout << "NoHand"; break;
        case HandDetectResult::HandDetected: std::cout << "HandDetected"; break;
        default: std::cout << "Unknown"; break;
    }
}
/*-------------------------------------------
                  Main Function
-------------------------------------------*/
int main(int argc, char **argv)
{
    struct timeval start_time, stop_time;

    ConfigInfo config_info = readConfig("./config/cfg.txt");

    Detector detector(config_info);
    CameraParameters camera_parameters(config_info);


    std::string image_path = "./model/hand.jpg";
    image_buffer_t src_image;
    memset(&src_image, 0, sizeof(image_buffer_t));
    int ret = read_image(image_path.c_str(), &src_image);
    // write_image("origin.png", &src_image);

    AndroidImageNV21 image_input;
    ConvertImageBufferToNV21(src_image, image_input);
    // write_image("ConvertImageBufferToNV21.png", &src_image);

    bool is_save_images = true;
    HandDetectResult results = hand_detect_interface(&image_input,is_save_images);
    
    std::cout << "HandDetect result: ";
    PrintHandDetectResult(results);
    std::cout << std::endl;

#if defined(RV1106_1103)
    //RV1106 rga requires that input and output bufs are memory allocated by dma
    ret = dma_buf_alloc(RV1106_CMA_HEAP_PATH, src_image.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                       (void **) & (rknn_app_ctx.img_dma_buf.dma_buf_virt_addr));
    memcpy(rknn_app_ctx.img_dma_buf.dma_buf_virt_addr, src_image.virt_addr, src_image.size);
    dma_sync_cpu_to_device(rknn_app_ctx.img_dma_buf.dma_buf_fd);
    free(src_image.virt_addr);
    src_image.virt_addr = (unsigned char *)rknn_app_ctx.img_dma_buf.dma_buf_virt_addr;
    src_image.fd = rknn_app_ctx.img_dma_buf.dma_buf_fd;
    rknn_app_ctx.img_dma_buf.size = src_image.size;
#endif
    
    if (ret != 0)
    {
        printf("read image fail! ret=%d image_path=%s\n", ret, image_path.c_str());
        goto out;
    }

    object_detect_result_list od_results;

    // 计算耗时
    gettimeofday(&start_time, NULL);

    // ret = detector.inference_yolov8_model(&src_image, &od_results);
    // if (ret != 0)
    // {
    //     printf("init_yolov8_model fail! ret=%d\n", ret);
    //     goto out;
    // }

    // gettimeofday(&stop_time, NULL);
    // printf("once run use %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);
    
    // // 画框和概率
    // char text[256];
    // for (int i = 0; i < od_results.count; i++)
    // {
    //     object_detect_result *det_result = &(od_results.results[i]);
    //     printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),
    //            det_result->box.left, det_result->box.top,
    //            det_result->box.right, det_result->box.bottom,
    //            det_result->prop);
        
    //     int x1 = det_result->box.left;
    //     int y1 = det_result->box.top;
    //     int x2 = det_result->box.right;
    //     int y2 = det_result->box.bottom;

    //     draw_rectangle(&src_image, x1, y1, x2 - x1, y2 - y1, COLOR_BLUE, 3);
        
    //     printf("finish draw_rectangle!!!!! \n");
    //     // camera_parameters.ObjectboxToCameraXYZ(od_results.results[i].box, od_results.results[i].camera_coordinates);
    //     camera_parameters.ObjectboxToCameraXYZ(det_result->box, det_result->camera_coordinates);

    //     sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), det_result->prop * 100);
    //     draw_text(&src_image, text, x1, y1 - 20, COLOR_RED, 10);
    // }

    // write_image("out.png", &src_image);

out:
    deinit_post_process();

    if (src_image.virt_addr != NULL)
    {
#if defined(RV1106_1103) 
        dma_buf_free(rknn_app_ctx.img_dma_buf.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                rknn_app_ctx.img_dma_buf.dma_buf_virt_addr);
#else
        
        free(src_image.virt_addr);
        
#endif
    }

    return 0;
}
