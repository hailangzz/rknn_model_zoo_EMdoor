#include "base_detect_interface.h"
#include "detect_context.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include "image_drawing.h"

static DetectContext g_ctx;

bool base_model_init(const char* config_path)
{
    if (g_ctx.initialized) {
        printf("carpet model already initialized");
        return true;
    }
    std::string config_file_path(config_path);
    g_ctx.config = readConfig(config_file_path);
    init_post_process();

    g_ctx.detector = new Detector(g_ctx.config);
    g_ctx.camera_params = new CameraParameters(g_ctx.config);
    
    g_ctx.initialized = true;

    printf("carpet_model_init success");
    return true;
}

// ================= 保存结果图像 =================
static int save_index = 0;   // 静态自增计数器

const char* save_dir = "./result_images";


bool base_detect_infer(const cv::Mat& img, std::vector<ObjectCameraDetectResult>& results)
{
    results.clear();

    if (!g_ctx.initialized || img.empty()) {
        printf("model not initialized or empty image\n");
        return false;
    }

    cv::Mat img_rgb;

    // 如果原图是 4 通道 RGBA，转换为 RGB
    if (img.channels() == 4) {
        cv::cvtColor(img, img_rgb, cv::COLOR_RGBA2RGB);
    } else if (img.channels() == 1) {
        cv::cvtColor(img, img_rgb, cv::COLOR_GRAY2RGB);
    } else {
        img_rgb = img;
    }

    image_buffer_t src_image;
    memset(&src_image, 0, sizeof(image_buffer_t));

    size_t buffer_size = img_rgb.total() * img_rgb.elemSize();

    // 使用 posix_memalign 分配对齐内存，避免 RGA 偶发日志
    if (posix_memalign((void**)&src_image.virt_addr, 64, buffer_size) != 0) {
        printf("failed to allocate aligned memory\n");
        return false;
    }

    // 初始化缓冲区
    memset(src_image.virt_addr, 0, buffer_size);

    // 拷贝图像数据
    memcpy(src_image.virt_addr, img_rgb.data, buffer_size);

    src_image.width  = img_rgb.cols;
    src_image.height = img_rgb.rows;
    src_image.format = IMAGE_FORMAT_RGB888; // 确保 RGA / NPU 支持
    src_image.size   = buffer_size;

    object_detect_result_list od_results;

    // YOLOv8 推理
    int ret = g_ctx.detector->inference_yolov8_model(&src_image, &od_results);
    if (ret != 0) {
        printf("yolov8 inference failed\n");
        free(src_image.virt_addr);
        return false;
    }

    // 遍历检测结果
    for (int i = 0; i < od_results.count; i++) {

        object_detect_result* det = &od_results.results[i];
        object_segment_result*seg = &od_results.results_seg[i];
        ObjectCameraDetectResult one;        

        std::vector<std::vector<cv::Point>> contours_mark_point;   // 存储轮廓点集
        std::vector<std::vector<cv::Point>> contours_mark_point_smoothed;   // 存储轮廓点集（平滑后）
        std::vector<std::vector<cv::Point>> contours_mark_point_filtered;   // 存储过滤后的轮廓点集
        
        //获取模型推理，mark轮廓点集数组信息。
        // extract_seg_mask_contours(od_results, i, src_image.width, src_image.height, contours_mark_point);
        // 获取 mask 轮廓
        extract_seg_mask_contours(seg, src_image.width, src_image.height, contours_mark_point);

        int total_points = 0;
        for (const auto& contour : contours_mark_point) {
            total_points += contour.size();
        }
        printf("det[%d] filtered total points = %d\n", i, total_points);

        filter_mask_contours(contours_mark_point, contours_mark_point_filtered); // 过滤轮廓点集（去除小面积、长宽比异常的轮廓）
        // one.object_contours_mark_point = contours_mark_point;     // 边界点数组赋值

        contours_mark_point_smoothed.resize(contours_mark_point_filtered.size());   // 防止数组越界
        for (size_t j = 0; j < contours_mark_point_filtered.size(); j++)
        {
            smoothContour(contours_mark_point_filtered[j], contours_mark_point_smoothed[j]);
        }
        
        // 👉 存储
        one.object_contours_mark_point = contours_mark_point_filtered;
        
        memset(&det->camera_coordinates, 0, sizeof(box_camera_coordinates)); //初始化

        

        // 坐标转换 （转换为，原始未矫正的，xyz尺寸值）
        g_ctx.camera_params->ObjectboxToCameraXYZ(det, contours_mark_point_filtered);
        
        
        fillCameraDetectResult(det, one, g_ctx.config); // 结果值填充

        // // 安全打印
        // auto &coord = det->camera_coordinates;
        // printf("Left Bottom: X=%f Y=%f Z=%f\n",
        //     coord.left_bottom.X, coord.left_bottom.Y, coord.left_bottom.Z);
        // printf("Right Bottom: X=%f Y=%f Z=%f\n",
        //     coord.right_bottom.X, coord.right_bottom.Y, coord.right_bottom.Z);
        // printf("Right Top: X=%f Y=%f Z=%f\n",
        //     coord.right_top.X, coord.right_top.Y, coord.right_top.Z);
        // printf("Left Top: X=%f Y=%f Z=%f\n",
        //     coord.left_top.X, coord.left_top.Y, coord.left_top.Z);
        // printf("%d\n", det->box.left);
        // printf("%d\n", det->box.top);
        // printf("%d\n", det->box.right);
        // printf("%d\n", det->box.bottom);

        results.push_back(one); 

        // 边框合法性校验：
        ObjectSize3D size;
        if (calcObjectSizeByAverage(one, size)) {
            // printf("Object size: width=%.3f m, height=%.3f m\n", size.width, size.height);
        }
    }

    // 调试输出
    // 打印总数量
    printf("od_results.count: %d\n", od_results.count);
    for (int i = 0; i < od_results.count; i++) {
        auto& det = results[i];
        printf("det.cls_id:%d, det.prop:%f\n", det.cls_id, det.prop);
    }
    

    free(src_image.virt_addr);

    return !results.empty();
}

void base_model_release()
{
    if (!g_ctx.initialized)
        return;

    deinit_post_process();

    delete g_ctx.detector;
    delete g_ctx.camera_params;

    g_ctx.detector = nullptr;
    g_ctx.camera_params = nullptr;
    g_ctx.initialized = false;

    printf("carpet_model_release finished");
}

