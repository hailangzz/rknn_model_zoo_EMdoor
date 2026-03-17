#include "carpet_detect_interface.h"
#include "detect_context.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include "image_drawing.h"

static DetectContext g_ctx;

bool carpet_model_init(const char* config_path)
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

/*
bool carpet_detect_infer(const cv::Mat& img, std::vector<ObjectCameraDetectResult>& results)
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
        ObjectCameraDetectResult one;

        // 坐标转换 （转换为，原始未矫正的，xyz尺寸值）
        g_ctx.camera_params->ObjectboxToCameraXYZ(det->box, det->camera_coordinates);

        one.prop   = det->prop;
        // one.cls_id = det->cls_id;   //模型输出的目标类别
        one.cls_id = g_ctx.config.CARPET_AREA;               // 同建图同事，确认的地毯区域目标ID，用于定义建图时的重要程度

        one.coords[0].X = det->camera_coordinates.left_top.X;
        one.coords[0].Y = det->camera_coordinates.left_top.Y;
        // one.coords[0].Z = det->camera_coordinates.left_top.Z;
        // one.coords[0].Z = det->camera_coordinates.left_top.Z*g_ctx.config.camera_z_axle_top_resize_rate;
        one.coords[0].Z = estimateDistance(det->camera_coordinates.left_top.Z);

        one.coords[1].X = det->camera_coordinates.right_top.X;
        one.coords[1].Y = det->camera_coordinates.right_top.Y;
        // one.coords[1].Z = det->camera_coordinates.right_top.Z;
        // one.coords[1].Z = det->camera_coordinates.right_top.Z*g_ctx.config.camera_z_axle_top_resize_rate;
        one.coords[1].Z = estimateDistance(det->camera_coordinates.right_top.Z);

        one.coords[2].X = det->camera_coordinates.right_bottom.X;
        one.coords[2].Y = det->camera_coordinates.right_bottom.Y;
        // one.coords[2].Z = det->camera_coordinates.right_bottom.Z;
        one.coords[2].Z = estimateDistance(det->camera_coordinates.right_bottom.Z);

        one.coords[3].X = det->camera_coordinates.left_bottom.X;
        one.coords[3].Y = det->camera_coordinates.left_bottom.Y;
        // one.coords[3].Z = det->camera_coordinates.left_bottom.Z;
        one.coords[3].Z = estimateDistance(det->camera_coordinates.left_bottom.Z);

        one.target_box.top    = det->box.top;
        one.target_box.bottom = det->box.bottom;
        one.target_box.left   = det->box.left;
        one.target_box.right  = det->box.right;

        // ===== 新增：edge 目标框，边界线处 采样点 =====        
        for (int index = 0; index < 20; ++index) {
            const auto& src = det->camera_coordinates.add_edge_point_single_pixel_camera_coordinates[index];

            single_pixel_camera_coordinates dst = src;
            dst.Z = estimateDistance(src.Z);

            // 正确赋值
            one.add_edge_point_single_pixel_camera_coordinates[index].X = dst.X;
            one.add_edge_point_single_pixel_camera_coordinates[index].Y = dst.Y;
            one.add_edge_point_single_pixel_camera_coordinates[index].Z = dst.Z;

            // printf("det->camera_coordinates.add_edge_point_single_pixel_camera_coordinates x:%.3f m, y:=%.3f m, z:=%.3f m\n", dst.X, dst.Y,dst.Z);

        }

        
        results.push_back(one); 

        ObjectSize3D size;
        if (calcObjectSizeByAverage(one, size)) {
            // printf("Object size: width=%.3f m, height=%.3f m\n", size.width, size.height);
        }
    }

    // // 调试输出
    // for (int i = 0; i < od_results.count; i++) {
    //     auto& det = results[i];
    //     printf("det.cls_id:%d, det.prop:%f\n", det.cls_id, det.prop);
    // }

    free(src_image.virt_addr);

    return !results.empty();
}
*/

bool carpet_detect_infer(const cv::Mat& img, std::vector<ObjectCameraDetectResult>& results)
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
        ObjectCameraDetectResult one;        

        std::vector<std::vector<cv::Point>> contours_mark_point;   // 存储轮廓点集
        std::vector<std::vector<cv::Point>> contours_mark_point_smoothed;   // 存储轮廓点集（平滑后）
        //获取模型推理，mark轮廓点集数组信息。
        extract_seg_mask_contours(od_results, i, src_image.width, src_image.height, contours_mark_point);
        for (size_t j = 0; j < contours_mark_point.size(); j++)
        {
            printf("Contour %zu, points = %zu\n", j, contours_mark_point[j].size());
            smoothContour(contours_mark_point[j], contours_mark_point_smoothed[j]);
            for (const auto &pt : contours_mark_point[j])
                printf("(%d,%d) ", pt.x, pt.y);
            printf("\n");
        }
        
        memset(&det->camera_coordinates, 0, sizeof(box_camera_coordinates)); //初始化
        // 坐标转换 （转换为，原始未矫正的，xyz尺寸值）
        g_ctx.camera_params->ObjectboxToCameraXYZ(det, contours_mark_point_smoothed);
        
        // 安全打印
        auto &coord = det->camera_coordinates;
        printf("Left Bottom: X=%f Y=%f Z=%f\n",
            coord.left_bottom.X, coord.left_bottom.Y, coord.left_bottom.Z);
        printf("Right Bottom: X=%f Y=%f Z=%f\n",
            coord.right_bottom.X, coord.right_bottom.Y, coord.right_bottom.Z);
        printf("Right Top: X=%f Y=%f Z=%f\n",
            coord.right_top.X, coord.right_top.Y, coord.right_top.Z);
        printf("Left Top: X=%f Y=%f Z=%f\n",
            coord.left_top.X, coord.left_top.Y, coord.left_top.Z);
        printf("%d\n", det->box.left);
        printf("%d\n", det->box.top);
        printf("%d\n", det->box.right);
        printf("%d\n", det->box.bottom);
        
        fillCameraDetectResult(det, one, g_ctx.config);

        results.push_back(one); 

        // 边框合法性校验：
        ObjectSize3D size;
        if (calcObjectSizeByAverage(one, size)) {
            // printf("Object size: width=%.3f m, height=%.3f m\n", size.width, size.height);
        }
    }

    // // 调试输出
    // for (int i = 0; i < od_results.count; i++) {
    //     auto& det = results[i];
    //     printf("det.cls_id:%d, det.prop:%f\n", det.cls_id, det.prop);
    // }

    free(src_image.virt_addr);

    return !results.empty();
}

void carpet_model_release()
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

