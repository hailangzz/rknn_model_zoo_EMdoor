#include "carpet_detect_interface.h"
#include "detect_context.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include "image_drawing.h"

// carpet_detect_interface.cpp
static DetectContext g_ctx;

inline float estimateDistance(float x)
{
    float polyfit_result;
    polyfit_result = g_ctx.config.camera_z_axle_top_resize_rate * x;
    polyfit_result = g_ctx.config.camera_z_axle_polyfit_w0 * x * x + g_ctx.config.camera_z_axle_polyfit_w1 * x + g_ctx.config.camera_z_axle_polyfit_w2;
    // y = 0.2220 x^2 + 0.8531 x + 0.1568
    return polyfit_result;
}

bool carpet_model_init(const char *config_path)
{
    if (g_ctx.initialized)
    {
        printf("carpet model already initialized");
        return true;
    }
    std::string config_file_path(config_path);
    g_ctx.config = readConfig(config_file_path);
    init_post_process();

    g_ctx.detector = new Detector(g_ctx.config);
    g_ctx.camera_params = new CameraParameters(g_ctx.config);
    g_ctx.debuger = new Debug(g_ctx.config.save_debug_images_path, g_ctx.config.is_save_debug_images, g_ctx.config.fps_limit);

    g_ctx.initialized = true;

    printf("carpet_model_init success");
    return true;
}

// ================= 保存结果图像 =================
static int save_index = 0; // 静态自增计数器

const char *save_dir = "./result_images";

bool carpet_detect_infer(const cv::Mat &img,
                         std::vector<ObjectCameraDetectResult> &results)
{
    results.clear();

    // ================= 基础检查 =================
    if (!g_ctx.initialized)
    {
        printf("[ERROR] model not initialized\n");
        return false;
    }

    if (img.empty() || img.data == nullptr)
    {
        printf("[ERROR] input image empty\n");
        return false;
    }

    printf("\n========== carpet_detect_infer ==========\n");
    printf("input image: w=%d h=%d c=%d\n",
           img.cols,
           img.rows,
           img.channels());

    printf("input image data ptr: %p\n", img.data);

    // ================= 深拷贝，避免多线程 Mat 野指针 =================
    cv::Mat safe_img = img.clone();

    if (safe_img.empty() || safe_img.data == nullptr)
    {
        printf("[ERROR] safe_img clone failed\n");
        return false;
    }

    // ================= 转 RGB =================
    cv::Mat img_rgb;

    if (safe_img.channels() == 4)
    {
        cv::cvtColor(safe_img, img_rgb, cv::COLOR_RGBA2RGB);
    }
    else if (safe_img.channels() == 1)
    {
        cv::cvtColor(safe_img, img_rgb, cv::COLOR_GRAY2RGB);
    }
    else if (safe_img.channels() == 3)
    {
        // 深拷贝
        img_rgb = safe_img.clone();
    }
    else
    {
        printf("[ERROR] unsupported image channels: %d\n",
               safe_img.channels());
        return false;
    }

    // ================= 检查 RGB 图像 =================
    if (img_rgb.empty() || img_rgb.data == nullptr)
    {
        printf("[ERROR] img_rgb invalid\n");
        return false;
    }

    // ================= 保证连续内存 =================
    if (!img_rgb.isContinuous())
    {
        printf("[WARN] img_rgb not continuous, clone again\n");
        img_rgb = img_rgb.clone();
    }

    if (!img_rgb.isContinuous())
    {
        printf("[ERROR] img_rgb still not continuous\n");
        return false;
    }

    printf("rgb image: w=%d h=%d c=%d\n",
           img_rgb.cols,
           img_rgb.rows,
           img_rgb.channels());

    printf("rgb image ptr: %p\n", img_rgb.data);

    // ================= 构建 RKNN 输入 =================
    image_buffer_t src_image;
    memset(&src_image, 0, sizeof(image_buffer_t));

    size_t buffer_size =
        img_rgb.cols *
        img_rgb.rows *
        img_rgb.channels();

    printf("buffer_size=%zu\n", buffer_size);

    // 64 字节对齐
    if (posix_memalign((void **)&src_image.virt_addr,
                       64,
                       buffer_size) != 0)
    {
        printf("[ERROR] posix_memalign failed\n");
        return false;
    }

    if (src_image.virt_addr == nullptr)
    {
        printf("[ERROR] aligned memory null\n");
        return false;
    }

    memset(src_image.virt_addr, 0, buffer_size);

    // ================= 拷贝图像数据 =================
    memcpy(src_image.virt_addr,
           img_rgb.data,
           buffer_size);

    src_image.width = img_rgb.cols;
    src_image.height = img_rgb.rows;
    src_image.format = IMAGE_FORMAT_RGB888;
    src_image.size = buffer_size;

    // ================= 推理 =================
    object_detect_result_list od_results;
    memset(&od_results, 0, sizeof(object_detect_result_list));

    printf("start inference...\n");

    int ret =
        g_ctx.detector->inference_yolov8_model(
            &src_image,
            &od_results);

    if (ret != 0)
    {
        printf("[ERROR] yolov8 inference failed ret=%d\n", ret);

        free(src_image.virt_addr);
        src_image.virt_addr = nullptr;

        return false;
    }

    printf("inference success, detect count=%d\n",
           od_results.count);

    // ================= 后处理 =================
    float box_max_prop = 0.f;

    for (int i = 0; i < od_results.count; i++)
    {
        object_detect_result *det = &od_results.results[i];

        ObjectCameraDetectResult one;

        box_max_prop =
            std::max(box_max_prop, det->prop);

        // 坐标转换
        g_ctx.camera_params->ObjectboxToCameraXYZ(
            det->box,
            det->camera_coordinates);

        one.prop = det->prop;

        one.cls_id = g_ctx.config.CARPET_AREA;

        // 四点坐标
        one.coords[0].X =
            det->camera_coordinates.left_top.X;
        one.coords[0].Y =
            det->camera_coordinates.left_top.Y;
        one.coords[0].Z =
            estimateDistance(
                det->camera_coordinates.left_top.Z);

        one.coords[1].X =
            det->camera_coordinates.right_top.X;
        one.coords[1].Y =
            det->camera_coordinates.right_top.Y;
        one.coords[1].Z =
            estimateDistance(
                det->camera_coordinates.right_top.Z);

        one.coords[2].X =
            det->camera_coordinates.right_bottom.X;
        one.coords[2].Y =
            det->camera_coordinates.right_bottom.Y;
        one.coords[2].Z =
            estimateDistance(
                det->camera_coordinates.right_bottom.Z);

        one.coords[3].X =
            det->camera_coordinates.left_bottom.X;
        one.coords[3].Y =
            det->camera_coordinates.left_bottom.Y;
        one.coords[3].Z =
            estimateDistance(
                det->camera_coordinates.left_bottom.Z);

        // box
        one.target_box.top = det->box.top;
        one.target_box.bottom = det->box.bottom;
        one.target_box.left = det->box.left;
        one.target_box.right = det->box.right;

        // edge points
        const int src_edge_point_num =
            sizeof(det->camera_coordinates.add_edge_point_single_pixel_camera_coordinates) / sizeof(det->camera_coordinates.add_edge_point_single_pixel_camera_coordinates[0]);

        const int dst_edge_point_num =
            sizeof(one.add_edge_point_single_pixel_camera_coordinates) / sizeof(one.add_edge_point_single_pixel_camera_coordinates[0]);

        const int edge_point_num =
            std::min(src_edge_point_num, dst_edge_point_num);

        for (int index = 0; index < edge_point_num; ++index)
        {
            const auto &src =
                det->camera_coordinates
                    .add_edge_point_single_pixel_camera_coordinates[index];

            one.add_edge_point_single_pixel_camera_coordinates[index].X =
                src.X;

            one.add_edge_point_single_pixel_camera_coordinates[index].Y =
                src.Y;

            one.add_edge_point_single_pixel_camera_coordinates[index].Z =
                estimateDistance(src.Z);
        }

        results.push_back(one);
    }

    // ================= Debug 保存 =================
    if (g_ctx.debuger &&
        box_max_prop > g_ctx.config.debug_score_threshold)
    {
        printf("save debug image\n");

        g_ctx.debuger->saveIfDetected(
            safe_img,
            "carpet_detect");
    }

    // ================= 释放 =================
    free(src_image.virt_addr);
    src_image.virt_addr = nullptr;

    printf("========== infer finished ==========\n\n");

    return !results.empty();
}

void carpet_model_release()
{
    if (!g_ctx.initialized)
        return;

    deinit_post_process();

    delete g_ctx.detector;
    delete g_ctx.camera_params;
    delete g_ctx.debuger;

    g_ctx.detector = nullptr;
    g_ctx.camera_params = nullptr;
    g_ctx.debuger = nullptr;

    g_ctx.initialized = false;

    printf("carpet_model_release finished");
}
