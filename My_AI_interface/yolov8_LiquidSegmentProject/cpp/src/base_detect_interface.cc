#include "base_detect_interface.h"
#include "detect_context.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <mutex>

#include "image_drawing.h"

static DetectContext g_ctx;
static std::mutex g_ctx_mutex;

// ================= 初始化 =================
bool base_model_init(const char *config_path)
{
    std::lock_guard<std::mutex> lock(g_ctx_mutex);

    if (g_ctx.initialized)
    {
        printf("carpet model already initialized\n");
        return true;
    }

    try
    {
        std::string config_file_path(config_path);
        g_ctx.config = readConfig(config_file_path);

        init_post_process();

        g_ctx.detector = new Detector(g_ctx.config);
        g_ctx.camera_params = new CameraParameters(g_ctx.config);
        g_ctx.debuger = new Debug(g_ctx.config.save_debug_images_path, g_ctx.config.is_save_debug_images, g_ctx.config.fps_limit);

        g_ctx.initialized = true;

        printf("carpet_model_init success\n");
        return true;
    }
    catch (...)
    {
        printf("model init failed\n");
        return false;
    }
}

// ================= 推理 =================
bool base_detect_infer(const cv::Mat &img, std::vector<ObjectCameraDetectResult> &results)
{
    std::lock_guard<std::mutex> lock(g_ctx_mutex);

    results.clear();

    if (!g_ctx.initialized || img.empty())
    {
        printf("model not initialized or empty image\n");
        return false;
    }

    cv::Mat img_rgb;

    if (img.channels() == 4)
    {
        cv::cvtColor(img, img_rgb, cv::COLOR_RGBA2RGB);
    }
    else if (img.channels() == 1)
    {
        cv::cvtColor(img, img_rgb, cv::COLOR_GRAY2RGB);
    }
    else
    {
        img_rgb = img.clone();
    }

    if (!img_rgb.isContinuous())
    {
        img_rgb = img_rgb.clone();
    }

    image_buffer_t src_image{};
    size_t buffer_size = img_rgb.total() * img_rgb.elemSize();

    if (posix_memalign((void **)&src_image.virt_addr, 64, buffer_size) != 0)
    {
        printf("failed to allocate aligned memory\n");
        return false;
    }

    memcpy(src_image.virt_addr, img_rgb.data, buffer_size);

    src_image.width = img_rgb.cols;
    src_image.height = img_rgb.rows;
    src_image.format = IMAGE_FORMAT_RGB888;
    src_image.size = buffer_size;

    object_detect_result_list od_results{};

    int ret = g_ctx.detector->inference_yolov8_model(&src_image, &od_results);
    if (ret != 0)
    {
        printf("yolov8 inference failed\n");
        free(src_image.virt_addr);
        return false;
    }

    // ================= 统计 =================
    float box_max_prop = std::numeric_limits<float>::lowest();

    // ================= 处理结果 =================
    for (int i = 0; i < od_results.count; i++)
    {

        object_detect_result *det = &od_results.results[i];

        // ---------- 最大置信度 ----------
        box_max_prop = std::max(box_max_prop, det->prop);

        // ---------- 阈值过滤 ----------
        if (det->prop < g_ctx.config.score_threshold)
        {
            continue;
        }

        // ⭐⭐⭐ seg 安全获取 ⭐⭐⭐
        object_segment_result *seg = nullptr;

        if (od_results.results_seg != nullptr &&
            i < od_results.count)
        {
            object_segment_result &seg_ref = od_results.results_seg[i];

            if (seg_ref.seg_mask != nullptr)
            {
                seg = &seg_ref;
            }
        }

        // ❗没有 seg 直接跳过（保持你原有策略）
        if (seg == nullptr)
        {
            continue;
        }

        ObjectCameraDetectResult one;

        std::vector<std::vector<cv::Point>> contours_mark_point;
        std::vector<std::vector<cv::Point>> contours_mark_point_filtered;
        std::vector<std::vector<cv::Point>> contours_mark_point_smoothed;

        extract_seg_mask_contours(seg, src_image.width, src_image.height, contours_mark_point);

        filter_mask_contours(contours_mark_point, contours_mark_point_filtered);

        contours_mark_point_smoothed.resize(contours_mark_point_filtered.size());
        for (size_t j = 0; j < contours_mark_point_filtered.size(); j++)
        {
            smoothContour(contours_mark_point_filtered[j],
                          contours_mark_point_smoothed[j]);
        }

        one.object_contours_mark_point = contours_mark_point_filtered;

        memset(&det->camera_coordinates, 0, sizeof(box_camera_coordinates));

        g_ctx.camera_params->ObjectboxToCameraXYZ(det, contours_mark_point_filtered);

        fillCameraDetectResult(det, one, g_ctx.config);

        if (!isEdgePointValid(one, 1.5f, 10))
        {
            continue;
        }

        results.push_back(one);
    }

    // ================= Debug 保存 =================
    if (g_ctx.debuger && box_max_prop > g_ctx.config.debug_score_threshold)
    {
        g_ctx.debuger->saveIfDetected(img, "liquid_detect");
    }

    // ================= Debug 输出 =================
    printf("od_results.count: %d\n", od_results.count);

    for (size_t i = 0; i < results.size(); i++)
    {
        const auto &det = results[i];
        printf("det.cls_id:%d, det.prop:%f\n", det.cls_id, det.prop);
    }

    // ================= 安全释放 seg_mask =================
    if (od_results.results_seg != nullptr)
    {
        for (int i = 0; i < od_results.count; i++)
        {
            if (od_results.results_seg[i].seg_mask)
            {
                free(od_results.results_seg[i].seg_mask);
                od_results.results_seg[i].seg_mask = nullptr;
            }
        }
    }

    free(src_image.virt_addr);

    return !results.empty();
}

// ================= 释放 =================
void base_model_release()
{
    std::lock_guard<std::mutex> lock(g_ctx_mutex);

    if (!g_ctx.initialized)
        return;

    deinit_post_process();

    delete g_ctx.detector;
    delete g_ctx.camera_params;

    g_ctx.detector = nullptr;
    g_ctx.camera_params = nullptr;
    g_ctx.initialized = false;

    printf("carpet_model_release finished\n");
}
