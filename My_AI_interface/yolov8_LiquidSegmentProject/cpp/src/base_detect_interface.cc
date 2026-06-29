#include "base_detect_interface.h"
#include "detect_context.h"
#include "track_filter.h"

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
        g_ctx.detection_track_filter = new TrackFilter(3, 2, 5, 120.0f, 2.5f);

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

    // ================= 处理结果 =================
    for (int i = 0; i < od_results.count; i++)
    {

        object_detect_result *det = &od_results.results[i];

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

        // 过滤掉边缘点超过 2 米，或者边缘点数量小于 5 的目标（剔除远距离误检目标）
        if (!isEdgePointValid(one, 2.0f, 5))
        {
            printf("Object edge points invalid: max distance > 2.0m or count < 5\n");
            continue;
        }
        // 边框合法性校验：过滤掉宽、高小于20厘米的目标
        ObjectSize3D size;
        float min_width = 0.04f;  // 20cm
        float min_height = 0.04f; // 20cm
        if (!calcObjectSizeByAverage(one, size, min_width, min_height))
        {
            printf("Object size: width=%.3f m, height=%.3f m\n", size.width, size.height);
            continue;
        }
        results.push_back(one);
    }

    //--------------------------------------
    // 时序滤波,过滤掉瞬间误检，及瞬间漏检目标（注意：此功能要防止干扰样本采集）
    //--------------------------------------

    std::vector<ObjectCameraDetectResult> track_filtered_results;

    g_ctx.detection_track_filter->update(
        results,
        track_filtered_results);
    results.swap(track_filtered_results);

    // 后面继续使用 results

    // ================= 打印 =================
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
    delete g_ctx.detection_track_filter;

    g_ctx.detector = nullptr;
    g_ctx.camera_params = nullptr;
    g_ctx.detection_track_filter = nullptr;

    g_ctx.initialized = false;

    printf("carpet_model_release finished\n");
}
