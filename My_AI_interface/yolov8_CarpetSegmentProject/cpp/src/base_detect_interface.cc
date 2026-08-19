#include "base_detect_interface.h"
#include "detect_context.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include "image_drawing.h"
#include "map.h"

static DetectContext g_ctx;

bool base_model_init(const char *config_path)
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

    // ========================================================
    // 清理普通 debug 图片
    g_ctx.debuger->removeExpiredDirectories(
        g_ctx.config.save_debug_images_path);

    // 清理空间位置采样图片
    g_ctx.debuger->removeExpiredDirectories(
        g_ctx.config.save_images_path_spatial_location_val);

    g_ctx.g_pose_sampler = new visual_localization::PoseSampler();
    g_ctx.initialized = true;

    printf("carpet_model_init success");
    return true;
}

// ============================================================
// 原始推理接口
// ============================================================
// bool base_detect_infer_real(
bool real_detect_infer(
    const cv::Mat &img,
    std::vector<ObjectCameraDetectResult> &results)
{
    results.clear();

    // ========================================================
    // 安全检查
    // ========================================================

    if (!g_ctx.initialized ||
        !g_ctx.detector ||
        !g_ctx.camera_params ||
        img.empty())
    {
        printf("model not initialized or empty image\n");
        return false;
    }

    // ========================================================
    // Mat处理
    // ========================================================

    cv::Mat img_rgb;

    if (img.channels() == 4)
    {
        cv::cvtColor(
            img,
            img_rgb,
            cv::COLOR_RGBA2RGB);
    }
    else if (img.channels() == 1)
    {
        cv::cvtColor(
            img,
            img_rgb,
            cv::COLOR_GRAY2RGB);
    }
    else
    {
        img_rgb = img;
    }

    if (!img_rgb.isContinuous())
    {
        img_rgb = img_rgb.clone();
    }

    // ========================================================
    // 构造输入
    // ========================================================

    image_buffer_t src_image;

    memset(
        &src_image,
        0,
        sizeof(image_buffer_t));

    size_t buffer_size =
        img_rgb.total() *
        img_rgb.elemSize();

    if (posix_memalign(
            (void **)&src_image.virt_addr,
            64,
            buffer_size) != 0)
    {
        printf("failed to allocate aligned memory\n");
        return false;
    }

    memcpy(
        src_image.virt_addr,
        img_rgb.data,
        buffer_size);

    src_image.width =
        img_rgb.cols;

    src_image.height =
        img_rgb.rows;

    src_image.format =
        IMAGE_FORMAT_RGB888;

    src_image.size =
        buffer_size;

    // ========================================================
    // 推理
    // ========================================================

    object_detect_result_list od_results;

    memset(
        &od_results,
        0,
        sizeof(od_results));

    int ret =
        g_ctx.detector->inference_yolov8_model(
            &src_image,
            &od_results);

    if (ret != 0)
    {
        printf("yolov8 inference failed\n");

        free(src_image.virt_addr);

        return false;
    }

    // 数据回传时，调试信息里的mark轮廓和类别id也一并回传，主要用于分析空间位置符合条件时，检测结果的有效性，以及空间位置不符合条件时，是否存在高置信度的检测结果
    std::vector<std::vector<cv::Point>> debug_all_contours;
    std::vector<int> debug_all_cls_ids;

    // ========================================================
    // 统计
    // ========================================================

    // float box_max_prop = std::numeric_limits<float>::lowest();
    float box_max_prop = 0.0f;

    // ========================================================
    // 遍历检测结果
    // ========================================================

    for (int i = 0; i < od_results.count; i++)
    {
        if (!od_results.results)
        {
            printf("results is null!\n");
            break;
        }

        object_detect_result *det =
            &od_results.results[i];

        box_max_prop =
            std::max(
                box_max_prop,
                det->prop);

        // 阈值过滤
        if (det->prop <
            g_ctx.config.score_threshold)
        {
            continue;
        }

        if (!od_results.results_seg)
        {
            printf("results_seg is null!\n");
            continue;
        }

        object_segment_result *seg =
            &od_results.results_seg[i];

        if (!seg)
        {
            printf("seg ptr invalid!\n");
            continue;
        }

        ObjectCameraDetectResult one;

        std::vector<std::vector<cv::Point>> contours;

        std::vector<std::vector<cv::Point>> contours_filtered;

        std::vector<std::vector<cv::Point>> contours_smoothed;

        // ====================================================
        // 提取轮廓
        // ====================================================

        extract_seg_mask_contours(
            seg,
            src_image.width,
            src_image.height,
            contours);

        // ====================================================
        // 过滤轮廓
        // ====================================================

        filter_mask_contours(
            contours,
            contours_filtered);

        // ====================================================
        // 平滑轮廓
        // ====================================================

        contours_smoothed.resize(
            contours_filtered.size());

        for (size_t j = 0;
             j < contours_filtered.size();
             j++)
        {
            // 局部凸包平滑
            smoothContourOuterOnly(
                contours_filtered[j],
                contours_smoothed[j]);
        }

        // ====================================================
        // 存储轮廓
        // ====================================================

        one.object_contours_mark_point =
            contours_filtered;

        memset(
            &det->camera_coordinates,
            0,
            sizeof(box_camera_coordinates));

        // ====================================================
        // 转换相机坐标
        // ====================================================

        g_ctx.camera_params->ObjectboxToCameraXYZ(
            det,
            contours_smoothed);

        // ====================================================
        // 填充结果
        // ====================================================

        fillCameraDetectResult(
            det,
            one,
            g_ctx.config);

        results.push_back(one);

        // 写入调试信息里的mark轮廓和类别id，主要用于分析空间位置符合条件时，检测结果的有效性
        for (const auto &contour : contours_smoothed)
        {
            debug_all_contours.push_back(contour);
            debug_all_cls_ids.push_back(det->cls_id);
        }

        // ====================================================
        // 尺寸计算
        // ====================================================

        ObjectSize3D size;
        // 进行尺寸计算，主要用于过滤掉一些不合理的检测结果（例如过大或过小的区域），以及为后续的跟踪和分析提供尺寸信息
        calcObjectSizeByAverage(
            one,
            size);
    }

    // ========================================================
    // Debug保存
    // ========================================================

    if (g_ctx.debuger) // 调试模式下
    {
        if (g_ctx.g_pose_sampler->IsPoseSaveImage) // 空间位置符合条件
        {
            // target_status: 判断当前图像的目标状态
            TargetStatus target_status;
            if (box_max_prop > g_ctx.config.score_threshold)
            {
                target_status = TargetStatus::EXISTS;
            }
            else if (box_max_prop > g_ctx.config.debug_score_threshold)
            {
                target_status = TargetStatus::MIDDLE;
            }
            else
            {
                // 由于在yolov8_detect中BOX_THRESH设置为0.35，因此此处confidence已被过滤掉，一直为0，所以当置信度较低时，直接归为NONE状态
                target_status = TargetStatus::NONE;
            }

            g_ctx.debuger->updateSavedPoseImageCount(target_status); // 更新基于空间的，图像目标有效性计数

            std::string task_name = g_ctx.config.task_name;

            std::string save_sample_info_string = g_ctx.debuger->set_ai_capture_save_info(g_ctx.debuger->device_id_sn, task_name, TargetStatusToStr(target_status), box_max_prop); // 设置AI自动化迭代的图像保存信息

            g_ctx.debuger->saveSegLabel(
                img,
                debug_all_contours,
                debug_all_cls_ids,
                task_name,
                save_sample_info_string,
                target_status);
        }
    }

    // ========================================================
    // Debug输出
    // ========================================================

    printf("od_results.count: %d\n",
           od_results.count);

    for (size_t i = 0;
         i < results.size();
         i++)
    {
        printf("det.cls_id:%d, det.prop:%f\n",
               results[i].cls_id,
               results[i].prop);
    }

    // ========================================================
    // 释放 seg mask
    // ========================================================

    if (od_results.results_seg)
    {
        for (int i = 0;
             i < od_results.count;
             i++)
        {
            if (od_results.results_seg[i].seg_mask)
            {
                free(
                    od_results.results_seg[i].seg_mask);

                od_results.results_seg[i].seg_mask =
                    nullptr;
            }
        }
    }

    // ========================================================
    // 释放输入
    // ========================================================

    free(src_image.virt_addr);

    return !results.empty();
}

// ============================================================
// 新增：带空间位姿采样的推理接口
// ============================================================

bool base_detect_infer(
    const cv::Mat &img,
    std::vector<ObjectCameraDetectResult> &results,
    const Eigen::Vector3d &position,
    const Eigen::Quaterniond &q)
{
    // ========================================================
    // 判断是否需要保存/推理
    // ========================================================

    g_ctx.g_pose_sampler->IsPoseSaveImage =
        g_ctx.g_pose_sampler->NeedSaveFrame(
            position,
            q);
    // 当空间位置状态，不符合图像存储条件时；
    if (!g_ctx.g_pose_sampler->IsPoseSaveImage)
    {
        g_ctx.debuger->setDebugImageSavePath(g_ctx.config.save_debug_images_path);

        printf("skip infer, duplicated pose area\n");
    }
    // 当空间位置状态，符合图像存储条件时；
    else
    {
        g_ctx.debuger->setDebugImageSavePath(g_ctx.config.save_images_path_spatial_location_val);
    }

    // ========================================================
    // 调用原始推理
    // ========================================================
    printf("Calling real_detect_infer\n");
    return real_detect_infer(
        img,
        results);
}

// == == == == == == == == == == == == == == == == == == == == == == == == == == == == == ==
//     模型释放 == == == == == == == == == == == == == == == == == == == == == == == == == == == == == ==

void base_model_release()
{
    if (!g_ctx.initialized)
    {
        return;
    }

    deinit_post_process();

    delete g_ctx.detector;
    delete g_ctx.camera_params;
    delete g_ctx.debuger;
    delete g_ctx.g_pose_sampler;

    g_ctx.detector = nullptr;
    g_ctx.camera_params = nullptr;
    g_ctx.debuger = nullptr;
    g_ctx.g_pose_sampler = nullptr;
    g_ctx.initialized = false;

    printf("carpet_model_release finished\n");
}