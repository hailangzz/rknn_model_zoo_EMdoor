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
        printf("glassobstacle model already initialized\n");
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

    printf("glassobstacle_model_init success, max_detect_distance_m=%.2f, sizeof(ObjectCameraDetectResult)=%zu\n",
           g_ctx.config.max_detect_distance_m,
           sizeof(ObjectCameraDetectResult));
    return true;
}

size_t base_detect_result_sizeof()
{
    return sizeof(ObjectCameraDetectResult);
}

// ============================================================
// 原始推理接口
// ============================================================
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

    // 若节点与 .so 使用了不同版本的 ObjectCameraDetectResult，push_back 会直接把堆写坏
    // 这里无法读取“调用方 sizeof”，只能在节点侧用 base_detect_result_sizeof() 比对

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

        // 置信度阈值过滤
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

        if (!seg || !seg->seg_mask)
        {
            printf("seg ptr / mask invalid!\n");
            continue;
        }

        ObjectCameraDetectResult one;

        // ====================================================
        // 稳定底边：框内从框底向上扫 mask（空隙上沿 = mark 下边缘）
        // 必须在 extract_seg_mask_contours 释放 mask 之前完成
        // ====================================================

        std::vector<cv::Point> bottom_line_px;
        if (!extractStableGlassBottomLineFromMask(
                seg->seg_mask,
                src_image.width,
                src_image.height,
                det->box,
                g_ctx.config.bottom_line_sample_step_px,
                g_ctx.config.bottom_line_median_k,
                g_ctx.config.bottom_line_fit_straight,
                g_ctx.config.bottom_line_fit_sample_num,
                bottom_line_px))
        {
            printf("[filter] reject: failed to extract stable bottom line, prop=%.3f\n",
                   det->prop);
            // 释放本目标 mask，避免泄漏
            if (seg->seg_mask)
            {
                free(seg->seg_mask);
                seg->seg_mask = nullptr;
            }
            continue;
        }

        // 底边已取出，释放 mask（建图只需要稳定底边线）
        if (seg->seg_mask)
        {
            free(seg->seg_mask);
            seg->seg_mask = nullptr;
        }

        const bool grounded = isGlassBottomGroundContact(
            det->box,
            src_image.height,
            g_ctx.config.floating_bottom_gap_ratio);

        float plane_y = 0.0f;
        std::vector<CameraCoordinate> raw_line_xyz;
        if (!g_ctx.camera_params->projectGlassBottomLineAuto(
                bottom_line_px,
                g_ctx.config,
                grounded,
                raw_line_xyz,
                &plane_y))
        {
            printf("[filter] reject: bottom line 3D projection failed (grounded=%d, box_bottom=%d, img_h=%d, line_px=%zu)\n",
                   grounded ? 1 : 0,
                   det->box.bottom,
                   src_image.height,
                   bottom_line_px.size());
            continue;
        }

        std::vector<CameraCoordinate> bottom_line_xyz;
        float line_width_m = -1.0f;
        float nearest_m = -1.0f;
        if (!sanitizeGlassBottomLineXYZ(
                raw_line_xyz,
                g_ctx.config,
                bottom_line_xyz,
                &line_width_m,
                &nearest_m))
        {
            printf("[filter] reject: sanitize bottom line failed (raw=%zu)\n",
                   raw_line_xyz.size());
            continue;
        }

        // ====================================================
        // 填充结果：建图用底边三维线；可视化用二维底边折线
        // ====================================================

        one.prop = det->prop;
        one.cls_id = g_ctx.config.BASE_AREA;
        one.target_box.left = det->box.left;
        one.target_box.top = det->box.top;
        one.target_box.right = det->box.right;
        one.target_box.bottom = det->box.bottom;

        one.add_edge_point_single_pixel_camera_coordinates = bottom_line_xyz;
        one.object_contours_mark_point = {bottom_line_px};

        // coords：左/右端点（兼容旧字段；不再输出不可靠的上沿角点）
        one.coords.assign(4, CameraCoordinate{0.f, 0.f, 0.f});
        if (!bottom_line_xyz.empty())
        {
            one.coords[0] = bottom_line_xyz.front();
            one.coords[1] = bottom_line_xyz.back();
            one.coords[2] = bottom_line_xyz.back();
            one.coords[3] = bottom_line_xyz.front();
        }

        // ====================================================
        // 几何后处理过滤
        // ====================================================

        float nearest_check = -1.0f;
        if (!isGlassObstacleValid(one, g_ctx.config, &nearest_check))
        {
            continue;
        }

        printf("[glass] line_pts=%zu width=%.3fm nearest=%.3fm grounded=%d plane_y=%.3f\n",
               bottom_line_xyz.size(),
               line_width_m,
               nearest_check,
               grounded ? 1 : 0,
               plane_y);

        results.push_back(std::move(one));

        debug_all_contours.push_back(bottom_line_px);
        debug_all_cls_ids.push_back(det->cls_id);
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

    printf("od_results.count: %d, kept: %zu\n",
           od_results.count,
           results.size());

    for (size_t i = 0;
         i < results.size();
         i++)
    {
        const float nearest = getNearestDetectDistance(results[i]);
        ObjectSize3D size{};
        const float width = calcObjectSizeByAverage(results[i], size) ? size.width : -1.0f;
        printf("det.cls_id:%d, det.prop:%f, nearest:%.3fm, width:%.3fm\n",
               results[i].cls_id,
               results[i].prop,
               nearest,
               width);
    }

    // ========================================================
    // 释放 seg mask（extract 可能已释放，这里兜底）
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
            releaseDetectEdgePoints(&od_results.results[i]);
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

    printf("glassobstacle_model_release finished\n");
}
