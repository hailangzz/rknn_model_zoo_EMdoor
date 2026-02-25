#include "hand_detect_interface.h"
#include "yolov8_detect.h"

#include "debug.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include "image_drawing.h"
#include "event_control.h"
#include "logs.h"

#include <mutex>
#include <cstdio>    // 用于 snprintf
#include <iostream>  // std::cout, std::cerr


// ====================== 配置路径 & 读取 ======================
static std::string& getConfigPath() {
    static std::string config_path = "./config/cfg.txt";  // 默认路径
    return config_path;
}

static ConfigInfo& getConfig()
{
    static ConfigInfo config = readConfig(getConfigPath().c_str());
    return config;
}

void hand_detect_set_config_path(const std::string& path) {
    getConfigPath() = path;

}

// ====================== 单例模块 ======================
static Detector& getDetector()
{
    static Detector detector(getConfig());
    return detector;
}

static HandDetectStateController& getState()
{
    static HandDetectStateController handDetectState(getConfig().max_frame_threshold,getConfig().max_detect_duration_s,getConfig().block_duration_s);
    return handDetectState;
}

static DebugNv21Saver& getDebugSaver() {
    static DebugNv21Saver saver(getConfig().debug_nv21_image_saver);
    return saver;
}

// ====================== 线程安全锁 ======================
static std::mutex detector_mutex;  // 用于保护 Detector
static std::mutex state_mutex;     // 用于保护 HandDetectStateController
static std::mutex debug_mutex;     // 用于保护 DebugNv21Saver

struct timeval start_time, stop_time;

// ====================== 手检测接口 ======================

HandDetectResult hand_detect_interface(
    std::shared_ptr<cv::Mat> image_object_input,
    bool is_save_images
) {
    try {        
        // ---------- 1. 输入检查 ----------
        if (!image_object_input || image_object_input->empty()) {
            LOGE("Invalid Mat input!");
            return HandDetectResult::NoHand;
        }

        if (image_object_input->type() != CV_8UC3) {
            LOGE("Input Mat must be CV_8UC3 (RGB888)");
            return HandDetectResult::NoHand;
        }

        int width  = image_object_input->cols;
        int height = image_object_input->rows;
        size_t rgb_size = (size_t)width * height * 3;

        Detector& detector = getDetector();
        HandDetectStateController& state = getState();
        bool hand_detect_event_result = false;

        // ---------- 2. 构建 image_buffer_t ----------
        image_buffer_t img_buf{};
        img_buf.width  = width;
        img_buf.height = height;
        img_buf.format = IMAGE_FORMAT_RGB888;
        img_buf.width_stride  = width * 3;
        img_buf.height_stride = height;
        img_buf.size      = rgb_size;
        img_buf.virt_addr = image_object_input->data;  // ⭐ 共享 Mat 内存
        img_buf.fd        = -1;

        bool is_exist_hand = false;

        {
            std::lock_guard<std::mutex> lock(detector_mutex);
            is_exist_hand = detector.infer_nv21_image_data(img_buf); 
            // 👉 建议改名 infer_rgb_image_data
        }

        if (is_save_images) {
            std::lock_guard<std::mutex> debug_lock(debug_mutex);
            getDebugSaver().saveRgbFrameDetect(
                &img_buf,
                detector.object_detect_result_list_
            );
        }

        // ---------- 3. 区域过滤 ----------
        std::vector<object_detect_result> filtered_results;
        {
            std::lock_guard<std::mutex> lock(detector_mutex);
            for (const auto& obj : detector.object_detect_result_list_) {
                const image_rect_t& box = obj.box;

                bool valid = bboxEllipseOverlapRatio(
                    box.left, box.top, box.right, box.bottom,
                    getConfig().center_x,
                    getConfig().center_y,
                    getConfig().axes_w,
                    getConfig().axes_h,
                    getConfig().target_effective_area_iou_thread,
                    20
                );

                if (valid)
                    filtered_results.push_back(obj);
            }
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            hand_detect_event_result = state.update(!filtered_results.empty());
        }

        return hand_detect_event_result
               ? HandDetectResult::HandDetected
               : HandDetectResult::NoHand;
    }
    catch (...) {
        LOGE("Exception in hand_detect_interface(Mat)");
        return HandDetectResult::NoHand;
    }
}


void print_version() {
    std::cout << "version info: HandDetect:1.0.0  2026-02-02_21-56-35"<< std::endl;
}

