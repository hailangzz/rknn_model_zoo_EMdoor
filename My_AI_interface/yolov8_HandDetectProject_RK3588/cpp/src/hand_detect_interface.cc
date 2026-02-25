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
    LinuxImageRGB888* image_object_input,
    bool is_save_images
) {
    try {
        if (!image_object_input || image_object_input->image_input_rgb888 == nullptr || image_object_input->image_width <= 0 || image_object_input->image_height <= 0) {
            LOGE("Invalid RGB image input!");
            return HandDetectResult::NoHand;
        }

        size_t rgb_size = (size_t)image_object_input->image_width * image_object_input->image_height * 3;

        Detector& detector = getDetector();
        LOGI("detector = getDetector()  is ok\n");
        HandDetectStateController& state = getState();
        LOGI("state = getState()  is ok\n");
        bool hand_detect_event_result = false;   // 手势识别结果

        // 构建 image_buffer_t
        image_buffer_t img_buf;
        memset(&img_buf, 0, sizeof(img_buf));
        img_buf.width  = image_object_input->image_width;
        img_buf.height = image_object_input->image_height;
        img_buf.format = IMAGE_FORMAT_RGB888;
        img_buf.width_stride  = image_object_input->image_width * 3;
        img_buf.height_stride = image_object_input->image_height;
        img_buf.size        = rgb_size;
        img_buf.virt_addr   = image_object_input->image_input_rgb888;
        img_buf.fd          = -1;

        bool is_exist_hand = false;
        {
            std::lock_guard<std::mutex> lock(detector_mutex);
            // 直接传入 RGB 数据
            is_exist_hand = detector.infer_nv21_image_data(img_buf);  // 你 infer 函数可以改名为 infer_rgb_image_data
        }
        LOGI("infer_rgb_image_data is ok\n");

        if (is_save_images) {
            std::lock_guard<std::mutex> debug_lock(debug_mutex);
            getDebugSaver().saveRgbFrameDetect(
                &img_buf,
                detector.object_detect_result_list_
            );
        }
        LOGI("saveRgbFrameDetect is ok\n");

        // 货架平台区域内，人手有效性检查
        std::vector<object_detect_result> filtered_results;
        {
            std::lock_guard<std::mutex> lock(detector_mutex); // 保护 object_detect_result_list_
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

        LOGI("filtered_results.size = %zu", filtered_results.size());
        for (size_t i = 0; i < filtered_results.size(); ++i)
        {
            const auto& box = filtered_results[i].box;
            LOGI("box[%zu]: L=%d T=%d R=%d B=%d",
                i, box.left, box.top, box.right, box.bottom);
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            hand_detect_event_result = state.update(!filtered_results.empty());
        }

        LOGI("HandDetectState update: event_state=%d", hand_detect_event_result);

        // 不再 malloc/free，因为 RGB 是外部传入
        // img_buf.virt_addr = rgb_image_input

        return hand_detect_event_result
               ? HandDetectResult::HandDetected
               : HandDetectResult::NoHand;
    }

    catch (...) {
        LOGE("Exception in hand_detect_interface_rgb");
        return HandDetectResult::NoHand;
    }
}


void print_version() {
    std::cout << "version info: HandDetect:1.0.0  2026-02-02_21-56-35"<< std::endl;
}

