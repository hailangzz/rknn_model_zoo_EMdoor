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

// #define ANDROID_ENV 1   // 0: Linux, 1: Android

// #if ANDROID_ENV
//     #include <android/log.h>
//     #define LOG_TAG "HandDetectNative"
//     #define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
//     #define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
// #else
//     // Linux 下 printf 风格宏
//     #define LOGI(fmt, ...) \
//         do { \
//             char buf[512]; \
//             snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__); \
//             std::cout << "[INFO] " << buf << std::endl; \
//         } while(0)

//     #define LOGE(fmt, ...) \
//         do { \
//             char buf[512]; \
//             snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__); \
//             std::cerr << "[ERROR] " << buf << std::endl; \
//         } while(0)
// #endif

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
// HandDetectResult hand_detect_interface(AndroidImageNV21* image_object_input,
//                                        bool is_save_images) {
//     try {
//         // 1. 参数检查
//         if (!image_object_input || !image_object_input->image_input_nv21) {
//             LOGE("Invalid NV21 image input!");
//             return HandDetectResult::NoHand;
//         }

//         if (image_object_input->image_width <= 0 ||
//             image_object_input->image_height <= 0) {
//             LOGE("Invalid image size!");
//             return HandDetectResult::NoHand;
//         }

//         Detector& detector = getDetector();
//         HandDetectStateController& state = getState();

//         image_buffer_t img_buf;
//         // 数据转换 (linux 可用)
//         wrap_nv21_to_image_buffer(image_object_input->image_input_nv21, image_object_input->image_width, image_object_input->image_height, &img_buf);
//         // wrap_nv21_to_image_buffer_safe(image_object_input->image_input_nv21, image_object_input->image_width, image_object_input->image_height, &img_buf);

//         // write_image("wrap_nv21_to_image_buffer.png", &img_buf);
//         // 2. 使用锁保护推理过程
//         std::lock_guard<std::mutex> lock(detector_mutex);
//         // bool is_exist_hand = detector.infer_nv21_image_data(
//         //     image_object_input->image_input_nv21,
//         //     image_object_input->image_width,
//         //     image_object_input->image_height
//         // );
//         bool is_exist_hand = detector.infer_nv21_image_data(img_buf);
        
        
//         // 3. 保存结果：确保线程安全
//         if (is_save_images) {
//             std::lock_guard<std::mutex> debug_lock(debug_mutex);
//             getDebugSaver().saveRgbFrameDetect(&img_buf, detector.object_detect_result_list_);
//         }

//         // 4. 更新状态：确保线程安全
//         {
//             std::lock_guard<std::mutex> lock(state_mutex);
//             state.update(is_exist_hand);
//         }

//         return is_exist_hand ? HandDetectResult::HandDetected
//                              : HandDetectResult::NoHand;
//     }
//     catch (const std::exception& e) {
//         LOGE("Exception in hand_detect_interface: %s", e.what());
//         return HandDetectResult::NoHand;
//     }
//     catch (...) {
//         LOGE("Unknown exception in hand_detect_interface");
//         return HandDetectResult::NoHand;
//     }
// }


// ====================== 手检测接口 ======================
HandDetectResult hand_detect_interface(
    AndroidImageNV21* image_object_input,
    bool is_save_images
) {

    try {
        if (!image_object_input ||
            !image_object_input->image_input_nv21) {
            LOGE("Invalid NV21 image input!");
            return HandDetectResult::NoHand;
        }

        int width  = image_object_input->image_width;
        int height = image_object_input->image_height;

        if (width <= 0 || height <= 0) {
            LOGE("Invalid image size: %dx%d", width, height);
            return HandDetectResult::NoHand;
        }

        // ✅ 正确获取 NV21 size
        size_t nv21_size = (size_t)width * height * 3 / 2;

        LOGI("size_t nv21_size  is ok\n");

        Detector& detector = getDetector();
        LOGI("detector = getDetector()  is ok\n");
        HandDetectStateController& state = getState();
        LOGI("state = getState()  is ok\n");
        bool hand_detect_event_result = false;   // 手势识别，结果状态值

        image_buffer_t img_buf;
        int ok = wrap_nv21_to_image_buffer_safe(
            image_object_input->image_input_nv21,
            nv21_size,
            width,
            height,
            &img_buf
        );
        LOGI("wrap_nv21_to_image_buffer_safe is ok\n");

        if (!ok) {
            LOGE("wrap_nv21_to_image_buffer_safe failed");
            return HandDetectResult::NoHand;
        }

        bool is_exist_hand = false;
        {
            std::lock_guard<std::mutex> lock(detector_mutex);
            is_exist_hand = detector.infer_nv21_image_data(img_buf);
        }
        LOGI("infer_nv21_image_data is ok\n");

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
            for (const auto& obj : detector.object_detect_result_list_)
            {
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

        if (img_buf.virt_addr) {
            free(img_buf.virt_addr);
            img_buf.virt_addr = NULL;
        }
        

        // return is_exist_hand
        //        ? HandDetectResult::HandDetected
        //        : HandDetectResult::NoHand;
        return hand_detect_event_result
               ? HandDetectResult::HandDetected
               : HandDetectResult::NoHand;
    }

    catch (...) {
        LOGE("Exception in hand_detect_interface");
        return HandDetectResult::NoHand;
    }
}

void print_version() {
    std::cout << "version info: HandDetect:1.0.0  2026-02-02_21-56-35"<< std::endl;
}

