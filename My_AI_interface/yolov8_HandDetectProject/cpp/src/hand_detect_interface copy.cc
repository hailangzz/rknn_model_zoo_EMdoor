#include "hand_detect_interface.h"
#include "yolov8_detect.h"
#include "debug.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include "image_drawing.h"
#include "event_control.h"
#include <mutex>


#define ANDROID_ENV 0

#if ANDROID_ENV
    #include <android/log.h>
    #define LOG_TAG "HandDetectNative"
    #define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
    #define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
    #define LOGI(...) std::cout << "[INFO] " << __VA_ARGS__ << std::endl
    #define LOGE(msg) std::cerr << "[ERROR] " << msg << std::endl

#endif


static std::string& getConfigPath() {
    static std::string config_path = "./config/cfg.txt";  // 默认路径
    return config_path;
}

// 配置读取函数
static ConfigInfo& getConfig()
{
    static ConfigInfo config = readConfig(getConfigPath().c_str());
    return config;
}

// 给调用者的接口：设置配置文件路径
void hand_detect_set_config_path(const std::string& path) {
    getConfigPath() = path;
}

static Detector& getDetector()
{
    static Detector detector(getConfig());
    return detector;
}

static HandDetectStateController& getState()
{
    static HandDetectStateController handDetectState(getConfig().max_frame_threshold);
    return handDetectState;
}

// 摄像头数据存储模块
static DebugNv21Saver& getDebugSaver() {
    static DebugNv21Saver saver(getConfig().debug_nv21_image_saver);
    return saver;
}


// 锁：确保线程安全
static std::mutex detector_mutex;  // 用于保护 Detector
static std::mutex state_mutex;     // 用于保护 HandDetectStateController
static std::mutex debug_mutex;     // 用于保护 DebugNv21Saver

struct timeval start_time, stop_time;

HandDetectResult hand_detect_interface(AndroidImageNV21* image_object_input,
                                       bool is_save_images) {
    try {
        // 1. 参数检查
        if (!image_object_input || !image_object_input->image_input_nv21) {
            LOGE("Invalid NV21 image input!");
            return HandDetectResult::NoHand;
        }

        if (image_object_input->image_width <= 0 ||
            image_object_input->image_height <= 0) {
            LOGE("Invalid image size!");
            return HandDetectResult::NoHand;
        }


        Detector& detector = getDetector();
        HandDetectStateController& state = getState();

        // 使用锁保护推理过程
        std::lock_guard<std::mutex> lock(detector_mutex);

        bool is_exist_hand = detector.infer_nv21_image_data(
            image_object_input->image_input_nv21,
            image_object_input->image_width,
            image_object_input->image_height
        );
        
        // 3. 保存结果：确保线程安全
        if (is_save_images) {
            std::lock_guard<std::mutex> debug_lock(debug_mutex);
            getDebugSaver().saveRgbFrameDetect(*image_object_input, detector.object_detect_result_list_);
        }

        // //只保存摄像头输入图片，用来采集数据：
        // std::vector<PalmBox> results;
        // std::lock_guard<std::mutex> debug_lock(debug_mutex);
        // getDebugSaver().saveRgbFrameDetect(*image_object_input, results);

        // 4. 更新状态：确保线程安全
        bool detected = is_exist_hand;
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            state.update(detected);
        }


        return detected ? HandDetectResult::HandDetected
                        : HandDetectResult::NoHand;
    }
    catch (const std::exception& e) {
        LOGE("Exception in hand_detect_interface: " << e.what());
        return HandDetectResult::NoHand;
    }
    catch (...) {
        LOGE("Unknown exception in hand_detect_interface");
        return HandDetectResult::NoHand;
    }
}



