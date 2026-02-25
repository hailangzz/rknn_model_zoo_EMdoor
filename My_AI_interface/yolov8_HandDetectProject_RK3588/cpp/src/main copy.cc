#include <iostream>
#include <sys/time.h>
#include "hand_detect_interface.h"  // 上面写好的 Detector 类
#include "event_control.h"


#define YUV420_NV21 1

AndroidImageNV21 load_image_to_nv21(const std::string& image_path) {
    AndroidImageNV21 result;
    result.image_input_nv21 = nullptr;
    result.image_width = 0;
    result.image_height = 0;

    // 1. 读取图片
    cv::Mat bgr = cv::imread(image_path, cv::IMREAD_COLOR);
    if (bgr.empty()) {
        std::cerr << "Failed to load image: " << image_path << std::endl;
        return result;
    }

    // 2. 保证宽高为偶数
    int width = bgr.cols & ~1;    // 向下取偶数
    int height = bgr.rows & ~1;
    cv::Mat bgr_even = bgr(cv::Rect(0, 0, width, height));

    // 3. 转为 I420（YUV420p）
    cv::Mat yuv420p;
    cv::cvtColor(bgr_even, yuv420p, cv::COLOR_BGR2YUV_I420);

    // 4. 分配 NV21 内存
    size_t nv21_size = width * height * 3 / 2;
    uint8_t* nv21_buf = new uint8_t[nv21_size];

    // 5. 复制 Y 分量
    memcpy(nv21_buf, yuv420p.data, width * height);

    // 6. 交错 VU
    uint8_t* src_u = yuv420p.data + width * height;
    uint8_t* src_v = src_u + (width * height) / 4;
    uint8_t* dst_uv = nv21_buf + width * height;

    for (int i = 0; i < (width * height) / 4; i++) {
        dst_uv[i * 2] = src_v[i];     // V
        dst_uv[i * 2 + 1] = src_u[i]; // U
    }

    // 7. 填充返回结构体
    result.image_input_nv21 = nv21_buf;
    result.image_width = width;
    result.image_height = height;

    return result;
}


int main(int argc, char** argv) {
    struct timeval start_time, stop_time;

    const std::string config_path = "./config/cfg.txt";
    hand_detect_set_config_path(config_path);


    std::string image_path = "/home/robot/zhangzhuo/rknn_yolov8_HandDetectProject_demo/model/hand.png";

    // 读取图片测试
    cv::Mat test = cv::imread(image_path, cv::IMREAD_COLOR);
    if (test.empty()) {
        std::cerr << "Cannot open image at: " << image_path << std::endl;
        return -1;
    }

    // 计算耗时
    gettimeofday(&start_time, NULL);

    # if YUV420_NV21
        AndroidImageNV21 image_input;
        
        image_input = load_image_to_nv21(image_path);

        bool is_save_images = true;
        HandDetectResult results = hand_detect_interface(&image_input,is_save_images);

    # else
        // std::vector<PalmBox>  results = detector.infer_image_rga_zero_copy(image_path);
        // 1. 读取图片
        cv::Mat img = cv::imread(image_path);
        if (img.empty()) {
            std::cerr << "Failed to read image: " << image_path << std::endl;
            return -1;
        }
        // 2. 预处理
        std::vector<float> input_data = preprocess_image(img, config_info.input_width, config_info.input_height);
        std::vector<PalmBox>  results = detector.infer(input_data);
    
    #endif

    


    return 0;
}
