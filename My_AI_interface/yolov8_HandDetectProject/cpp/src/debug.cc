#include "debug.h"
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <iomanip>
// #include "utils.hpp"
#include "logs.h"


// #if ANDROID_ENV
//     #include <android/log.h>
//     #define LOG_TAG "HandDetectDebug"
//     #define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
//     #define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
// #else
//     #define LOGI(msg) std::cout << "[INFO] " << msg << std::endl
//     #define LOGE(msg) std::cerr << "[ERROR] " << msg << std::endl

// #endif


// --------------------- 工具：递归创建目录 ---------------------
static bool makeDirsRecursive(const std::string& path)
{
    if (path.empty()) return false;
    if (access(path.c_str(), F_OK) == 0) return true;  // 已经存在

    std::string sub;
    for (size_t i = 1; i < path.size(); ++i)
    {
        if (path[i] == '/')
        {
            sub = path.substr(0, i);
            if (!sub.empty() && access(sub.c_str(), F_OK) != 0)
            {
                if (mkdir(sub.c_str(), 0755) != 0)
                {
                     LOGE("[Debug] Cannot create dir:");
                    return false;
                }
            }
        }
    }

    // 创建最后一级
    if (access(path.c_str(), F_OK) != 0)
    {
        if (mkdir(path.c_str(), 0755) != 0)
        {
            return false;
        }
    }
    LOGI("[Debug] Directory exists or created: %s", path.c_str());
    // LOGI("[Debug] Directory exists or created: " << path.c_str());
    return true;
}

// --------------------- 构造函数 ---------------------
DebugNv21Saver::DebugNv21Saver(const std::string& save_dir)
        : save_dir_(save_dir), frame_count_(0)
{
    if (!ensureDirectory(save_dir_)) {
        std::cerr << "[Debug] Warning: cannot create directory: " << save_dir_ << std::endl;
        LOGI("[Debug] Warning: cannot create directory: " );
    }
}

// --------------------- 目录检查 ---------------------
bool DebugNv21Saver::ensureDirectory(const std::string& dir)
{
    return makeDirsRecursive(dir);
}

// --------------------- 文件名生成 ---------------------
std::string DebugNv21Saver::generateFileName()
{
    std::ostringstream oss;
    // oss << save_dir_
    //     << "/rgb_"
    //     << std::setw(5) << std::setfill('0') << frame_count_
    //     << ".jpg";            // 存储手势检测，结果图片存储路径；

    oss << save_dir_<< "/hand_detect_result.jpg"; // 写死手势检测图片存储名称；

    frame_count_++;
    return oss.str();
}

// --------------------- NV21 → JPG 保存函数 ---------------------
void DebugNv21Saver::saveRgbFrame(const AndroidImageNV21& img)
{
    std::lock_guard<std::mutex> lock(save_mutex_);

    int w = img.image_width;
    int h = img.image_height;

    // -------- 参数校验（关键）--------
    if (!img.image_input_nv21 ||
        w <= 0 || h <= 0 ||
        (w % 2 != 0) || (h % 2 != 0)) {
        LOGE("[Debug] Invalid NV21 input");
        return;
    }

    try {
        // NV21 = Y + VU → 高度 = h * 3 / 2
        cv::Mat yuv(h + h / 2, w, CV_8UC1, img.image_input_nv21);

        if (yuv.empty()) {
            LOGE("[Debug] YUV Mat empty");
            return;
        }

        cv::Mat rgb;
        cv::cvtColor(yuv, rgb, cv::COLOR_YUV2BGR_NV21);

        if (rgb.empty() || rgb.channels() != 3) {
            LOGE("[Debug] cvtColor failed");
            return;
        }

        std::string filename = generateFileName();

        if (!cv::imwrite(filename, rgb)) {
            LOGE("[Debug] imwrite failed: %s", filename.c_str());
            // LOGE("[Debug] imwrite failed: " << filename.c_str());
            return;
        }

        LOGI("[Debug] Saved RGB frame: %s", filename.c_str());
        // LOGI("[Debug] Saved RGB frame: " << filename.c_str());
    }
    catch (const cv::Exception& e) {
        LOGE("[Debug] OpenCV exception: %s", e.what());
        // LOGE("[Debug] OpenCV exception: " << e.what());
        
    }
    catch (const std::exception& e) {
        LOGE("[Debug] std exception: %s", e.what());
        // LOGE("[Debug] std exception: " << e.what());
    }
    catch (...) {
        LOGE("[Debug] Unknown exception in saveRgbFrame");
    }
}

void DebugNv21Saver::saveRgbFrameDetect(image_buffer_t* img, std::vector<object_detect_result>& results)
{

    if (!img || !img->virt_addr) {
        std::cerr << "[Error] saveRgbFrameDetect: invalid image buffer!" << std::endl;
        return;
    }

    std::lock_guard<std::mutex> lock(save_mutex_);

    // 获取图像宽高
    int img_w = img->width;
    int img_h = img->height;

    try {

        char text[256];
        // 遍历检测结果
        for (size_t i = 0; i < results.size(); ++i) {
            const object_detect_result& obj = results[i];

            // 打印检测信息
            std::cout << "Box " << i
                      << ": prop=" << obj.prop
                      << " cls_id=" << obj.cls_id
                      << " left=" << obj.box.left
                      << " top=" << obj.box.top
                      << " right=" << obj.box.right
                      << " bottom=" << obj.box.bottom
                      << std::endl;

            // 安全校验坐标
            int x1 = std::max(0, obj.box.left);
            int y1 = std::max(0, obj.box.top);
            int x2 = std::min(img_w, obj.box.right);
            int y2 = std::min(img_h, obj.box.bottom);

            if (x2 <= x1 || y2 <= y1) {
                continue; // 坐标无效，跳过
            }

            
            // 绘制矩形框
            draw_rectangle(img, x1, y1, x2 - x1, y2 - y1, COLOR_BLUE, 3);
            sprintf(text, "%s %.1f%%", coco_cls_to_name(obj.cls_id), obj.prop * 100);
            draw_text(img, text, x1, y1 - 20, COLOR_RED, 10);



            // 可选：绘制类别和概率
            /*
            char text[64];
            snprintf(text, sizeof(text), "cls:%d prop:%.2f", obj.cls_id, obj.prop);
            draw_text(img, x1, std::max(0, y1 - 5), text, COLOR_RED);
            */
        }
        std::string filename = generateFileName();
        write_image(filename.c_str(), img);

    } catch (const std::exception& e) {
        std::cerr << "[Error] saveRgbFrameDetect exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "[Error] saveRgbFrameDetect unknown exception" << std::endl;
    }
}


void DebugNv21Saver::drawPalmBoxes(cv::Mat& image, const std::vector<object_detect_result>& boxes) {
    const int img_w = image.cols;
    const int img_h = image.rows;

    for (const auto& obj : boxes) {

        // 获取框坐标
        int x1 = std::max(0, obj.box.left);
        int y1 = std::max(0, obj.box.top);
        int x2 = std::min(img_w, obj.box.right);
        int y2 = std::min(img_h, obj.box.bottom);

        std::cout << " x1=" << x1 
                  << " y1=" << y1
                  << " x1=" << x1
                  << " y2=" << y2
                  << std::endl;

        // 安全校验
        if (x2 <= x1 || y2 <= y1) {
            continue;
        }

        cv::Rect rect(x1, y1, x2 - x1, y2 - y1);

        // 绘制矩形框
        cv::rectangle(image, rect, cv::Scalar(0, 255, 0), 2);

        // // 绘制置信度和类别
        // std::ostringstream ss;
        // ss << "cls:" << obj.cls_id 
        //    << " prop:" << std::fixed << std::setprecision(2) << obj.prop;
        
        

        // cv::putText(image, ss.str(), cv::Point(x1, std::max(0, y1 - 5)),
        //             cv::FONT_HERSHEY_SIMPLEX, 0.5,
        //             cv::Scalar(255, 0, 0), 1);
        cv::putText(image, "hand", cv::Point(x1, std::max(0, y1 - 5)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 0, 0), 1);


        // 可选：绘制相机坐标
        /*
        std::ostringstream ss_cam;
        ss_cam << "cam:(" 
               << obj.camera_coordinates.x << "," 
               << obj.camera_coordinates.y << "," 
               << obj.camera_coordinates.z << ")";
        cv::putText(image, ss_cam.str(), cv::Point(x1, std::max(0, y2 + 15)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 255), 1);
        */
    }
}

cv::Mat DebugNv21Saver::plotDetectBoxsMat(cv::Mat & image_mat,
                                          const std::vector<object_detect_result>& boxes)
{
    cv::Mat image = image_mat;

    // 打印结果
    for (size_t i = 0; i < boxes.size(); i++) {
        const object_detect_result& obj = boxes[i];

        std::cout << "Box " << i 
                  << ": prop=" << obj.prop
                  << " cls_id=" << obj.cls_id
                  << " left=" << obj.box.left
                  << " top=" << obj.box.top
                  << " right=" << obj.box.right
                  << " bottom=" << obj.box.bottom
                  << std::endl;

        // // 如果需要打印相机坐标
        // std::cout << "Camera Coordinates: "
        //           << "x=" << obj.camera_coordinates.x
        //           << " y=" << obj.camera_coordinates.y
        //           << " z=" << obj.camera_coordinates.z
        //           << std::endl;

        std::cout << std::endl;
    }

    // 绘制检测结果
    drawPalmBoxes(image, boxes);

    // 可选：显示或保存图像
    // cv::imshow("Palm Detection", image);
    // cv::waitKey(0);
    // cv::imwrite("palm_detect_result.jpg", image);

    return image;
}

