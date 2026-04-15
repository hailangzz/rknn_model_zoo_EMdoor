#include "debug.h"


Debug::Debug(const std::string& debug_image_save_path, bool is_save_debug_image)
    : debug_image_save_path_(debug_image_save_path), is_save_debug_image_(is_save_debug_image)
{
    // 创建目录（如果不存在）
    struct stat st;
    if (stat(debug_image_save_path_.c_str(), &st) != 0) {
        mkdir(debug_image_save_path_.c_str(), 0777);
    }
}

std::string Debug::generateFileName(const std::string& tag)
{
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) % 1000;

    std::tm tm_time;
    localtime_r(&time_t_now, &tm_time);

    std::ostringstream oss;
    oss << debug_image_save_path_ << "/";

    oss << std::put_time(&tm_time, "%Y%m%d_%H%M%S");
    oss << "_" << ms.count();

    if (!tag.empty()) {
        oss << "_" << tag;
    }

    oss << ".jpg";

    return oss.str();
}

// void Debug::saveIfDetected(const cv::Mat& image, const std::string& tag)
// {

//     if (!is_save_debug_image_ || image.empty()) {
//         return;
//     }

//     std::lock_guard<std::mutex> lock(mutex_);

//     std::string filename = generateFileName(tag);

//     try {
//         cv::imwrite(filename, image);
//         std::cout << "[DebugImageSaver] saved: " << filename << std::endl;
//     }
//     catch (const std::exception& e) {
//         std::cerr << "[DebugImageSaver] save failed: " << e.what() << std::endl;
//     }
// }

void Debug::saveIfDetected(const cv::Mat& image, const std::string& tag)
{
    if (!is_save_debug_image_ || image.empty()) return;

    if (!image.isContinuous()) {
        std::cerr << "[Debug] image not continuous!" << std::endl;
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    std::string filename = generateFileName(tag);

    image_buffer_t buf;
    memset(&buf, 0, sizeof(buf));

    buf.width = image.cols;
    buf.height = image.rows;
    buf.format = IMAGE_FORMAT_RGB888;
    buf.size = image.total() * image.elemSize();
    buf.virt_addr = (unsigned char*)image.data;

    write_image(filename.c_str(), &buf);
}