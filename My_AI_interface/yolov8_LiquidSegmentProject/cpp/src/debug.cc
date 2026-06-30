#include "debug.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

#include <iomanip>
#include <iostream>

// =====================================================
// 递归创建目录（mkdir -p）
// =====================================================

static bool createDirectories(
    const std::string &path)
{
    if (path.empty())
    {
        return false;
    }

    std::string current;

    for (size_t i = 0; i < path.size(); ++i)
    {
        current += path[i];

        // 遇到 '/' 或最后一个字符
        if (path[i] == '/' ||
            i == path.size() - 1)
        {
            // 跳过空路径
            if (current.empty())
            {
                continue;
            }

            struct stat st;

            // 不存在则创建
            if (stat(current.c_str(), &st) != 0)
            {
                if (mkdir(current.c_str(), 0777) != 0)
                {
                    // 已存在不算失败
                    if (errno != EEXIST)
                    {
                        std::cerr
                            << "[Debug] mkdir failed: "
                            << current
                            << " errno="
                            << errno
                            << std::endl;

                        return false;
                    }
                }
            }
        }
    }

    return true;
}

// =====================================================
// Constructor
// =====================================================

Debug::Debug(
    const std::string &debug_image_save_path,
    bool is_save_debug_image,
    int fps_limit)
    : debug_image_save_path_(debug_image_save_path),
      is_save_debug_image_(is_save_debug_image),
      fps_limit_(fps_limit)
{
    // 自动创建目录
    if (!createDirectories(debug_image_save_path_))
    {
        std::cerr
            << "[Debug] create dir failed: "
            << debug_image_save_path_
            << std::endl;
    }
    else
    {
        std::cout
            << "[Debug] save path: "
            << debug_image_save_path_
            << std::endl;
    }

    // FPS限频
    if (fps_limit_ > 0)
    {
        save_interval_ms_ =
            1000 / fps_limit_;
    }
    else
    {
        save_interval_ms_ = 0;
    }
}

// =====================================================
// 生成文件名
// =====================================================

std::string Debug::generateFileName(
    const std::string &tag)
{
    auto now =
        std::chrono::system_clock::now();

    auto time_t_now =
        std::chrono::system_clock::to_time_t(now);

    auto ms =
        std::chrono::duration_cast<
            std::chrono::milliseconds>(
            now.time_since_epoch()) %
        1000;

    std::tm tm_time;

    localtime_r(
        &time_t_now,
        &tm_time);

    std::ostringstream oss;

    oss << debug_image_save_path_ << "/";

    oss << std::put_time(
        &tm_time,
        "%Y%m%d_%H%M%S");

    oss << "_"
        << std::setw(3)
        << std::setfill('0')
        << ms.count();

    if (!tag.empty())
    {
        oss << "_"
            << tag;
    }

    oss << ".jpg";

    return oss.str();
}

// =====================================================
// 保存图片
// =====================================================

void Debug::saveIfDetected(
    const cv::Mat &image,
    const std::string &tag)
{
    // 开关关闭
    if (!is_save_debug_image_)
    {
        return;
    }

    // 空图
    if (image.empty())
    {
        std::cerr
            << "[Debug] image empty"
            << std::endl;

        return;
    }

    // =================================================
    // 限频
    // =================================================

    int64_t now_ms =
        std::chrono::duration_cast<
            std::chrono::milliseconds>(
            std::chrono::steady_clock::now()
                .time_since_epoch())
            .count();

    int64_t last_ms =
        last_save_time_ms_.load();

    if (save_interval_ms_ > 0 &&
        now_ms - last_ms < save_interval_ms_)
    {
        return;
    }

    // 更新时间
    last_save_time_ms_.store(now_ms);

    // =================================================
    // 连续内存检查
    // =================================================

    if (!image.isContinuous())
    {
        std::cerr
            << "[Debug] image not continuous"
            << std::endl;

        return;
    }

    // =================================================
    // 线程锁
    // =================================================

    std::lock_guard<std::mutex> lock(mutex_);

    // =================================================
    // 文件名
    // =================================================

    std::string filename =
        generateFileName(tag);

    // =================================================
    // RK image buffer
    // =================================================

    image_buffer_t buf;

    memset(
        &buf,
        0,
        sizeof(buf));

    buf.width = image.cols;

    buf.height = image.rows;

    buf.format = IMAGE_FORMAT_RGB888;

    buf.size =
        image.total() *
        image.elemSize();

    buf.virt_addr =
        (unsigned char *)image.data;

    // =================================================
    // 保存
    // =================================================

    int ret =
        write_image(
            filename.c_str(),
            &buf);

    if (ret != 0)
    {
        std::cerr
            << "[Debug] save failed: "
            << filename
            << std::endl;
    }
    else
    {
        std::cout
            << "[Debug] saved: "
            << filename
            << std::endl;
    }
}

void Debug::saveIfDetectedAddContours(
    const cv::Mat &image,
    const std::vector<std::vector<cv::Point>> &contours,
    const std::vector<int> &cls_ids,
    const std::string &tag)
{
    //----------------------------------
    // 开关检查
    //----------------------------------

    if (!is_save_debug_image_)
    {
        return;
    }

    //----------------------------------
    // 图像检查
    //----------------------------------

    if (image.empty())
    {
        std::cerr
            << "[Debug] image empty"
            << std::endl;

        return;
    }

    //----------------------------------
    // 文件名
    //----------------------------------

    std::string image_filename =
        generateFileName(tag);

    std::string label_filename =
        image_filename.substr(
            0,
            image_filename.find_last_of('.')) +
        ".txt";

    //----------------------------------
    // 保存图片
    //----------------------------------

    cv::Mat draw_img = image.clone();

    image_buffer_t buf;

    memset(
        &buf,
        0,
        sizeof(buf));

    buf.width =
        draw_img.cols;

    buf.height =
        draw_img.rows;

    buf.format =
        IMAGE_FORMAT_RGB888;

    buf.size =
        draw_img.total() *
        draw_img.elemSize();

    buf.virt_addr =
        (unsigned char *)draw_img.data;

    int ret =
        write_image(
            image_filename.c_str(),
            &buf);

    if (ret != 0)
    {
        std::cerr
            << "[Debug] save image failed: "
            << image_filename
            << std::endl;

        return;
    }

    //----------------------------------
    // 创建标签文件
    //----------------------------------

    std::ofstream ofs(label_filename);

    if (!ofs.is_open())
    {
        std::cerr
            << "[Debug] open label file failed: "
            << label_filename
            << std::endl;

        return;
    }

    //----------------------------------
    // contours为空
    // 保存空标签文件
    //----------------------------------

    if (contours.empty())
    {
        ofs.close();

        std::cout
            << "[Debug] image saved: "
            << image_filename
            << std::endl;

        std::cout
            << "[Debug] empty label saved: "
            << label_filename
            << std::endl;

        return;
    }

    //----------------------------------
    // 数量检查
    //----------------------------------

    if (contours.size() != cls_ids.size())
    {
        std::cerr
            << "[Debug] contours size("
            << contours.size()
            << ") != cls_ids size("
            << cls_ids.size()
            << ")"
            << std::endl;

        ofs.close();

        return;
    }

    //----------------------------------
    // 图像尺寸
    //----------------------------------

    const float img_w =
        static_cast<float>(image.cols);

    const float img_h =
        static_cast<float>(image.rows);

    //----------------------------------
    // 写入YOLOv8 Seg标签
    //----------------------------------

    for (size_t i = 0;
         i < contours.size();
         i++)
    {
        const auto &contour =
            contours[i];

        //----------------------------------
        // 至少需要3个点
        //----------------------------------

        if (contour.size() < 3)
        {
            continue;
        }

        //----------------------------------
        // class id
        //----------------------------------

        ofs << cls_ids[i];

        //----------------------------------
        // polygon points
        //----------------------------------

        for (const auto &pt : contour)
        {
            float x =
                static_cast<float>(pt.x) /
                img_w;

            float y =
                static_cast<float>(pt.y) /
                img_h;

            x = std::max(
                0.0f,
                std::min(1.0f, x));

            y = std::max(
                0.0f,
                std::min(1.0f, y));

            ofs << " "
                << std::fixed
                << std::setprecision(6)
                << x
                << " "
                << y;
        }

        ofs << "\n";
    }

    ofs.close();

    //----------------------------------
    // 日志
    //----------------------------------

    std::cout
        << "[Debug] image saved: "
        << image_filename
        << std::endl;

    std::cout
        << "[Debug] seg label saved: "
        << label_filename
        << std::endl;
}
