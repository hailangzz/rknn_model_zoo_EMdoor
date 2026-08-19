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

std::string Debug::getDeviceSN(const std::string &file_path)
{
    std::ifstream ifs(file_path);

    if (!ifs.is_open())
    {
        return "";
    }

    std::string line;

    while (std::getline(ifs, line))
    {
        auto pos = line.find("\"sn\"");

        if (pos != std::string::npos)
        {
            // 从 "sn" 的位置开始寻找冒号
            auto colon = line.find(":", pos);

            if (colon == std::string::npos)
            {
                continue;
            }

            auto first_quote = line.find("\"", colon + 1);

            auto second_quote = line.find("\"", first_quote + 1);

            if (first_quote != std::string::npos &&
                second_quote != std::string::npos)
            {
                return line.substr(
                    first_quote + 1,
                    second_quote - first_quote - 1);
            }
        }
    }

    return "";
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

    // 获取设备SN码
    device_id_sn =
        getDeviceSN(produce_info_path_);
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

void Debug::setDebugImageSavePath(const std::string &path)
{
    std::lock_guard<std::mutex> lock(mutex_);

    debug_image_save_path_ = path;

    if (!createDirectories(debug_image_save_path_))
    {
        std::cerr
            << "[Debug] create dir failed: "
            << debug_image_save_path_
            << std::endl;
    }
}

void Debug::updateSavedPoseImageCount(TargetStatus status)
{
    std::lock_guard<std::mutex> lock(mutex_);

    switch (status)
    {
    case TargetStatus::EXISTS:
        saved_pose_exist_target_image_count_++;
        break;

    case TargetStatus::MIDDLE:
        // 可选：单独统计 middle
        saved_pose_middle_target_image_count_++;
        break;

    case TargetStatus::NONE:
        saved_pose_null_target_image_count_++;
        break;

    default:
        break;
    }
}

int Debug::getSavedPoseImageCount(TargetStatus status)
{
    std::lock_guard<std::mutex> lock(mutex_);

    switch (status)
    {
    case TargetStatus::EXISTS:
        return saved_pose_exist_target_image_count_;

    case TargetStatus::MIDDLE:
        return saved_pose_middle_target_image_count_;

    case TargetStatus::NONE:
        return saved_pose_null_target_image_count_;

    default:
        return 0;
    }
}

std::string Debug::buildSaveDirectory(
    const std::string &task_name,
    TargetStatus status)
{
    std::string today = getCurrentDate();

    std::string sn =
        device_id_sn.empty()
            ? "unknown_sn"
            : device_id_sn;

    std::string save_dir =
        debug_image_save_path_ + "/" +
        task_name + "/" +
        sn + "/" +
        today + "/" +
        TargetStatusToStr(status);

    return save_dir;
}

void Debug::saveSegLabel(
    const cv::Mat &image,
    const std::vector<std::vector<cv::Point>> &contours,
    const std::vector<int> &cls_ids,
    const std::string &task_name,
    const std::string &save_sample_info_string,
    TargetStatus status)
{
    // =================================================
    // 开关检查
    // =================================================

    if (!is_save_debug_image_)
    {
        return;
    }

    // =================================================
    // 图像检查
    // =================================================

    if (image.empty())
    {
        std::cerr
            << "[Debug] saveSegLabel image empty"
            << std::endl;

        return;
    }

    // =================================================
    // contour 与 cls_id 数量检查
    // =================================================

    if (!contours.empty() &&
        contours.size() != cls_ids.size())
    {
        std::cerr
            << "[Debug] contours.size != cls_ids.size"
            << std::endl;

        return;
    }

    // =================================================
    // 锁
    // =================================================

    std::lock_guard<std::mutex> lock(mutex_);
    // =================================================
    // 获取目录
    // =================================================

    std::string save_dir =
        buildSaveDirectory(
            task_name,
            status);

    // =================================================
    // 创建目录
    // =================================================

    if (!createDirectoryRecursive(save_dir))
    {
        std::cerr
            << "[Debug] create save dir failed: "
            << save_dir
            << std::endl;

        return;
    }

    // =================================================
    // 时间戳
    // =================================================

    int64_t timestamp_ms =
        getCurrentTimestampMs();

    // =================================================
    // 文件名
    // =================================================

    std::string image_filename =
        save_dir + "/" +
        std::to_string(timestamp_ms) + "_" + save_sample_info_string +
        ".jpg";

    std::string label_filename =
        save_dir + "/" +
        std::to_string(timestamp_ms) + "_" + save_sample_info_string +
        ".txt";

    // =================================================
    // 保存图片
    // =================================================

    image_buffer_t buf;

    memset(
        &buf,
        0,
        sizeof(buf));

    buf.width =
        image.cols;

    buf.height =
        image.rows;

    buf.format =
        IMAGE_FORMAT_RGB888;

    buf.size =
        image.total() *
        image.elemSize();

    buf.virt_addr =
        (unsigned char *)image.data;

    int ret =
        write_image(
            image_filename.c_str(),
            &buf);

    if (ret != 0)
    {
        std::cerr
            << "[Debug] image save failed: "
            << image_filename
            << std::endl;

        return;
    }

    std::cout
        << "[Debug] image saved: "
        << image_filename
        << std::endl;

    // =================================================
    // contour 为空
    // 只保存图片，不保存 txt
    // =================================================

    if (contours.empty())
    {
        std::cout
            << "[Debug] contours empty, skip label save"
            << std::endl;

        return;
    }

    // =================================================
    // 打开 label 文件
    // =================================================

    std::ofstream ofs(label_filename);

    if (!ofs.is_open())
    {
        std::cerr
            << "[Debug] open label file failed: "
            << label_filename
            << std::endl;

        return;
    }

    // =================================================
    // 图像尺寸
    // =================================================

    const float img_w =
        static_cast<float>(image.cols);

    const float img_h =
        static_cast<float>(image.rows);

    // =================================================
    // 写入 contour
    // =================================================

    for (size_t i = 0;
         i < contours.size();
         i++)
    {
        const auto &contour =
            contours[i];

        const int cls_id =
            cls_ids[i];

        // 至少需要3个点
        if (contour.size() < 3)
        {
            continue;
        }

        // class id
        ofs << cls_id;

        // polygon 点
        for (const auto &pt : contour)
        {
            float x =
                static_cast<float>(pt.x) / img_w;

            float y =
                static_cast<float>(pt.y) / img_h;

            // clamp 到 [0,1]
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

    // =================================================
    // 输出日志
    // =================================================

    std::cout
        << "[Debug] seg label saved: "
        << label_filename
        << std::endl;
}

// 获取当前日期，格式为YYYYMMDD
std::string Debug::getCurrentDate()
{
    auto now = std::chrono::system_clock::now();

    std::time_t now_time =
        std::chrono::system_clock::to_time_t(now);

    std::tm local_tm;

    localtime_r(&now_time, &local_tm);

    char buffer[16] = {0};

    std::strftime(
        buffer,
        sizeof(buffer),
        "%Y%m%d",
        &local_tm);

    return std::string(buffer);
}
// 递归创建目录
bool Debug::createDirectoryRecursive(
    const std::string &path)
{
    if (path.empty())
    {
        return false;
    }

    // 已存在
    struct stat st;

    if (stat(path.c_str(), &st) == 0)
    {
        return S_ISDIR(st.st_mode);
    }

    std::string current_path;

    for (size_t i = 0; i < path.size(); ++i)
    {
        current_path += path[i];

        // 遇到 '/'
        if (path[i] == '/')
        {
            if (current_path.empty())
            {
                continue;
            }

            if (stat(current_path.c_str(), &st) != 0)
            {
                if (mkdir(current_path.c_str(), 0755) != 0)
                {
                    if (errno != EEXIST)
                    {
                        std::cerr
                            << "[Debug] mkdir failed: "
                            << current_path
                            << " errno="
                            << errno
                            << std::endl;

                        return false;
                    }
                }
            }
        }
    }

    // 创建最后一级目录
    if (stat(current_path.c_str(), &st) != 0)
    {
        if (mkdir(current_path.c_str(), 0755) != 0)
        {
            if (errno != EEXIST)
            {
                std::cerr
                    << "[Debug] mkdir failed: "
                    << current_path
                    << " errno="
                    << errno
                    << std::endl;

                return false;
            }
        }
    }

    return true;
}
// 删除过期目录
void Debug::removeExpiredDirectories()
{
    DIR *dir = opendir(debug_image_save_path_.c_str());

    if (dir == nullptr)
    {
        std::cerr << "open dir failed: "
                  << debug_image_save_path_
                  << std::endl;
        return;
    }

    // 获取过期日期
    auto now = std::chrono::system_clock::now();

    auto expire_time =
        now - std::chrono::hours(24 * keep_days_);

    std::time_t expire_tt =
        std::chrono::system_clock::to_time_t(expire_time);

    std::tm expire_tm;

    localtime_r(&expire_tt, &expire_tm);

    char expire_date[16] = {0};

    std::strftime(
        expire_date,
        sizeof(expire_date),
        "%Y%m%d",
        &expire_tm);

    struct dirent *entry;

    while ((entry = readdir(dir)) != nullptr)
    {
        std::string dir_name = entry->d_name;

        // 跳过 . 和 ..
        if (dir_name == "." || dir_name == "..")
        {
            continue;
        }

        // 必须是8位日期目录
        if (dir_name.size() != 8)
        {
            continue;
        }

        // 判断是否过期
        if (dir_name < expire_date)
        {
            std::string full_path =
                debug_image_save_path_ + "/" + dir_name;

            std::cout << "remove expired dir: "
                      << full_path
                      << std::endl;

            // 递归删除目录
            std::string cmd =
                "rm -rf " + full_path;

            system(cmd.c_str());
        }
    }

    closedir(dir);
}

int64_t Debug::getCurrentTimestampMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::string Debug::set_ai_capture_save_info(
    const std::string &device_id_sn,
    const std::string &model_task_type,
    const std::string &target_status,
    float confidence)
{
    float safe_confidence = std::max(0.0f, confidence);
    ai_capture_save_info_.device_id_sn = device_id_sn;
    ai_capture_save_info_.model_task_type = model_task_type;
    ai_capture_save_info_.has_target = target_status;
    ai_capture_save_info_.confidence = confidence;

    std::ostringstream oss;
    // confidence 保留三位小数
    oss << device_id_sn
        << "_" << model_task_type
        << "_" << target_status
        << "_" << std::fixed
        << std::setprecision(3)
        << safe_confidence;

    return oss.str();
}
