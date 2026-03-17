#include "subscriber_operation.h"

#include <fstream>
#include <iomanip>

// 将xyz数据点，写入到本地文件中
void appendCoordsToFile(
    carpet_detect_msgs::ObjectCameraDetectResult& dst,
    const std::string& file_path)
{
    std::ofstream ofs(file_path, std::ios::out | std::ios::app);
    if (!ofs.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return;
    }

    ofs << std::fixed << std::setprecision(6);

    for (size_t i = 0; i < dst.coords.size(); ++i) {
        ofs << dst.coords[i].x << ","
            << dst.coords[i].y << ","
            << dst.coords[i].z;

        if (i != dst.coords.size() - 1) {
            ofs << " ";
        }
    }

    ofs << "\n";  // 写完一次换一行
    ofs.close();
}



CarpetDetectNode::CarpetDetectNode(ros::NodeHandle& nh)        
    {
      nh_ = nh;
      has_new_frame_ = false;
      running_ = true;
      infer_enable_ = false;  // 默认允许推理

      /* 初始化模型 */
      if (!carpet_model_init("./config/cfg.txt"))
      {
          ROS_FATAL("Failed to init carpet model");
          throw std::runtime_error("model init failed");
      }

      /* 订阅图像 */
      image_sub_ = nh_.subscribe( image_sub_topic_, 1, &CarpetDetectNode::imageCallback, this);

      //  新增：发布检测结果
      detect_result_pub_ = nh_.advertise<carpet_detect_msgs::ObjectCameraDetectResultArray>( detect_result_pub_topic_, 1);

      // 新增：发布带检测框图像
      debug_image_pub_ = nh_.advertise<sensor_msgs::Image>(debug_image_pub_topic_, 1);

      /* 启动 ROS 参数监控线程 */
      rosparam_monitor_thread_ = std::thread(&CarpetDetectNode::rosparamMonitorLoop, this);

      ROS_INFO("CarpetDetectNode initialized");
    }

CarpetDetectNode::~CarpetDetectNode()
    {
        shutdown();
    }

void CarpetDetectNode::create_infer_thread(){
  try
      {
    /* 启动推理线程 */
      infer_thread_ = std::thread(&CarpetDetectNode::inferenceLoop, this);
      }
  catch (const cv_bridge::Exception& e)
    {
      ROS_ERROR("infer_thread_ exception: %s", e.what());
    }
}

void CarpetDetectNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv::Mat img = cv_bridge::toCvShare(msg, "rgb8")->image;

            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_frame_ = img.clone();
                has_new_frame_ = true;
            }

            frame_cv_.notify_one();
        }
        catch (const cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }


    /* ============ 推理主循环 ============ */
void CarpetDetectNode::inferenceLoop()
{
    // ROS_INFO("YOLOv8 inference thread started");

    while (ros::ok() && running_)
    {

        cv::Mat img;

        /* ---------- 等待新帧 ---------- */
        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            frame_cv_.wait(lock, [this] {
                return (has_new_frame_ && infer_enable_) || !running_ || !ros::ok();
            });

            if (!running_ || !ros::ok())
                break;

            img = latest_frame_.clone();
            has_new_frame_ = false;
        }

        /* ---------- 推理 ---------- */
        // ROS_DEBUG("carpet_detect_infer begin");
        // carpet_detect_infer(img, camera_coordinates_results_);
        if (!carpet_detect_infer(img, camera_coordinates_results_))
            {
                ROS_DEBUG("No valid carpet detected, skip publish");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

        ROS_DEBUG("carpet_detect_infer finished");

        /* ---------- 构造 ROS 消息 ---------- */
        carpet_detect_msgs::ObjectCameraDetectResultArray msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = camera_coordinate_system_flag_;   // 很重要

        for (const auto& r : camera_coordinates_results_)
            {
                ROS_INFO(
                    "Detect object: cls_id=%d, prop=%.3f, "
                    "LT(%.2f, %.2f, %.2f), RT(%.2f, %.2f, %.2f), "
                    "RB(%.2f, %.2f, %.2f), LB(%.2f, %.2f, %.2f)",
                    r.cls_id, r.prop,
                    r.coords[0].X, r.coords[0].Y, r.coords[0].Z,
                    r.coords[1].X, r.coords[1].Y, r.coords[1].Z,
                    r.coords[2].X, r.coords[2].Y, r.coords[2].Z,
                    r.coords[3].X, r.coords[3].Y, r.coords[3].Z
                );


                for (int i = 0; i < 20; ++i) {
                    auto& p = r.add_edge_point_single_pixel_camera_coordinates[i];
                    ROS_INFO(" dst : [%2d] X=%.3f, Y=%.3f, Z=%.3f\n", i, p.X, p.Y, p.Z);
    }

                carpet_detect_msgs::ObjectCameraDetectResult one;
                convertToMsg(r, one);
                msg.results.push_back(one);
            }

        /* ---------- 发布 ---------- */
        detect_result_pub_.publish(msg);
        ROS_DEBUG("detect_result_pub_ published, size=%zu", msg.results.size());

        // 发布绘图结果
        publishDebugImage(img, camera_coordinates_results_, msg.header);
    }

    // ROS_INFO("YOLOv8 inference thread exited");
}

    /* ============ 资源释放 ============ */
void CarpetDetectNode::shutdown()
{
    if (!running_)
        return;

    running_ = false;
    frame_cv_.notify_all();

    if (infer_thread_.joinable())
        infer_thread_.join();
    
    if (rosparam_monitor_thread_.joinable())
        rosparam_monitor_thread_.join();

    carpet_model_release();

    ROS_INFO("CarpetDetectNode shutdown complete");
}

void CarpetDetectNode::publishDebugImage(const cv::Mat& img,const std::vector<ObjectCameraDetectResult>& results,const std_msgs::Header& header)
{
    if (img.empty())
        return;

    // 一定要 clone，不能修改原图
    cv::Mat debug_img = img.clone();

    for (const auto& r : results)
    {
        // ================================
        // 1. 使用像素坐标画框（关键修正点）
        // ================================
        const ObjectTargetBox& box = r.target_box;

        // 合法性保护（防止越界）
        int left   = std::max(0, box.left);
        int top    = std::max(0, box.top);
        int right  = std::min(box.right,  img.cols - 1);
        int bottom = std::min(box.bottom, img.rows - 1);

        cv::Point pt1(left, top);
        cv::Point pt2(right, bottom);

        cv::rectangle(
            debug_img,
            pt1,
            pt2,
            cv::Scalar(0, 255, 0),  // 绿色框
            2
        );

        // ================================
        // 2. 画类别 + 置信度
        // ================================
        char text[64];
        snprintf(text, sizeof(text), "id:%d %.2f", r.cls_id, r.prop);

        int text_y = std::max(0, top - 5);
        cv::putText(
            debug_img,
            text,
            cv::Point(left, text_y),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0, 255, 0),
            1
        );
    }

    // ================================
    // 3. OpenCV → ROS Image
    // ================================
    sensor_msgs::ImagePtr img_msg =
        cv_bridge::CvImage(header, "rgb8", debug_img).toImageMsg();

    debug_image_pub_.publish(img_msg);
}


// void CarpetDetectNode::convertToMsg(
//     const ObjectCameraDetectResult& src,
//     carpet_detect_msgs::ObjectCameraDetectResult& dst)
// {
//     dst.prop = src.prop;
//     dst.cls_id = src.cls_id;

//     // ===== 关键修复：先 resize（4 + 20）=====
//     dst.coords.resize(4 + 20);

//     // ===== 4 个角点 =====
//     for (int i = 0; i < 4; i++) {
//         dst.coords[i].x = src.coords[i].X;
//         dst.coords[i].y = src.coords[i].Y;
//         dst.coords[i].z = src.coords[i].Z;

//     }

//     ROS_INFO("CarpetDetectNode::convertToMsg");

//     // // ===== 20 个 edge points =====
//     // for (int i = 0; i < 20; i++) {
//     //     dst.coords[i + 4].x =
//     //         src.add_edge_point_single_pixel_camera_coordinates[i].X;
//     //     dst.coords[i + 4].y =
//     //         src.add_edge_point_single_pixel_camera_coordinates[i].Y;
//     //     dst.coords[i + 4].z =
//     //         src.add_edge_point_single_pixel_camera_coordinates[i].Z;
//     // }

//     // ROS_INFO(
//     //     "convertToMsg Detect object: cls_id=%d, prop=%.3f, "
//     //     "LT(%.2f, %.2f, %.2f), RT(%.2f, %.2f, %.2f), "
//     //     "RB(%.2f, %.2f, %.2f), LB(%.2f, %.2f, %.2f)",
//     //     dst.cls_id, dst.prop,
//     //     dst.coords[0].x, dst.coords[0].y, dst.coords[0].z,
//     //     dst.coords[1].x, dst.coords[1].y, dst.coords[1].z,
//     //     dst.coords[2].x, dst.coords[2].y, dst.coords[2].z,
//     //     dst.coords[3].x, dst.coords[3].y, dst.coords[3].z
//     // );
// }

void CarpetDetectNode::convertToMsg(
    const ObjectCameraDetectResult& src,
    carpet_detect_msgs::ObjectCameraDetectResult& dst)
{
    dst.prop = src.prop;
    dst.cls_id = src.cls_id;

    // ===== 总共长度 = 4角点 + 20边缘点 =====
    // dst.coords.resize(4 + 20);
    const int corner_num =
        sizeof(src.coords) / sizeof(src.coords[0]);

    const int edge_num =
        sizeof(src.add_edge_point_single_pixel_camera_coordinates) /
        sizeof(src.add_edge_point_single_pixel_camera_coordinates[0]);
    dst.coords.resize(corner_num + edge_num);


    int dst_idx = 0;  // dst.coords 的写入索引
    int edge_idx = 0; // src.add_edge_point_single_pixel_camera_coordinates 的索引

    // ===== 4 个角点，每个角点后跟 5 个边缘点 =====
    for (int i = 0; i < 4; i++) {
        // 写角点
        dst.coords[dst_idx].x = src.coords[i].X;
        dst.coords[dst_idx].y = src.coords[i].Y;
        dst.coords[dst_idx].z = src.coords[i].Z;
        dst_idx++;

        ROS_INFO(" src.coords : [%2d] X=%.3f, Y=%.3f, Z=%.3f\n", i, src.coords[i].X, src.coords[i].Y, src.coords[i].Z);
        // 写紧跟的 5 个边缘点
        for (int j = 0; j < 5; j++) {
            dst.coords[dst_idx].x =
                src.add_edge_point_single_pixel_camera_coordinates[edge_idx].X;
            dst.coords[dst_idx].y =
                src.add_edge_point_single_pixel_camera_coordinates[edge_idx].Y;
            dst.coords[dst_idx].z =
                src.add_edge_point_single_pixel_camera_coordinates[edge_idx].Z;

            ROS_INFO(" src.coords : [%2d] X=%.3f, Y=%.3f, Z=%.3f\n", i, src.add_edge_point_single_pixel_camera_coordinates[edge_idx].X, src.add_edge_point_single_pixel_camera_coordinates[edge_idx].Y, src.add_edge_point_single_pixel_camera_coordinates[edge_idx].Z);
            dst_idx++;
            edge_idx++;
        }
    }

    // 将xyz点结果，写入到本地文件中
    // appendCoordsToFile( dst, "./carpet_coords.txt");


    ROS_INFO("CarpetDetectNode::convertToMsg");

    ROS_INFO(
        "convertToMsg Detect object: cls_id=%d, prop=%.3f, "
        "LT(%.2f, %.2f, %.2f), RT(%.2f, %.2f, %.2f), "
        "RB(%.2f, %.2f, %.2f), LB(%.2f, %.2f, %.2f)",
        dst.cls_id, dst.prop,
        dst.coords[0].x, dst.coords[0].y, dst.coords[0].z,
        dst.coords[6].x, dst.coords[6].y, dst.coords[6].z,
        dst.coords[12].x, dst.coords[12].y, dst.coords[12].z,
        dst.coords[18].x, dst.coords[18].y, dst.coords[18].z
    );
}


/* ============ ROS 参数监控线程 ============ */
void CarpetDetectNode::rosparamMonitorLoop()
{
    ros::Rate rate(1.0);  // 每秒读取一次
    while (ros::ok() && running_)
    {
        bool param_val = false;
        if (nh_.getParam(infer_enable_param_, param_val))
        {
            infer_enable_ = param_val;
            ROS_DEBUG("carpet_detect_enable = %d", infer_enable_);
        }
        rate.sleep();
    }
}
