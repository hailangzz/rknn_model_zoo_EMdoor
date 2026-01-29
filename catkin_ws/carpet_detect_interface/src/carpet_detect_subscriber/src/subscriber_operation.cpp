#include "subscriber_operation.h"


CarpetDetectNode::CarpetDetectNode(ros::NodeHandle& nh)        
    {
      nh_ = nh;
      has_new_frame_ = false;
      running_ = true;

      /* 初始化模型 */
      if (!carpet_model_init("/home/robot/models/yolov8.rknn"))
      {
          ROS_FATAL("Failed to init carpet model");
          throw std::runtime_error("model init failed");
      }

      /* 订阅图像 */
      image_sub_ = nh_.subscribe( "camera/color/image_raw", 1, &CarpetDetectNode::imageCallback, this);

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
    ROS_INFO("YOLOv8 inference thread started");

    while (ros::ok() && running_)
    {
        cv::Mat img;

        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            frame_cv_.wait(lock, [this] {
                return has_new_frame_ || !running_ || !ros::ok();
            });

            if (!running_ || !ros::ok())
                break;

            img = latest_frame_.clone();
            has_new_frame_ = false;
        }

        ROS_INFO("carpet_detect_infer begin");
        carpet_detect_infer(img);
        ROS_INFO("carpet_detect_infer finished");
    }

    ROS_INFO("YOLOv8 inference thread exited");
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

    carpet_model_release();

    ROS_INFO("CarpetDetectNode shutdown complete");
}