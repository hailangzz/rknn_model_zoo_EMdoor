#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <thread>
#include <mutex>
#include <condition_variable>

#include "carpet_detect_interface.h"


class CarpetDetectNode
{
public:
    CarpetDetectNode(ros::NodeHandle& nh);
    ~CarpetDetectNode();

    void create_infer_thread();

private:

    /* ============ ROS 回调 ============ */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    
    /* ============ 推理主循环 ============ */
    void inferenceLoop();    
    /* ============ 资源释放 ============ */
    void shutdown();

    /* ============ ROS ============ */
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;

    /* ============ 推理线程 ============ */
    std::thread infer_thread_;
    bool running_;

    /* ============ 图像缓冲 ============ */
    cv::Mat latest_frame_;
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    bool has_new_frame_;

        
};

