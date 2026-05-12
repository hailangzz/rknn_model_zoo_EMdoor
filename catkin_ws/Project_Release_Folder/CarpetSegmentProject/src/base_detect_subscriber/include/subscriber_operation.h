#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <thread>
#include <mutex>
#include <condition_variable>

#include "base_detect_interface.h"
#include <base_detect_msgs/ObjectCameraDetectResultArray.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

class BaseDetectNode
{
public:
    BaseDetectNode(ros::NodeHandle &nh);
    ~BaseDetectNode();

    void create_infer_thread();

private:
    /* ============ ROS 回调 ============ */
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

    /* ============ 推理主循环 ============ */
    void inferenceLoop();

    /* ============ ROS 参数监控线程 ============ */
    void rosparamMonitorLoop();

    /* ============ 资源释放 ============ */
    void shutdown();

    void publishDebugImage(const cv::Mat &img,
                           const std::vector<ObjectCameraDetectResult> &results,
                           const std_msgs::Header &header);
    void convertToMsg(
        const ObjectCameraDetectResult &src,
        base_detect_msgs::ObjectCameraDetectResult &dst);

    /* ============ ROS ============ */
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Subscriber pose_sub_;

    const std::string image_sub_topic_ = "camera/color/image_raw";
    const std::string pose_sub_topic_ = "/pose_with_cov";

    /* ============ 推理线程 ============ */
    std::thread infer_thread_;
    bool running_;

    /* ============ 检测开关 ============ */
    bool infer_enable_;
    const std::string infer_enable_param_ = "/task_manager/carpet_inspection_enable";
    std::thread rosparam_monitor_thread_;

    /* ============ 图像缓存 ============ */
    cv::Mat latest_frame_;
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    bool has_new_frame_;

    std::vector<ObjectCameraDetectResult> camera_coordinates_results_;

    /* ============ 发布器 ============ */
    ros::Publisher detect_result_pub_;
    ros::Publisher debug_image_pub_;

    const std::string detect_result_pub_topic_ = "/camera_detect/object_camera_coordinates_results";
    const std::string debug_image_pub_topic_ = "/camera_detect/object_camera_coordinates_image";

    const std::string camera_coordinate_system_flag_ = "color_camera_optical_link";

    /* ============ Pose缓存（关键修复） ============ */
    geometry_msgs::PoseWithCovarianceStamped latest_pose_;
    std::mutex pose_mutex_;
    bool has_pose_;
};