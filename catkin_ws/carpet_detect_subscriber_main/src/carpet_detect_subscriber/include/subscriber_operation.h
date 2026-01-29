#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <thread>
#include <mutex>
#include <condition_variable>

#include "carpet_detect_interface.h"
#include <carpet_detect_msgs/ObjectCameraDetectResultArray.h>


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

    /* ============ ROS 参数监控线程 ============ */
    void rosparamMonitorLoop();

    /* ============ 资源释放 ============ */
    void shutdown();

    void publishDebugImage(const cv::Mat& img,const std::vector<ObjectCameraDetectResult>& results,const std_msgs::Header& header);


    /* ============ ROS ============ */
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    const std::string image_sub_topic_= "camera/color/image_raw";

    /* ============ 推理线程 ============ */
    std::thread infer_thread_;
    bool running_;

    /* ============ 检测功能开关 ============ */
    bool infer_enable_;                     // 私有推理开关
    const std::string infer_enable_param_ = "/task_manager/carpet_inspection_enable";  // ROS param 名
    std::thread rosparam_monitor_thread_;  // 新增：监控开关参数线程

    /* ============ 图像缓冲 ============ */
    cv::Mat latest_frame_;
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    bool has_new_frame_;
    // ObjectCameraDetectResult camera_coordinates_results_[64]; // 最多存 64 个检测目标的相机坐标值
    std::vector<ObjectCameraDetectResult> camera_coordinates_results_;


    //  新增：检测结果发布器
    ros::Publisher detect_result_pub_;
    const std::string detect_result_pub_topic_= "/camera_detect/object_camera_coordinates_results";
    // struct → msg 转换函数
    void convertToMsg(const ObjectCameraDetectResult& src,carpet_detect_msgs::ObjectCameraDetectResult& dst);

    // 可视化图像发布器
    ros::Publisher debug_image_pub_;
    const std::string debug_image_pub_topic_ =  "/camera_detect/object_camera_coordinates_image";

    // ros下坐标系标志
    const std::string camera_coordinate_system_flag_= "color_camera_optical_link";

};

