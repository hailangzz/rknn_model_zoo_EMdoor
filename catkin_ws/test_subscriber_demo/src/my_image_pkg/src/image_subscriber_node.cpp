#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // 将 ROS 图像消息转换为 OpenCV 图像 (RGB格式)
        cv::Mat img = cv_bridge::toCvShare(msg, "rgb8")->image;

        // 打印尺寸：宽 × 高 × 通道
        ROS_INFO("Image size: width=%d, height=%d, channels=%d",
                img.cols, img.rows, img.channels());

        // 显示图像
        cv::imshow("Received Image", img);
        cv::waitKey(1);

        // static int count = 0;
        // // 可选：保存图像
        // // 1️⃣ 先拼完整路径
        // std::string filename = "/home/robot/zhangzhuo/catkin_ws/ros_images/image_" + std::to_string(count++) + ".png";
        // // 2️⃣ 保存图片
        // cv::imwrite(filename, img);
        // // 3️⃣ 打印保存的文件名
        // ROS_INFO("Saved image: %s", filename.c_str());

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_subscriber_node");
    ros::NodeHandle nh;

    // 订阅你给出的 topic
    ros::Subscriber sub = nh.subscribe("camera/color/image_raw", 10, imageCallback);

    ROS_INFO("Subscribed to topic: camera/color/image_raw");

    ros::spin();

    return 0;
}
