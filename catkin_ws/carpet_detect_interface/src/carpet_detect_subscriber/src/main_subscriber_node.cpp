#include "subscriber_operation.h"

/* ================= main ================= */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "carpet_detect_ros_node");
    ros::NodeHandle nh;

    try
    {
        CarpetDetectNode node(nh);
        node.create_infer_thread();
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_FATAL("Exception: %s", e.what());
    }

    return 0;
}
