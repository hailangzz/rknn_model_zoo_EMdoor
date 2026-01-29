#include "carpet_detect_interface.h"
#include <ros/ros.h>


bool carpet_model_init(const char* model_path){
  ROS_INFO("carpet_model_init finished.");
  return true;
}

bool carpet_detect_infer(const cv::Mat& img){
  ROS_INFO("carpet_detect_infer finished.");
  return true;
}

void carpet_model_release(){
  ROS_INFO("carpet_model_release finished.");
}