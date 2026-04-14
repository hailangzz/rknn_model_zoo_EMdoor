#pragma once
#include "base_detect_interface.h"

// iou追踪模块
struct Track
{
    int id;
    cv::Rect2f box;

    cv::Point2f velocity;   // ⭐ 速度 (vx, vy)
    int lost_frames;
    int age;
};


class MotionTracker
  {
  public:
      MotionTracker()
      {
          next_id = 0;
      }

      void update(std::vector<ObjectCameraDetectResult>& detections);
      

  private:
      std::vector<Track> tracks;
      int next_id;
      cv::Rect2f toRect(const ObjectCameraDetectResult& det);
      cv::Point2f get_center(const cv::Rect2f& r);
      float center_distance(const cv::Rect2f& a, const cv::Rect2f& b);
      float compute_iou(const cv::Rect2f& a, const cv::Rect2f& b);
      
  };