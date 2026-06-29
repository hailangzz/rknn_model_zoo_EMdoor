/**
 * @brief 检测结果时序滤波器
 *
 * 功能：
 * 1. 过滤偶发误检（连续多帧检测到才输出）
 * 2. 补偿偶发漏检（短时间丢失继续输出历史结果）
 * 3. 基于类别、BBox中心距离和面积变化进行目标关联
 * 4. 自动管理目标Track的创建、更新和删除
 *
 * 适用于扫地机器人YOLO检测结果稳定化处理。
 */

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "base_detect_interface.h"

class TrackFilter
{
public:
  TrackFilter(
      int confirm_frames = 3,
      int max_miss = 2,
      int delete_miss = 5,
      float max_center_dist = 120.0f,
      float max_area_ratio = 2.5f);

  /*
  static TrackFilter g_track_filter(
    3,      // 连续3帧确认
    2,      // 允许漏检2帧
    5,      // 丢失5帧删除
    80.0f,  // 中心点最大移动距离
    2.5f    // 面积最大变化倍数
    );

    //1280×720：max_center_dist = 120; //640×480：max_center_dist = 80
*/

  void update(
      const std::vector<ObjectCameraDetectResult> &detections,
      std::vector<ObjectCameraDetectResult> &outputs);

private:
  struct TrackObject
  {
    int track_id = 0;

    int hit_count = 0;
    int miss_count = 0;

    bool confirmed = false;

    ObjectCameraDetectResult result;

    cv::Point2f center;

    float area = 0;
  };

private:
  cv::Point2f getCenter(
      const ObjectCameraDetectResult &obj) const;

  float getArea(
      const ObjectCameraDetectResult &obj) const;

  bool isMatch(
      const TrackObject &track,
      const ObjectCameraDetectResult &det) const;

private:
  std::vector<TrackObject> tracks_;

  int next_track_id_ = 0;

  int confirm_frames_;
  int max_miss_;
  int delete_miss_;

  float max_center_dist_;
  float max_area_ratio_;
};