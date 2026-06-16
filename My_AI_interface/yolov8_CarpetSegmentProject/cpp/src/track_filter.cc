#include "track_filter.h"
#include <algorithm>

TrackFilter::TrackFilter(
    int confirm_frames,
    int max_miss,
    int delete_miss,
    float max_center_dist,
    float max_area_ratio)
    : confirm_frames_(confirm_frames),
      max_miss_(max_miss),
      delete_miss_(delete_miss),
      max_center_dist_(max_center_dist),
      max_area_ratio_(max_area_ratio)
{
}

cv::Point2f TrackFilter::getCenter(
    const ObjectCameraDetectResult &obj) const
{
  return cv::Point2f(
      (obj.target_box.left + obj.target_box.right) * 0.5f,
      (obj.target_box.top + obj.target_box.bottom) * 0.5f);
}

float TrackFilter::getArea(
    const ObjectCameraDetectResult &obj) const
{
  return (obj.target_box.right - obj.target_box.left) *
         (obj.target_box.bottom - obj.target_box.top);
}

bool TrackFilter::isMatch(
    const TrackObject &track,
    const ObjectCameraDetectResult &det) const
{
  //---------------------------------
  // 类别必须一致
  //---------------------------------

  if (track.result.cls_id != det.cls_id)
  {
    return false;
  }

  //---------------------------------
  // 中心点距离
  //---------------------------------

  cv::Point2f center_now =
      getCenter(det);

  float center_dist =
      cv::norm(center_now - track.center);

  if (center_dist > max_center_dist_)
  {
    return false;
  }

  //---------------------------------
  // 面积变化
  //---------------------------------

  float area_now =
      getArea(det);

  float min_area =
      std::max(
          1.0f,
          std::min(area_now, track.area));

  float max_area =
      std::max(area_now, track.area);

  float ratio =
      max_area / min_area;

  if (ratio > max_area_ratio_)
  {
    return false;
  }

  return true;
}

void TrackFilter::update(
    const std::vector<ObjectCameraDetectResult> &detections,
    std::vector<ObjectCameraDetectResult> &outputs)
{
  outputs.clear();

  //---------------------------------
  // 标记检测是否已匹配
  //---------------------------------

  std::vector<bool> used(
      detections.size(),
      false);

  //---------------------------------
  // 更新已有Track
  //---------------------------------

  for (auto &track : tracks_)
  {
    int best_idx = -1;

    float best_dist = 999999.0f;

    for (size_t i = 0; i < detections.size(); i++)
    {
      if (used[i])
        continue;

      if (!isMatch(track, detections[i]))
        continue;

      float dist =
          cv::norm(
              getCenter(detections[i]) -
              track.center);

      if (dist < best_dist)
      {
        best_dist = dist;
        best_idx = (int)i;
      }
    }

    if (best_idx >= 0)
    {
      //---------------------------------
      // 匹配成功
      //---------------------------------

      used[best_idx] = true;

      track.result =
          detections[best_idx];

      track.center =
          getCenter(detections[best_idx]);

      track.area =
          getArea(detections[best_idx]);

      track.hit_count++;

      track.miss_count = 0;

      if (track.hit_count >= confirm_frames_)
      {
        track.confirmed = true;
      }
    }
    else
    {
      //---------------------------------
      // 当前帧未匹配到
      //---------------------------------

      track.miss_count++;
    }
  }

  //---------------------------------
  // 创建新Track
  //---------------------------------

  for (size_t i = 0; i < detections.size(); i++)
  {
    if (used[i])
      continue;

    TrackObject track;

    track.track_id =
        next_track_id_++;

    track.result =
        detections[i];

    track.center =
        getCenter(detections[i]);

    track.area =
        getArea(detections[i]);

    track.hit_count = 1;

    track.miss_count = 0;

    track.confirmed = false;

    tracks_.push_back(track);
  }

  //---------------------------------
  // 输出稳定目标
  //---------------------------------

  for (auto &track : tracks_)
  {
    if (!track.confirmed)
      continue;

    //---------------------------------
    // 漏检补偿
    //---------------------------------

    if (track.miss_count <= max_miss_)
    {
      outputs.push_back(
          track.result);
    }
  }

  //---------------------------------
  // 删除长期丢失Track
  //---------------------------------

  tracks_.erase(
      std::remove_if(
          tracks_.begin(),
          tracks_.end(),
          [this](const TrackObject &t)
          {
            return t.miss_count >
                   delete_miss_;
          }),
      tracks_.end());
}