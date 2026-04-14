#include "tracker.h"


void MotionTracker::update(std::vector<ObjectCameraDetectResult>& detections)
      {
          // ⭐ 初始化 track_id
          for (auto& det : detections)
              det.track_id = -1;

          // ---------------------------
          // 1️⃣ 预测轨迹（简单匀速模型）
          // ---------------------------
          for (auto& track : tracks)
          {
              track.box.x += track.velocity.x;
              track.box.y += track.velocity.y;
          }

          std::vector<bool> matched_det(detections.size(), false);

          // ---------------------------
          // 2️⃣ 匹配（IoU + 中心点）
          // ---------------------------
          for (auto& track : tracks)
          {
              float best_score = 0;
              int best_idx = -1;

              for (int i = 0; i < detections.size(); i++)
              {
                  if (matched_det[i]) continue;

                  cv::Rect2f det_box = toRect(detections[i]);

                  float iou = compute_iou(track.box, det_box);
                  float dist = center_distance(track.box, det_box);

                  // ⭐ 综合评分（关键）
                  float score = iou - 0.001f * dist;

                  if (score > best_score)
                  {
                      best_score = score;
                      best_idx = i;
                  }
              }

              // ⭐ 匹配条件（双约束）
              if (best_idx != -1)
              {
                  cv::Rect2f det_box = toRect(detections[best_idx]);
                  float iou = compute_iou(track.box, det_box);
                  float dist = center_distance(track.box, det_box);

                  if (iou > 0.3 || dist < 50)
                  {
                      // ✅ 更新速度（关键）
                      cv::Point2f new_center = get_center(det_box);
                      cv::Point2f old_center = get_center(track.box);

                      track.velocity = new_center - old_center;

                      // ✅ 更新box
                      track.box = det_box;

                      track.lost_frames = 0;
                      track.age++;

                      detections[best_idx].track_id = track.id;
                      matched_det[best_idx] = true;
                      continue;
                  }
              }

              // ❌ 没匹配
              track.lost_frames++;
          }

          // ---------------------------
          // 3️⃣ 新建track
          // ---------------------------
          for (int i = 0; i < detections.size(); i++)
          {
              if (!matched_det[i])
              {
                  Track t;
                  t.id = next_id++;
                  t.box = toRect(detections[i]);
                  t.velocity = cv::Point2f(0, 0);
                  t.lost_frames = 0;
                  t.age = 1;

                  detections[i].track_id = t.id;
                  tracks.push_back(t);
              }
          }

          // ---------------------------
          // 4️⃣ 删除丢失track
          // ---------------------------
          tracks.erase(
              std::remove_if(tracks.begin(), tracks.end(),
                  [](Track& t)
                  {
                      return t.lost_frames > 10;
                  }),
              tracks.end()
          );

          // ---------------------------
          // 5️⃣ 限制track数量（防爆）
          // ---------------------------
          if (tracks.size() > 50)
          {
              tracks.erase(tracks.begin());
          }
      }


// ---------------------------
// 工具函数
// ---------------------------

cv::Rect2f MotionTracker::toRect(const ObjectCameraDetectResult& det)
{
    return cv::Rect2f(
        det.target_box.left,
        det.target_box.top,
        det.target_box.right - det.target_box.left,
        det.target_box.bottom - det.target_box.top
    );
}

cv::Point2f MotionTracker::get_center(const cv::Rect2f& r)
{
    return cv::Point2f(
        r.x + r.width * 0.5f,
        r.y + r.height * 0.5f
    );
}

float MotionTracker::center_distance(const cv::Rect2f& a, const cv::Rect2f& b)
{
    cv::Point2f ca = get_center(a);
    cv::Point2f cb = get_center(b);

    return cv::norm(ca - cb);
}

float MotionTracker::compute_iou(const cv::Rect2f& a, const cv::Rect2f& b)
{
    float xx1 = std::max(a.x, b.x);
    float yy1 = std::max(a.y, b.y);
    float xx2 = std::min(a.x + a.width,  b.x + b.width);
    float yy2 = std::min(a.y + a.height, b.y + b.height);

    float w = std::max(0.0f, xx2 - xx1);
    float h = std::max(0.0f, yy2 - yy1);

    float inter = w * h;
    float union_area = a.area() + b.area() - inter;

    return union_area > 0 ? inter / union_area : 0;
}