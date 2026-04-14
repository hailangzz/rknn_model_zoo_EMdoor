
#include <cmath>
#include <iostream>
#include "common.h"
#include "postprocess.h"
#include "types.h"
#include <opencv2/opencv.hpp>
#include "base_detect_interface.h"

class CameraParameters {
public:
    // 构造函数
    CameraParameters(const ConfigInfo& config)
        : fx(config.camera_fx), fy(config.camera_fy), cx(config.camera_cx), cy(config.camera_cy), H(config.camera_H), pitch(config.camera_pitch),
          D_0(config.camera_D_0),D_1(config.camera_D_1),D_2(config.camera_D_2),D_3(config.camera_D_3),D_4(config.camera_D_4),D_5(config.camera_D_5),D_6(config.camera_D_6),D_7(config.camera_D_7){}

    bool ObjectboxToCameraXYZ(object_detect_result* det, const std::vector<std::vector<cv::Point>> &contours_mark_point);

private:


    // 像素坐标 -> 相机坐标（地面假设）
    bool pixelToCameraXYZGround(float u, float v, single_pixel_camera_coordinates &P);
    bool XYZGroundTopixel(single_pixel_camera_coordinates P,float &u, float &v);
    // 相机内参
    float fx, fy;
    float cx, cy;

    // 安装参数
    float H;      // 高度
    float pitch;  // 俯仰角

    //畸变参数
    float D_0, D_1, D_2, D_3, D_4, D_5, D_6, D_7;

    // 原始图像尺寸
    int orig_width = 1920;
    int orig_height = 1080;

    // 检测图像尺寸
    int det_width = 1280;
    int det_height = 720;
};


struct ObjectSize3D {
    float width;   // 物体宽度（单位：与 XYZ 一致，通常是米）
    float height;  // 物体高度
};
// 估计目标物体的实际宽度、高度
bool calcObjectSizeByAverage(ObjectCameraDetectResult& one,ObjectSize3D& size_out);


// // iou追踪模块
// struct Track
// {
//     int id;
//     cv::Rect2f box;
//     int lost_frames;   // 丢失帧数
//     int age;           // 存活时间
// };

// class SimpleTracker
// {
// public:
//     SimpleTracker()
//     {
//         next_id = 0;
//     }

//     void update(std::vector<ObjectCameraDetectResult>& detections)
//     {
//         std::vector<bool> matched_det(detections.size(), false);

//         // ---------------------------
//         // 1️⃣ 匹配（IoU）
//         // ---------------------------
//         for (auto& track : tracks)
//         {
//             float best_iou = 0;
//             int best_idx = -1;

//             for (int i = 0; i < detections.size(); i++)
//             {
//                 if (matched_det[i]) continue;

//                 float iou = compute_iou(track.box, toRect(detections[i]));

//                 if (iou > best_iou)
//                 {
//                     best_iou = iou;
//                     best_idx = i;
//                 }
//             }

//             if (best_iou > 0.3)  // 👉 IoU阈值（可调）
//             {
//                 // ✅ 匹配成功
//                 track.box = toRect(detections[best_idx]);
//                 track.lost_frames = 0;
//                 track.age++;

//                 detections[best_idx].track_id = track.id;
//                 matched_det[best_idx] = true;
//             }
//             else
//             {
//                 //  没匹配
//                 track.lost_frames++;
//             }
//         }

//         // ---------------------------
//         // 2️⃣ 新目标
//         // ---------------------------
//         for (int i = 0; i < detections.size(); i++)
//         {
//             if (!matched_det[i])
//             {
//                 Track new_track;
//                 new_track.id = next_id++;
//                 new_track.box = toRect(detections[i]);
//                 new_track.lost_frames = 0;
//                 new_track.age = 1;

//                 detections[i].track_id = new_track.id;
//                 tracks.push_back(new_track);
//             }
//         }

//         // ---------------------------
//         // 3️⃣ 删除丢失目标
//         // ---------------------------
//         tracks.erase(
//             std::remove_if(tracks.begin(), tracks.end(),
//                 [](Track& t)
//                 {
//                     return t.lost_frames > 10; // 👉 最大丢失帧
//                 }),
//             tracks.end()
//         );
//     }

// private:
//     std::vector<Track> tracks;
//     int next_id;

//     cv::Rect2f toRect(const ObjectCameraDetectResult& det)
//     {
//         return cv::Rect2f(
//             det.target_box.left,
//             det.target_box.top,
//             det.target_box.right - det.target_box.left,
//             det.target_box.bottom - det.target_box.top
//         );
//     }

//     float compute_iou(const cv::Rect2f& a, const cv::Rect2f& b)
//     {
//         float xx1 = std::max(a.x, b.x);
//         float yy1 = std::max(a.y, b.y);
//         float xx2 = std::min(a.x + a.width,  b.x + b.width);
//         float yy2 = std::min(a.y + a.height, b.y + b.height);

//         float w = std::max(0.0f, xx2 - xx1);
//         float h = std::max(0.0f, yy2 - yy1);

//         float inter = w * h;
//         float union_area = a.area() + b.area() - inter;

//         return union_area > 0 ? inter / union_area : 0;
//     }
// };