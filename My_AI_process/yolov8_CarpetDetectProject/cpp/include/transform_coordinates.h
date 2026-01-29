#include <cmath>
#include <iostream>
#include "common.h"
#include "types.h"

class CameraParameters {
public:
    // 构造函数
    CameraParameters(const ConfigInfo& config)
        : fx(config.camera_fx), fy(config.camera_fy), cx(config.camera_cx), cy(config.camera_cy), H(config.camera_H), pitch(config.camera_pitch) {}

    bool ObjectboxToCameraXYZ(image_rect_t object_rect, box_camera_coordinates &camera_coordinates);

private:
    // 像素坐标 -> 相机坐标（地面假设）
    bool pixelToCameraXYZGround(float u, float v, single_pixel_camera_coordinates &camera_coordinates);
    // 相机内参
    float fx, fy;
    float cx, cy;

    // 安装参数
    float H;      // 高度
    float pitch;  // 俯仰角
};
