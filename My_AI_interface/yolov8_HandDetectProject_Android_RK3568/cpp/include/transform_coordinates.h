
#include <cmath>
#include <iostream>
#include "common.h"
#include "types.h"
#include <opencv2/opencv.hpp>

class CameraParameters {
public:
    // 构造函数
    CameraParameters(const ConfigInfo& config)
        : fx(config.camera_fx), fy(config.camera_fy), cx(config.camera_cx), cy(config.camera_cy), H(config.camera_H), pitch(config.camera_pitch),
          D_0(config.camera_D_0),D_1(config.camera_D_1),D_2(config.camera_D_2),D_3(config.camera_D_3),D_4(config.camera_D_4),D_5(config.camera_D_5),D_6(config.camera_D_6),D_7(config.camera_D_7){}

    bool ObjectboxToCameraXYZ(image_rect_t object_rect, box_camera_coordinates &camera_coordinates);

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
