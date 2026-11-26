
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

    // 像素 -> 水平面 Y=plane_y 交点（plane_y=0 为地面）
    bool pixelToCameraXYZPlaneY(float u, float v, float plane_y,
                                single_pixel_camera_coordinates &P);

    // 将玻璃底边二维折线投影为三维建图线
    bool projectGlassBottomLine(const std::vector<cv::Point> &bottom_line_px,
                                float plane_y,
                                std::vector<CameraCoordinate> &out_line_xyz);

    // 多高度平面尝试（地面/门缝/窗台），返回有效点最多且深度合理的结果
    bool projectGlassBottomLineAuto(const std::vector<cv::Point> &bottom_line_px,
                                    const ConfigInfo &config,
                                    bool prefer_ground_contact,
                                    std::vector<CameraCoordinate> &out_line_xyz,
                                    float *used_plane_y);

private:

    // 像素坐标 -> 相机坐标（地面假设，Y=0）
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
bool calcObjectSizeByAverage(const ObjectCameraDetectResult &one, ObjectSize3D &size_out);

// 底边线三维长度
float calcBottomLineWidth(const std::vector<CameraCoordinate> &line_xyz);
