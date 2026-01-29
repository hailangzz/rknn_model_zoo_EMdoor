#include "transform_coordinates.h"
#include <cstring>   // for memset

bool CameraParameters::pixelToCameraXYZGround(float u, float v, single_pixel_camera_coordinates &camera_coordinates){

        
        // 1. 像素 -> 归一化坐标
        float x_n = (u - cx) / fx;
        float y_n = (v - cy) / fy;

        // 2. 绕 X 轴旋转（俯仰角）
        float cos_p = std::cos(pitch);
        float sin_p = std::sin(pitch);

        float Xd = x_n;
        float Yd = cos_p * y_n - sin_p;
        float Zd = sin_p * y_n + cos_p;

        // 3. 射线与地面 Y=H 求交
        if (std::fabs(Yd) < 1e-6) return false;

        float t = H / Yd;
        camera_coordinates.X = t * Xd;
        camera_coordinates.Y = H;
        camera_coordinates.Z = t * Zd;
        printf("float u:%f, float v:%f.\n",u,v);
        printf("camera_coordinates info is  X:%f,Y:%f,Z:%f.\n",camera_coordinates.X,camera_coordinates.Y,camera_coordinates.Z);
        return true;
    }

bool CameraParameters::ObjectboxToCameraXYZ(image_rect_t object_rect, box_camera_coordinates &camera_coordinates){
    printf("enter ObjectboxToCameraXYZ: left=%d, top=%d, right=%d, bottom=%d\n",
           object_rect.left, object_rect.top, object_rect.right, object_rect.bottom);

    if (!pixelToCameraXYZGround(object_rect.left, object_rect.top, camera_coordinates.left_top)) {
        memset(&camera_coordinates.left_top, 0, sizeof(single_pixel_camera_coordinates));
    }
    if (!pixelToCameraXYZGround(object_rect.right, object_rect.top, camera_coordinates.right_top)) {
        memset(&camera_coordinates.right_top, 0, sizeof(single_pixel_camera_coordinates));
    }
    if (!pixelToCameraXYZGround(object_rect.right, object_rect.bottom, camera_coordinates.right_bottom)) {
        memset(&camera_coordinates.right_bottom, 0, sizeof(single_pixel_camera_coordinates));
    }
    if (!pixelToCameraXYZGround(object_rect.left, object_rect.bottom, camera_coordinates.left_bottom)) {
        memset(&camera_coordinates.left_bottom, 0, sizeof(single_pixel_camera_coordinates));
    }

    return true;
}
