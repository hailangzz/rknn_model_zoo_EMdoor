#include "transform_coordinates.h"
#include <cstring>   // for memset


#include <cstdio>
#include <ctime>

static void appendCameraLog(float u, float v,
                            float fx, float fy, float cx, float cy,
                            float D_0, float D_1, float D_2, float D_3,
                            float D_4, float D_5, float D_6, float D_7,
                            float H, float pitch,
                            float X, float Y, float Z)
{
    FILE* fp = fopen("/home/robot/zhangzhuo/catkin_ws/logs/camera_xyz_debug.log", "a");
    if (!fp)
        return;

    // 时间戳（方便你区分多次调用）
    std::time_t t = std::time(nullptr);
    std::tm* tm = std::localtime(&t);

    fprintf(fp,
        "\n[%04d-%02d-%02d %02d:%02d:%02d]\n",
        tm->tm_year + 1900,
        tm->tm_mon + 1,
        tm->tm_mday,
        tm->tm_hour,
        tm->tm_min,
        tm->tm_sec
    );

    fprintf(fp, "Pixel: u=%.2f v=%.2f\n", u, v);

    fprintf(fp, "[Intrinsic]\n");
    fprintf(fp, "fx=%.6f fy=%.6f cx=%.6f cy=%.6f\n",
            fx, fy, cx, cy);

    fprintf(fp, "[Distortion]\n");
    fprintf(fp, "k1=%.8f k2=%.8f p1=%.8f p2=%.8f\n",
            D_0, D_1, D_2, D_3);
    fprintf(fp, "k3=%.8f k4=%.8f k5=%.8f k6=%.8f\n",
            D_4, D_5, D_6, D_7);

    fprintf(fp, "[Extrinsic]\n");
    fprintf(fp, "H=%.6f pitch=%.6f rad (%.2f deg)\n",
            H, pitch, pitch * 180.0 / 3.1415926);

    fprintf(fp, "[Camera XYZ on Ground]\n");
    fprintf(fp, "X=%.6f Y=%.6f Z=%.6f\n", X, Y, Z);

    fprintf(fp, "----------------------------------------\n");

    fflush(fp);
    fclose(fp);
}


// bool CameraParameters::pixelToCameraXYZGround(float u, float v, single_pixel_camera_coordinates &camera_coordinates){

//         // 1. 像素 -> 归一化坐标
//         float x_n = (u - cx) / fx;
//         float y_n = (v - cy) / fy;

//         // 2. 绕 X 轴旋转（俯仰角）
//         float cos_p = std::cos(pitch);
//         float sin_p = std::sin(pitch);

//         float Xd = x_n;
//         float Yd = cos_p * y_n - sin_p;
//         float Zd = sin_p * y_n + cos_p;

//         // 3. 射线与地面 Y=H 求交
//         if (std::fabs(Yd) < 1e-6) return false;

//         float t = H / Yd;
//         camera_coordinates.X = t * Xd;
//         camera_coordinates.Y = H;
//         camera_coordinates.Z = t * Zd;
//         printf("float u:%f, float v:%f.\n",u,v);
//         printf("camera_coordinates info is  X:%f,Y:%f,Z:%f.\n",camera_coordinates.X,camera_coordinates.Y,camera_coordinates.Z);
//         return true;
//     }

// 由于图像resize，导致的检测坐标，转XYZ相机坐标时，目标框物体变小。
bool CameraParameters::pixelToCameraXYZGround(float u_scaled, float v_scaled, single_pixel_camera_coordinates &camera_coordinates) {
        // ================================
        // 1. 缩放比例
        // ================================
        float scale_x = static_cast<float>(det_width) / orig_width;
        float scale_y = static_cast<float>(det_height) / orig_height;


        // 映射回原始图像像素坐标
        float u = u_scaled / scale_x;
        float v = v_scaled / scale_y;

        fx = fx / scale_x;
        fy = fy / scale_y;
        cx = cx / scale_x;
        cy = cy / scale_y;

        // ================================
        // 2. 组装相机内参矩阵 K
        // ================================
        cv::Mat K = (cv::Mat_<double>(3,3) <<
            fx,  0,  cx,
            0,  fy,  cy,
            0,  0,   1);

        // ================================
        // 3. 组装畸变参数（rational model）
        // ================================
        cv::Mat D = (cv::Mat_<double>(1,8) <<
            D_0, D_1, D_2, D_3,
            D_4, D_5, D_6, D_7);

        // ================================
        // 4. 像素 → 去畸变 → 归一化相机坐标
        // ================================
        std::vector<cv::Point2f> src(1), dst;
        src[0] = cv::Point2f(u, v);

        // 输出 dst 是归一化相机坐标 (x, y)
        cv::undistortPoints(src, dst, K, D);

        float x = dst[0].x;
        float y = dst[0].y;

        // ================================
        // 5. 构造相机坐标系射线
        // ================================
        float Xc = x;
        float Yc = y;
        float Zc = 1.0f;

        // ================================
        // 6. 绕 X 轴做 pitch 旋转
        // ================================
        float cos_p = std::cos(pitch);
        float sin_p = std::sin(pitch);

        float Xd = Xc;
        float Yd =  cos_p * Yc - sin_p * Zc;
        float Zd =  sin_p * Yc + cos_p * Zc;

        // ================================
        // 7. 射线与地面 Y = 0 求交
        // ================================
        if (Yd >= -1e-6f) {
            // 射线朝上或平行地面，看不到地面
            return false;
        }

        float t = -H / Yd;

        camera_coordinates.X = t * Xd;
        camera_coordinates.Y = 0.0f;
        camera_coordinates.Z = -(t * Zd);

        return true;
    }

// 当前可用转换代码；
// bool CameraParameters::pixelToCameraXYZGround(float u, float v,single_pixel_camera_coordinates &camera_coordinates)
//     {
//         // ================================
//         // 1. 组装相机内参矩阵 K
//         // ================================
//         cv::Mat K = (cv::Mat_<double>(3,3) <<
//             fx,  0,  cx,
//             0, fy,  cy,
//             0,  0,   1);

//         // ================================
//         // 2. 组装畸变参数（rational model）
//         // ================================
//         cv::Mat D = (cv::Mat_<double>(1,8) <<
//             D_0, D_1, D_2, D_3,
//             D_4, D_5, D_6, D_7);

//         // ================================
//         // 3. 像素 → 去畸变 → 归一化坐标
//         // ================================
//         std::vector<cv::Point2f> src(1), dst;
//         src[0] = cv::Point2f(u, v);

//         // 输出 dst 是归一化相机坐标 (x, y)
//         cv::undistortPoints(src, dst, K, D);

//         float x = dst[0].x;
//         float y = dst[0].y;

//         // ================================
//         // 4. 构造相机坐标系下的射线
//         // ================================
//         float Xc = x;
//         float Yc = y;
//         float Zc = 1.0f;

//         // ================================
//         // 5. 绕 X 轴做 pitch 旋转
//         // ================================
//         float cos_p = std::cos(pitch);
//         float sin_p = std::sin(pitch);

//         float Xd = Xc;
//         float Yd =  cos_p * Yc - sin_p * Zc;
//         float Zd =  sin_p * Yc + cos_p * Zc;

//         // ================================
//         // 6. 与地面 Y = 0 求交
//         //    相机在 (0, H, 0)
//         // ================================
//         // if (Yd >= -1e-6f)
//         // {
//         //     // 射线朝上或平行地面，看不到地面
//         //     return false;
//         // }

//         float t = -H / Yd;

//         camera_coordinates.X = t * Xd;
//         camera_coordinates.Y = 0.0f;
//         camera_coordinates.Z = -(t * Zd);


//         // float reconver_pixel_u,reconver_pixel_v;
//         // XYZGroundTopixel(camera_coordinates,reconver_pixel_u,reconver_pixel_v);

        

//         // printf("Pixel: u=%.2f v=%.2f\n", u, v);
//         // printf("[Intrinsic]\n");

//         // printf("reconver Pixel: reconver_pixel_u=%.2f reconver_pixel_v=%.2f\n", reconver_pixel_u, reconver_pixel_v);
//         // printf("[Intrinsic]\n");

        
//         // printf("fx=%.6f fy=%.6f cx=%.6f cy=%.6f\n",
//         //         fx, fy, cx, cy);

//         // printf("[Distortion]\n");
//         // printf("k1=%.8f k2=%.8f p1=%.8f p2=%.8f\n",
//         //         D_0, D_1, D_2, D_3);
//         // printf("k3=%.8f k4=%.8f k5=%.8f k6=%.8f\n",
//         //         D_4, D_5, D_6, D_7);

//         // printf("[Extrinsic]\n");
//         // printf("H=%.6f pitch=%.6f rad (%.2f deg)\n",
//         //         H, pitch, pitch * 180.0 / 3.1415926);

//         // printf("[Camera XYZ on Ground]\n");
//         // printf("X=%.6f Y=%.6f Z=%.6f\n", camera_coordinates.X, camera_coordinates.Y, camera_coordinates.Z);

//         // printf("----------------------------------------\n");

//         return true;
//     }

bool CameraParameters::XYZGroundTopixel(single_pixel_camera_coordinates P,float &u, float &v)
{
    float k1 = D_0;
    float k2 = D_1;
    float p1 = D_2;
    float p2 = D_3;
    float k3 = D_4;
    float k4 = D_5;
    float k5 = D_6;
    float k6 = D_7;

    // 归一化坐标
    float x = P.X / P.Z;
    float y = P.Y / P.Z;
    float r2 = x * x + y * y;
    float r4 = r2 * r2;
    float r6 = r4 * r2;
    float r8 = r6 * r2;
    float r10 = r8 * r2;
    float r12 = r10 * r2;

    // 径向畸变
    float radial_factor =
        1 + k1 * r2 + k2 * r4 + k3 * r6 + k4 * r8 + k5 * r10 + k6 * r12;
    float x_radial = x * radial_factor;
    float y_radial = y * radial_factor;

    // 切向畸变
    float x_prime = x_radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    float y_prime = y_radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
    u = fx * x_prime + cx;
    v = fy * y_prime + cy;

    return true;

}




// bool CameraParameters::pixelToCameraXYZGround(
//     float u, float v,
//     single_pixel_camera_coordinates &P)
// {
//     // 1. 像素 → 归一化相机坐标
//     float x = (u - cx) / fx;
//     float y = (v - cy) / fy;

//     // 相机坐标下的初始射线方向
//     // OpenCV: X右 Y下 Z前
//     float Xc = x;
//     float Yc = y;
//     float Zc = 1.0f;

//     // 2. 绕 X 轴旋转（pitch，向下为负）
//     float cos_p = std::cos(pitch);
//     float sin_p = std::sin(pitch);

//     float Xd = Xc;
//     float Yd =  cos_p * Yc - sin_p * Zc;
//     float Zd =  sin_p * Yc + cos_p * Zc;

//     // 3. 与地面（Y = H）求交
//     if (Yd <= 1e-6f)
//         return false;  // 射线不与地面相交（看天/平行）

//     float t = H / Yd;

//     P.X = t * Xd;
//     P.Y = H;
//     P.Z = t * Zd;

//     return true;
// }


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










