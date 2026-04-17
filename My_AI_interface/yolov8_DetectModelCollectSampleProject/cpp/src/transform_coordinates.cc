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

//// 由于图像resize，导致的检测坐标，转XYZ相机坐标时，目标框物体变小。
bool CameraParameters::pixelToCameraXYZGround(
    float u_scaled,
    float v_scaled,
    single_pixel_camera_coordinates &camera_coordinates)
{
    // ================================
    // 1. 检测图 → 原图 像素映射
    // （仅在等比例 resize 时成立）
    // ================================
    const float scale_x =
        static_cast<float>(det_width) / orig_width;
    const float scale_y =
        static_cast<float>(det_height) / orig_height;

    const float u = u_scaled / scale_x;
    const float v = v_scaled / scale_y;

    // ================================
    // 2. 使用“局部内参副本”（严禁改成员变量）
    // ================================
    const float fx_ = fx / scale_x;
    const float fy_ = fy / scale_y;
    const float cx_ = cx / scale_x;
    const float cy_ = cy / scale_y;

    const cv::Mat K = (cv::Mat_<double>(3,3) <<
        fx_,  0.0, cx_,
        0.0, fy_,  cy_,
        0.0, 0.0,  1.0);

    const cv::Mat D = (cv::Mat_<double>(1,8) <<
        D_0, D_1, D_2, D_3,
        D_4, D_5, D_6, D_7);

    // ================================
    // 3. 去畸变 → 归一化相机坐标
    // dst: (X/Z, Y/Z)
    // ================================
    std::vector<cv::Point2f> src(1), dst;
    src[0] = cv::Point2f(u, v);

    cv::undistortPoints(src, dst, K, D);

    const float xn = dst[0].x;
    const float yn = dst[0].y;

    // ================================
    // 4. 构造相机坐标系单位射线
    // ================================
    float Xc = xn;
    float Yc = yn;
    float Zc = 1.0f;

    // ================================
    // 5. 绕 X 轴做 pitch 旋转
    // （相机 → 世界）
    // ================================
    const float cos_p = std::cos(pitch);
    const float sin_p = std::sin(pitch);

    const float Xw = Xc;
    const float Yw =  cos_p * Yc - sin_p * Zc;
    const float Zw =  sin_p * Yc + cos_p * Zc;

    // ================================
    // 6. 射线与地面 Y = 0 求交
    // 相机位于 (0, H, 0)
    // ================================
    // if (Yw >= -1e-4f) {
    //     // 射线朝上或近平行地面
    //     return false;
    // }

    const float t = -H / Yw;

    // ================================
    // 7. 计算交点（世界 / 相机前向坐标）
    // ================================
    camera_coordinates.X = -(t * Xw);
    camera_coordinates.Y = 0.0f;
    camera_coordinates.Z = -(t * Zw);

    return true;
}


// //当前可用转换代码；
// bool CameraParameters::pixelToCameraXYZGround(float u, float v,single_pixel_camera_coordinates &camera_coordinates)
//     {
//         printf("enter CameraParameters::pixelToCameraXYZGround\n");

//         printf("[Distortion]\n");
//         printf("k1=%.8f k2=%.8f p1=%.8f p2=%.8f\n",
//                 D_0, D_1, D_2, D_3);
//         printf("k3=%.8f k4=%.8f k5=%.8f k6=%.8f\n",
//                 D_4, D_5, D_6, D_7);

//         printf("[Extrinsic]\n");
//         printf("H=%.6f pitch=%.6f rad (%.2f deg)\n",
//                 H, pitch, pitch * 180.0 / 3.1415926);

//         printf("[Camera XYZ on Ground]\n");
//         printf("X=%.6f Y=%.6f Z=%.6f\n", camera_coordinates.X, camera_coordinates.Y, camera_coordinates.Z);

//         printf("----------------------------------------\n");

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

        
    // printf("enter ObjectboxToCameraXYZ: left=%d, top=%d, right=%d, bottom=%d\n",
    //        object_rect.left, object_rect.top, object_rect.right, object_rect.bottom);

    if (!pixelToCameraXYZGround(object_rect.left, object_rect.top, camera_coordinates.left_top)) {
        memset(&camera_coordinates.left_top, 0, sizeof(single_pixel_camera_coordinates));
        fprintf(stderr, "[ERROR] pixelToCameraXYZGround failed for left_top (%d,%d)\n",
                object_rect.left, object_rect.top);
    }

    if (!pixelToCameraXYZGround(object_rect.right, object_rect.top, camera_coordinates.right_top)) {
        memset(&camera_coordinates.right_top, 0, sizeof(single_pixel_camera_coordinates));
        fprintf(stderr, "[ERROR] pixelToCameraXYZGround failed for right_top (%d,%d)\n",
                object_rect.right, object_rect.top);
    }
    if (!pixelToCameraXYZGround(object_rect.right, object_rect.bottom, camera_coordinates.right_bottom)) {
        memset(&camera_coordinates.right_bottom, 0, sizeof(single_pixel_camera_coordinates));
        fprintf(stderr, "[ERROR] pixelToCameraXYZGround failed for right_bottom (%d,%d)\n",
                object_rect.right, object_rect.bottom);
    }
    if (!pixelToCameraXYZGround(object_rect.left, object_rect.bottom, camera_coordinates.left_bottom)) {
        memset(&camera_coordinates.left_bottom, 0, sizeof(single_pixel_camera_coordinates));
        fprintf(stderr, "[ERROR] pixelToCameraXYZGround failed for left_bottom (%d,%d)\n",
                object_rect.left, object_rect.bottom);
    }


    // ------------------- 固定数组采样 -------------------
    const int edge_sample_num = 5; // 每条边采样点数量（不含端点）
    int idx = 0; // 当前数组索引

    auto sample_edge = [&](int x1, int y1, int x2, int y2) {
        for (int i = 1; i <= edge_sample_num && idx < 20; ++i) {
            float t = static_cast<float>(i) / (edge_sample_num + 1);
            int u = static_cast<int>(x1 + t * (x2 - x1));
            int v = static_cast<int>(y1 + t * (y2 - y1));

            single_pixel_camera_coordinates cam_pt;
            if (pixelToCameraXYZGround(u, v, cam_pt)) {
                camera_coordinates.add_edge_point_single_pixel_camera_coordinates[idx++] = cam_pt;
            }
        }
    };

    //上边
    sample_edge(object_rect.left, object_rect.top,
                object_rect.right, object_rect.top);

    // 右边
    sample_edge(object_rect.right, object_rect.top,
                object_rect.right, object_rect.bottom);

    // 下边
    sample_edge(object_rect.right, object_rect.bottom,
                object_rect.left, object_rect.bottom);

    // 左边
    sample_edge(object_rect.left, object_rect.bottom,
                object_rect.left, object_rect.top);

    
    // // ------------------- 打印 camera_coordinates -------------------
    // printf("Corner points:\n");  //Z轴为原始尺寸时
    // printf("  left_top     : X=%.3f, Y=%.3f, Z=%.3f\n",
    //        camera_coordinates.left_top.X,
    //        camera_coordinates.left_top.Y,
    //        camera_coordinates.left_top.Z);
    // printf("  right_top    : X=%.3f, Y=%.3f, Z=%.3f\n",
    //        camera_coordinates.right_top.X,
    //        camera_coordinates.right_top.Y,
    //        camera_coordinates.right_top.Z);
    // printf("  right_bottom : X=%.3f, Y=%.3f, Z=%.3f\n",
    //        camera_coordinates.right_bottom.X,
    //        camera_coordinates.right_bottom.Y,
    //        camera_coordinates.right_bottom.Z);
    // printf("  left_bottom  : X=%.3f, Y=%.3f, Z=%.3f\n",
    //        camera_coordinates.left_bottom.X,
    //        camera_coordinates.left_bottom.Y,
    //        camera_coordinates.left_bottom.Z);

    // printf("Edge sample points (idx = 0 ~ %d):\n", idx-1);
    // for (int i = 0; i < idx; ++i) {
    //     auto& p = camera_coordinates.add_edge_point_single_pixel_camera_coordinates[i];
    //     printf("  [%2d] X=%.3f, Y=%.3f, Z=%.3f\n", i, p.X, p.Y, p.Z);
    // }


    return true;
}



static inline float distance3D(const CameraCoordinate& a, const CameraCoordinate& b)
{
    float dx = a.X - b.X;
    float dy = a.Y - b.Y;
    float dz = a.Z - b.Z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 计算物体宽度、高度
bool calcObjectSizeByAverage(ObjectCameraDetectResult& one,ObjectSize3D& size_out)
{
    // 四个顶点
    const CameraCoordinate& lt = one.coords[0]; // left_top
    const CameraCoordinate& rt = one.coords[1]; // right_top
    const CameraCoordinate& rb = one.coords[2]; // right_bottom
    const CameraCoordinate& lb = one.coords[3]; // left_bottom

    // 宽度（上边 & 下边）
    float width_top    = distance3D(lt, rt);
    float width_bottom = distance3D(lb, rb);

    // 高度（左边 & 右边）
    float height_left  = distance3D(lt, lb);
    float height_right = distance3D(rt, rb);

    // 平均
    size_out.width  = (width_top + width_bottom) * 0.5f;
    size_out.height = (height_left + height_right) * 0.5f;

    // 基本合法性检查
    if (size_out.width <= 0.0f || size_out.height <= 0.0f) {
        return false;
    }

    return true;
}
