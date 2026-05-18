#include "transform_coordinates.h"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <cstring>
#include <limits>
#include <opencv2/opencv.hpp>

static inline bool isFiniteFloat(float v)
{
    return std::isfinite(v);
}

static void appendCameraLog(float u, float v,
                            float fx, float fy, float cx, float cy,
                            float D_0, float D_1, float D_2, float D_3,
                            float D_4, float D_5, float D_6, float D_7,
                            float H, float pitch,
                            float X, float Y, float Z)
{
    FILE *fp = fopen("/home/robot/zhangzhuo/catkin_ws/logs/camera_xyz_debug.log", "a");

    if (!fp)
        return;

    std::time_t t = std::time(nullptr);
    std::tm *tm = std::localtime(&t);

    fprintf(fp,
            "\n[%04d-%02d-%02d %02d:%02d:%02d]\n",
            tm->tm_year + 1900,
            tm->tm_mon + 1,
            tm->tm_mday,
            tm->tm_hour,
            tm->tm_min,
            tm->tm_sec);

    fprintf(fp, "Pixel: u=%.2f v=%.2f\n", u, v);

    fprintf(fp, "[Intrinsic]\n");
    fprintf(fp,
            "fx=%.6f fy=%.6f cx=%.6f cy=%.6f\n",
            fx, fy, cx, cy);

    fprintf(fp, "[Distortion]\n");
    fprintf(fp,
            "k1=%.8f k2=%.8f p1=%.8f p2=%.8f\n",
            D_0, D_1, D_2, D_3);

    fprintf(fp,
            "k3=%.8f k4=%.8f k5=%.8f k6=%.8f\n",
            D_4, D_5, D_6, D_7);

    fprintf(fp, "[Extrinsic]\n");

    fprintf(fp,
            "H=%.6f pitch=%.6f rad (%.2f deg)\n",
            H,
            pitch,
            pitch * 180.0 / 3.1415926);

    fprintf(fp, "[Camera XYZ on Ground]\n");

    fprintf(fp,
            "X=%.6f Y=%.6f Z=%.6f\n",
            X,
            Y,
            Z);

    fprintf(fp,
            "----------------------------------------\n");

    fflush(fp);
    fclose(fp);
}

bool CameraParameters::pixelToCameraXYZGround(
    float u_scaled,
    float v_scaled,
    single_pixel_camera_coordinates &camera_coordinates)
{
    // =========================================================
    // 0. 参数合法性检查
    // =========================================================
    if (orig_width <= 0 || orig_height <= 0)
    {
        fprintf(stderr,
                "[ERROR] invalid original image size: %d x %d\n",
                orig_width,
                orig_height);
        return false;
    }

    if (det_width <= 0 || det_height <= 0)
    {
        fprintf(stderr,
                "[ERROR] invalid detect image size: %d x %d\n",
                det_width,
                det_height);
        return false;
    }

    if (std::fabs(fx) < 1e-6f ||
        std::fabs(fy) < 1e-6f)
    {
        fprintf(stderr,
                "[ERROR] invalid fx/fy\n");
        return false;
    }

    // =========================================================
    // 1. resize 比例
    // =========================================================
    const float scale_x =
        static_cast<float>(det_width) /
        static_cast<float>(orig_width);

    const float scale_y =
        static_cast<float>(det_height) /
        static_cast<float>(orig_height);

    if (std::fabs(scale_x) < 1e-6f ||
        std::fabs(scale_y) < 1e-6f)
    {
        fprintf(stderr,
                "[ERROR] invalid scale_x/scale_y\n");
        return false;
    }

    // =========================================================
    // 2. 检测图 -> 原图
    // =========================================================
    const float u = u_scaled / scale_x;
    const float v = v_scaled / scale_y;

    if (!isFiniteFloat(u) || !isFiniteFloat(v))
    {
        fprintf(stderr,
                "[ERROR] invalid mapped uv\n");
        return false;
    }

    // =========================================================
    // 3. 构建局部内参
    // =========================================================
    const float fx_ = fx / scale_x;
    const float fy_ = fy / scale_y;
    const float cx_ = cx / scale_x;
    const float cy_ = cy / scale_y;

    cv::Mat K =
        (cv::Mat_<double>(3, 3)
             << fx_,
         0.0, cx_,
         0.0, fy_, cy_,
         0.0, 0.0, 1.0);

    cv::Mat D =
        (cv::Mat_<double>(1, 8)
             << D_0,
         D_1, D_2, D_3,
         D_4, D_5, D_6, D_7);

    // =========================================================
    // 4. 去畸变
    // =========================================================
    std::vector<cv::Point2f> src(1), dst;

    src[0] = cv::Point2f(u, v);

    try
    {
        cv::undistortPoints(src, dst, K, D);
    }
    catch (const cv::Exception &e)
    {
        fprintf(stderr,
                "[ERROR] undistortPoints exception: %s\n",
                e.what());

        return false;
    }

    if (dst.empty())
    {
        fprintf(stderr,
                "[ERROR] undistortPoints output empty\n");

        return false;
    }

    const float xn = dst[0].x;
    const float yn = dst[0].y;

    if (!isFiniteFloat(xn) ||
        !isFiniteFloat(yn))
    {
        fprintf(stderr,
                "[ERROR] invalid undistort result\n");

        return false;
    }

    // =========================================================
    // 5. 相机坐标系单位射线
    // =========================================================
    const float Xc = xn;
    const float Yc = yn;
    const float Zc = 1.0f;

    // =========================================================
    // 6. pitch 旋转
    // =========================================================
    const float cos_p = std::cos(pitch);
    const float sin_p = std::sin(pitch);

    const float Xw = Xc;

    const float Yw =
        cos_p * Yc - sin_p * Zc;

    const float Zw =
        sin_p * Yc + cos_p * Zc;

    if (!isFiniteFloat(Yw))
    {
        fprintf(stderr,
                "[ERROR] invalid Yw\n");

        return false;
    }

    // =========================================================
    // 7. 与地面求交
    // =========================================================
    if (std::fabs(Yw) < 1e-6f)
    {
        fprintf(stderr,
                "[ERROR] Yw too small\n");

        return false;
    }

    const float t = -H / Yw;

    if (!isFiniteFloat(t))
    {
        fprintf(stderr,
                "[ERROR] invalid t\n");

        return false;
    }

    // =========================================================
    // 8. 输出结果
    // =========================================================
    camera_coordinates.X = -(t * Xw);
    camera_coordinates.Y = 0.0f;
    camera_coordinates.Z = -(t * Zw);

    if (!isFiniteFloat(camera_coordinates.X) ||
        !isFiniteFloat(camera_coordinates.Y) ||
        !isFiniteFloat(camera_coordinates.Z))
    {
        fprintf(stderr,
                "[ERROR] invalid camera xyz\n");

        return false;
    }

    appendCameraLog(
        u,
        v,
        fx_,
        fy_,
        cx_,
        cy_,
        D_0,
        D_1,
        D_2,
        D_3,
        D_4,
        D_5,
        D_6,
        D_7,
        H,
        pitch,
        camera_coordinates.X,
        camera_coordinates.Y,
        camera_coordinates.Z);

    return true;
}

bool CameraParameters::XYZGroundTopixel(
    single_pixel_camera_coordinates P,
    float &u,
    float &v)
{
    if (std::fabs(P.Z) < 1e-6f)
    {
        fprintf(stderr,
                "[ERROR] P.Z too small\n");

        return false;
    }

    float k1 = D_0;
    float k2 = D_1;
    float p1 = D_2;
    float p2 = D_3;
    float k3 = D_4;
    float k4 = D_5;
    float k5 = D_6;
    float k6 = D_7;

    // =========================================================
    // 归一化坐标
    // =========================================================
    float x = P.X / P.Z;
    float y = P.Y / P.Z;

    float r2 = x * x + y * y;
    float r4 = r2 * r2;
    float r6 = r4 * r2;
    float r8 = r6 * r2;
    float r10 = r8 * r2;
    float r12 = r10 * r2;

    // =========================================================
    // 径向畸变
    // =========================================================
    float radial_factor =
        1 +
        k1 * r2 +
        k2 * r4 +
        k3 * r6 +
        k4 * r8 +
        k5 * r10 +
        k6 * r12;

    float x_radial = x * radial_factor;
    float y_radial = y * radial_factor;

    // =========================================================
    // 切向畸变
    // =========================================================
    float x_prime =
        x_radial +
        2 * p1 * x * y +
        p2 * (r2 + 2 * x * x);

    float y_prime =
        y_radial +
        p1 * (r2 + 2 * y * y) +
        2 * p2 * x * y;

    u = fx * x_prime + cx;
    v = fy * y_prime + cy;

    if (!isFiniteFloat(u) ||
        !isFiniteFloat(v))
    {
        fprintf(stderr,
                "[ERROR] invalid pixel uv\n");

        return false;
    }

    return true;
}

bool CameraParameters::ObjectboxToCameraXYZ(
    image_rect_t object_rect,
    box_camera_coordinates &camera_coordinates)
{
    // =========================================================
    // 1. bbox 合法性检查
    // =========================================================
    if (object_rect.left >= object_rect.right ||
        object_rect.top >= object_rect.bottom)
    {
        fprintf(stderr,
                "[ERROR] invalid bbox\n");

        return false;
    }

    // =========================================================
    // 2. 初始化
    // =========================================================
    camera_coordinates.left_top = {};
    camera_coordinates.right_top = {};
    camera_coordinates.right_bottom = {};
    camera_coordinates.left_bottom = {};

    // =========================================================
    // 3. 四角坐标转换
    // =========================================================
    if (!pixelToCameraXYZGround(
            object_rect.left,
            object_rect.top,
            camera_coordinates.left_top))
    {
        fprintf(stderr,
                "[ERROR] left_top failed\n");
    }

    if (!pixelToCameraXYZGround(
            object_rect.right,
            object_rect.top,
            camera_coordinates.right_top))
    {
        fprintf(stderr,
                "[ERROR] right_top failed\n");
    }

    if (!pixelToCameraXYZGround(
            object_rect.right,
            object_rect.bottom,
            camera_coordinates.right_bottom))
    {
        fprintf(stderr,
                "[ERROR] right_bottom failed\n");
    }

    if (!pixelToCameraXYZGround(
            object_rect.left,
            object_rect.bottom,
            camera_coordinates.left_bottom))
    {
        fprintf(stderr,
                "[ERROR] left_bottom failed\n");
    }

    // =========================================================
    // 4. 边缘采样
    // =========================================================
    constexpr int edge_sample_num = 5;

    constexpr int MAX_EDGE_POINTS =
        sizeof(camera_coordinates.add_edge_point_single_pixel_camera_coordinates) /
        sizeof(single_pixel_camera_coordinates);

    int idx = 0;

    auto sample_edge =
        [&](int x1, int y1, int x2, int y2)
    {
        for (int i = 1;
             i <= edge_sample_num &&
             idx < MAX_EDGE_POINTS;
             ++i)
        {
            float t =
                static_cast<float>(i) /
                (edge_sample_num + 1);

            int u =
                static_cast<int>(
                    x1 + t * (x2 - x1));

            int v =
                static_cast<int>(
                    y1 + t * (y2 - y1));

            single_pixel_camera_coordinates cam_pt{};

            if (pixelToCameraXYZGround(
                    u,
                    v,
                    cam_pt))
            {
                camera_coordinates
                    .add_edge_point_single_pixel_camera_coordinates[idx++] =
                    cam_pt;
            }
        }
    };

    // 上边
    sample_edge(
        object_rect.left,
        object_rect.top,
        object_rect.right,
        object_rect.top);

    // 右边
    sample_edge(
        object_rect.right,
        object_rect.top,
        object_rect.right,
        object_rect.bottom);

    // 下边
    sample_edge(
        object_rect.right,
        object_rect.bottom,
        object_rect.left,
        object_rect.bottom);

    // 左边
    sample_edge(
        object_rect.left,
        object_rect.bottom,
        object_rect.left,
        object_rect.top);

    // =========================================================
    // 5. debug 打印
    // =========================================================
    printf("\n========== Camera Coordinates ==========\n");

    printf("left_top     : X=%.3f Y=%.3f Z=%.3f\n",
           camera_coordinates.left_top.X,
           camera_coordinates.left_top.Y,
           camera_coordinates.left_top.Z);

    printf("right_top    : X=%.3f Y=%.3f Z=%.3f\n",
           camera_coordinates.right_top.X,
           camera_coordinates.right_top.Y,
           camera_coordinates.right_top.Z);

    printf("right_bottom : X=%.3f Y=%.3f Z=%.3f\n",
           camera_coordinates.right_bottom.X,
           camera_coordinates.right_bottom.Y,
           camera_coordinates.right_bottom.Z);

    printf("left_bottom  : X=%.3f Y=%.3f Z=%.3f\n",
           camera_coordinates.left_bottom.X,
           camera_coordinates.left_bottom.Y,
           camera_coordinates.left_bottom.Z);

    printf("========================================\n");

    return true;
}

static inline float distance3D(
    const CameraCoordinate &a,
    const CameraCoordinate &b)
{
    float dx = a.X - b.X;
    float dy = a.Y - b.Y;
    float dz = a.Z - b.Z;

    return std::sqrt(
        dx * dx +
        dy * dy +
        dz * dz);
}

bool calcObjectSizeByAverage(
    ObjectCameraDetectResult &one,
    ObjectSize3D &size_out)
{
    const CameraCoordinate &lt = one.coords[0];
    const CameraCoordinate &rt = one.coords[1];
    const CameraCoordinate &rb = one.coords[2];
    const CameraCoordinate &lb = one.coords[3];

    float width_top =
        distance3D(lt, rt);

    float width_bottom =
        distance3D(lb, rb);

    float height_left =
        distance3D(lt, lb);

    float height_right =
        distance3D(rt, rb);

    size_out.width =
        (width_top + width_bottom) * 0.5f;

    size_out.height =
        (height_left + height_right) * 0.5f;

    if (!isFiniteFloat(size_out.width) ||
        !isFiniteFloat(size_out.height))
    {
        fprintf(stderr,
                "[ERROR] invalid object size\n");

        return false;
    }

    if (size_out.width <= 0.0f ||
        size_out.height <= 0.0f)
    {
        fprintf(stderr,
                "[ERROR] object size <= 0\n");

        return false;
    }

    return true;
}