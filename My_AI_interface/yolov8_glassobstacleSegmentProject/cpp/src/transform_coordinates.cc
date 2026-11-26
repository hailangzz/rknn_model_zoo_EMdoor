#include "transform_coordinates.h"
#include <cstring>   // for memset

#include <algorithm>
#include <cstdio>
#include <ctime>
#include <vector>

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
bool CameraParameters::pixelToCameraXYZPlaneY(
    float u_scaled,
    float v_scaled,
    float plane_y,
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
    // ================================
    std::vector<cv::Point2f> src(1), dst;
    src[0] = cv::Point2f(u, v);

    cv::undistortPoints(src, dst, K, D);

    const float xn = dst[0].x;
    const float yn = dst[0].y;

    // ================================
    // 4. 构造相机坐标系射线
    // ================================
    const float Xc = xn;
    const float Yc = yn;
    const float Zc = 1.0f;

    // ================================
    // 5. 绕 X 轴做 pitch 旋转
    // ================================
    const float cos_p = std::cos(pitch);
    const float sin_p = std::sin(pitch);

    const float Xw = Xc;
    const float Yw = cos_p * Yc - sin_p * Zc;
    const float Zw = sin_p * Yc + cos_p * Zc;

    // ================================
    // 6. 射线与水平面 Y = plane_y 求交
    // 相机位于 (0, H, 0)；plane_y=0 即地面
    // 注意：与历史地面假设一致，t 可能为负（俯视地面时 Yw>0），
    //       不能用 t<=0 直接判失败，否则会把合法地面交点全部丢掉。
    // ================================
    if (std::fabs(Yw) < 1e-6f)
        return false;

    // H + t * Yw = plane_y  =>  t = (plane_y - H) / Yw
    const float t = (plane_y - H) / Yw;

    camera_coordinates.X = -(t * Xw);
    camera_coordinates.Y = plane_y;
    camera_coordinates.Z = -(t * Zw);

    // 只校验交点是否在相机前方、数值是否合法
    if (!std::isfinite(camera_coordinates.X) ||
        !std::isfinite(camera_coordinates.Z) ||
        camera_coordinates.Z <= 1e-4f)
    {
        return false;
    }
    return true;
}

bool CameraParameters::pixelToCameraXYZGround(
    float u_scaled,
    float v_scaled,
    single_pixel_camera_coordinates &camera_coordinates)
{
    return pixelToCameraXYZPlaneY(u_scaled, v_scaled, 0.0f, camera_coordinates);
}

bool CameraParameters::projectGlassBottomLine(
    const std::vector<cv::Point> &bottom_line_px,
    float plane_y,
    std::vector<CameraCoordinate> &out_line_xyz)
{
    out_line_xyz.clear();
    out_line_xyz.reserve(bottom_line_px.size());

    int fail_cnt = 0;
    for (const auto &pt : bottom_line_px)
    {
        single_pixel_camera_coordinates cam{};
        if (!pixelToCameraXYZPlaneY(
                static_cast<float>(pt.x),
                static_cast<float>(pt.y),
                plane_y,
                cam))
        {
            fail_cnt++;
            continue;
        }
        CameraCoordinate c{};
        c.X = cam.X;
        c.Y = cam.Y;
        c.Z = cam.Z;
        out_line_xyz.push_back(c);
    }

    if (out_line_xyz.size() < 2)
    {
        printf("[glass] projectBottomLine fail: in=%zu ok=%zu fail=%d plane_y=%.3f\n",
               bottom_line_px.size(), out_line_xyz.size(), fail_cnt, plane_y);
        return false;
    }
    return true;
}

bool CameraParameters::projectGlassBottomLineAuto(
    const std::vector<cv::Point> &bottom_line_px,
    const ConfigInfo &config,
    bool prefer_ground_contact,
    std::vector<CameraCoordinate> &out_line_xyz,
    float *used_plane_y)
{
    out_line_xyz.clear();

    // 候选水平面：地面、门缝、窗台（可高于相机，覆盖悬空/高处玻璃底边）
    std::vector<float> planes;
    planes.push_back(0.0f);
    planes.push_back(std::max(0.01f, config.glass_floating_assume_height_m));
    planes.push_back(0.40f);
    planes.push_back(0.80f);
    planes.push_back(1.20f);

    if (prefer_ground_contact)
    {
        // 接地优先：保持 0 在最前即可
    }
    else
    {
        // 悬空优先试门缝/窗台，地面放后面兜底
        planes = {std::max(0.01f, config.glass_floating_assume_height_m),
                  0.40f, 0.80f, 1.20f, 0.0f};
    }

    const float z_min = std::max(0.05f, config.min_detect_distance_m * 0.5f);
    const float z_max = std::max(config.max_detect_distance_m * 2.0f, 8.0f);

    size_t best_count = 0;
    float best_plane = 0.0f;
    std::vector<CameraCoordinate> best_xyz;

    for (float plane_y : planes)
    {
        std::vector<CameraCoordinate> cand;
        if (!projectGlassBottomLine(bottom_line_px, plane_y, cand))
            continue;

        size_t valid = 0;
        for (const auto &p : cand)
        {
            if (std::isfinite(p.Z) && p.Z >= z_min && p.Z <= z_max)
                valid++;
        }

        if (valid > best_count)
        {
            best_count = valid;
            best_plane = plane_y;
            best_xyz.swap(cand);
        }

        // 地面且有效点足够，直接采用
        if (plane_y <= 1e-6f && valid >= static_cast<size_t>(std::max(2, config.min_edge_points)))
            break;
    }

    if (best_count < 2 || best_xyz.size() < 2)
    {
        printf("[glass] projectBottomLineAuto failed, no valid plane\n");
        return false;
    }

    out_line_xyz.swap(best_xyz);
    if (used_plane_y)
        *used_plane_y = best_plane;
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


bool CameraParameters::ObjectboxToCameraXYZ(
    object_detect_result* det,
    const std::vector<std::vector<cv::Point>> &contours_mark_point)
{
    if (!det) return false;

    // printf("det->box.left: %d\n", det->box.left);
    // printf("det->box.top: %d\n", det->box.top);
    // printf("det->box.right: %d\n", det->box.right);
    // printf("det->box.bottom: %d\n", det->box.bottom);

    // ---------- 1. 四个角点 ----------
    if (!pixelToCameraXYZGround(det->box.left, det->box.top, det->camera_coordinates.left_top)) {
        fprintf(stderr, "[ERROR] pixelToCameraXYZGround failed for left_top (%d,%d)\n",
                det->box.left, det->box.top);
    }

    if (!pixelToCameraXYZGround(det->box.right, det->box.top, det->camera_coordinates.right_top)) {
        fprintf(stderr, "[ERROR] pixelToCameraXYZGround failed for right_top (%d,%d)\n",
                det->box.right, det->box.top);
    }

    if (!pixelToCameraXYZGround(det->box.right, det->box.bottom, det->camera_coordinates.right_bottom)) {
        fprintf(stderr, "[ERROR] pixelToCameraXYZGround failed for right_bottom (%d,%d)\n",
                det->box.right, det->box.bottom);
    }

    if (!pixelToCameraXYZGround(det->box.left, det->box.bottom, det->camera_coordinates.left_bottom)) {
        fprintf(stderr, "[ERROR] pixelToCameraXYZGround failed for left_bottom (%d,%d)\n",
                det->box.left, det->box.bottom);
    }

    // printf("camera_coordinates info: "
    //        "left_top: X:%f,Y:%f,Z:%f, "
    //        "right_top: X:%f,Y:%f,Z:%f, "
    //        "right_bottom: X:%f,Y:%f,Z:%f, "
    //        "left_bottom: X:%f,Y:%f,Z:%f\n",
    //        det->camera_coordinates.left_top.X, det->camera_coordinates.left_top.Y, det->camera_coordinates.left_top.Z,
    //        det->camera_coordinates.right_top.X, det->camera_coordinates.right_top.Y, det->camera_coordinates.right_top.Z,
    //        det->camera_coordinates.right_bottom.X, det->camera_coordinates.right_bottom.Y, det->camera_coordinates.right_bottom.Z,
    //        det->camera_coordinates.left_bottom.X, det->camera_coordinates.left_bottom.Y, det->camera_coordinates.left_bottom.Z);

    // ---------- 2. 边缘点 ----------
    // 先清空旧边缘点指针（防止野指针）
    if (det->camera_coordinates.add_edge_point_single_pixel_camera_coordinates) {
        free(det->camera_coordinates.add_edge_point_single_pixel_camera_coordinates);
        det->camera_coordinates.add_edge_point_single_pixel_camera_coordinates = nullptr;
        det->camera_coordinates.add_edge_point_num = 0;
    }

    


    // 统计总边缘点数量
    int total_points = 0;
    for (const auto &contour : contours_mark_point) total_points += static_cast<int>(contour.size());
    if (total_points == 0) return true;

    
    // 分配内存
    det->camera_coordinates.add_edge_point_single_pixel_camera_coordinates =
        (single_pixel_camera_coordinates *)malloc(
            total_points * sizeof(single_pixel_camera_coordinates)
        );
    if (!det->camera_coordinates.add_edge_point_single_pixel_camera_coordinates) {
        fprintf(stderr, "[ERROR] malloc failed for edge points\n");
        return false;
    }


    // 填充边缘点
    det->camera_coordinates.add_edge_point_num = 0;
    for (const auto &contour : contours_mark_point) {
        for (const auto &pt : contour) {
            single_pixel_camera_coordinates cam_pt;
            if (pixelToCameraXYZGround(pt.x, pt.y, cam_pt)) {
                det->camera_coordinates.add_edge_point_single_pixel_camera_coordinates[
                    det->camera_coordinates.add_edge_point_num++
                ] = cam_pt;
            }
        }
    }

    // printf("Edge points count: %d\n", det->camera_coordinates.add_edge_point_num);
    // for (int i = 0; i < det->camera_coordinates.add_edge_point_num; ++i) {
    //     const auto &p = det->camera_coordinates.add_edge_point_single_pixel_camera_coordinates[i];
    //     printf("Edge point %d: X:%f Y:%f Z:%f\n", i, p.X, p.Y, p.Z);
    // }


    // // ---------- 3. 打印左下角信息 ----------
    // // const auto &lb = det->camera_coordinates.left_bottom;
    // if (det->camera_coordinates.left_bottom.X != 0 || det->camera_coordinates.left_bottom.Y != 0 || det->camera_coordinates.left_bottom.Z != 0) {
    //     printf("Left Bottom: X:%f Y:%f Z:%f\n", det->camera_coordinates.left_bottom.X, det->camera_coordinates.left_bottom.Y, det->camera_coordinates.left_bottom.Z);
    // } else {
    //     printf("Left Bottom: invalid\n");
    // }

    // if (det->camera_coordinates.right_bottom.X != 0 || det->camera_coordinates.right_bottom.Y != 0 || det->camera_coordinates.right_bottom.Z != 0) {
    //     printf("Right Bottom: X:%f Y:%f Z:%f\n", det->camera_coordinates.right_bottom.X, det->camera_coordinates.right_bottom.Y, det->camera_coordinates.right_bottom.Z);
    // } else {
    //     printf("Right Bottom: invalid\n");
    // }

    // if (det->camera_coordinates.right_top.X != 0 || det->camera_coordinates.right_top.Y != 0 || det->camera_coordinates.right_top.Z != 0) {
    //     printf("Right Top: X:%f Y:%f Z:%f\n", det->camera_coordinates.right_top.X, det->camera_coordinates.right_top.Y, det->camera_coordinates.right_top.Z);
    // } else {
    //     printf("Right Top: invalid\n");
    // }

    // if (det->camera_coordinates.left_top.X != 0 || det->camera_coordinates.left_top.Y != 0 || det->camera_coordinates.left_top.Z != 0) {
    //     printf("Left Top: X:%f Y:%f Z:%f\n", det->camera_coordinates.left_top.X, det->camera_coordinates.left_top.Y, det->camera_coordinates.left_top.Z);
    // } else {
    //     printf("Left Top: invalid\n");
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
bool calcObjectSizeByAverage(const ObjectCameraDetectResult &one, ObjectSize3D &size_out)
{
    // 优先：底边建图线长度即玻璃宽度
    if (one.add_edge_point_single_pixel_camera_coordinates.size() >= 2)
    {
        size_out.width = calcBottomLineWidth(one.add_edge_point_single_pixel_camera_coordinates);
        size_out.height = 0.0f;
        return size_out.width > 0.0f;
    }

    if (one.coords.size() < 2)
        return false;

    // 兼容：coords[0]/左端, coords[1]=右端
    size_out.width = distance3D(one.coords[0], one.coords[1]);
    size_out.height = 0.0f;
    return std::isfinite(size_out.width) && size_out.width > 0.0f;
}

float calcBottomLineWidth(const std::vector<CameraCoordinate> &line_xyz)
{
    if (line_xyz.size() < 2)
        return -1.0f;

    // 用左右端点直线距离，避免折线绕远/异常点把宽度撑到几百米
    return distance3D(line_xyz.front(), line_xyz.back());
}
