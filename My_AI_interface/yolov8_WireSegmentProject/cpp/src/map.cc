#include "map.h"

#include <cmath>

namespace visual_localization
{

    PoseSampler::PoseSampler() {}

    double PoseSampler::QuaternionToYaw(
        const Eigen::Quaterniond &q)
    {
        Eigen::Vector3d euler =
            q.toRotationMatrix()
                .eulerAngles(0, 1, 2);

        return euler[2];
    }

    PoseKey PoseSampler::ComputePoseKey(
        const Eigen::Vector3d &position,
        double yaw)
    {
        PoseKey key;

        key.x =
            std::round(
                position.x() /
                kDistanceResolution);

        key.y =
            std::round(
                position.y() /
                kDistanceResolution);

        double normalized_yaw =
            std::fmod(yaw, 2 * M_PI);

        if (normalized_yaw < 0)
        {
            normalized_yaw +=
                2 * M_PI;
        }

        key.yaw =
            static_cast<int>(
                normalized_yaw /
                kYawResolution);

        key.yaw %= kYawBins;

        return key;
    }

    bool PoseSampler::NeedSaveFrame(
        const Eigen::Vector3d &position,
        const Eigen::Quaterniond &q)
    {
        double yaw =
            QuaternionToYaw(q);

        PoseKey key =
            ComputePoseKey(
                position,
                yaw);

        // 第一帧
        if (!has_last_pose_)
        {
            sampled_pose_keys_.insert(key);

            last_saved_position_ =
                position;

            last_saved_yaw_ =
                yaw;

            has_last_pose_ = true;

            return true;
        }

        // 距离变化
        double dist =
            (position -
             last_saved_position_)
                .norm();

        // yaw变化
        double yaw_diff =
            std::fabs(
                yaw -
                last_saved_yaw_);

        if (yaw_diff > M_PI)
        {
            yaw_diff =
                2 * M_PI -
                yaw_diff;
        }

        // 防抖动
        if (dist < min_save_distance_ &&
            yaw_diff < min_save_yaw_)
        {
            return false;
        }

        // 已采样
        auto it =
            sampled_pose_keys_.find(key);

        if (it !=
            sampled_pose_keys_.end())
        {
            return false;
        }

        // 新区域
        sampled_pose_keys_.insert(key);

        last_saved_position_ =
            position;

        last_saved_yaw_ =
            yaw;

        return true;
    }

} // namespace visual_localization
