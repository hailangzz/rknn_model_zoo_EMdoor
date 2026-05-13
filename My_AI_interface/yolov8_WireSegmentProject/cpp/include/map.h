#ifndef POSE_SAMPLER_H_
#define POSE_SAMPLER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <unordered_set>

namespace visual_localization
{

  constexpr double kDistanceResolution = 0.5;

  // 30°
  constexpr double kYawResolution = 0.52;

  const int kYawBins =
      static_cast<int>(2 * M_PI / kYawResolution);

  struct PoseKey
  {

    int x = 0;

    int y = 0;

    int yaw = 0;

    bool operator==(const PoseKey &other) const
    {

      return x == other.x &&
             y == other.y &&
             yaw == other.yaw;
    }

    struct Hash
    {

      std::size_t operator()(
          const PoseKey &k) const
      {

        return std::hash<int>()(k.x) ^
               (std::hash<int>()(k.y) << 1) ^
               (std::hash<int>()(k.yaw) << 2);
      }
    };
  };

  class PoseSampler
  {

  public:
    PoseSampler();

    bool NeedSaveFrame(
        const Eigen::Vector3d &position,
        const Eigen::Quaterniond &q);

    bool IsPoseSaveImage = false; // 是否符合空间位置，保存图像条件

  private:
    PoseKey ComputePoseKey(
        const Eigen::Vector3d &position,
        double yaw);

    double QuaternionToYaw(
        const Eigen::Quaterniond &q);

  private:
    std::unordered_set<
        PoseKey,
        PoseKey::Hash>
        sampled_pose_keys_;

    Eigen::Vector3d last_saved_position_ =
        Eigen::Vector3d::Zero();

    double last_saved_yaw_ = 0.0;

    bool has_last_pose_ = false;

    double min_save_distance_ = 0.5;

    double min_save_yaw_ = 0.3;
  };

} // namespace visual_localization

#endif
