#include <ecce/pose_map.hpp>

void addCamera(const std::string& name, const std::array<double, 3>& ypr,
               const gtsam::Point3& position, PoseMap& poses) {
  // Identifier for factor graph symbols
  static unsigned int cameraId = 0;
  const auto rot = gtsam::Rot3::Ypr(ypr[0], ypr[1], ypr[2]);
  poses[name] = {gtsam::Symbol('c', cameraId), gtsam::Pose3(rot, position)};
  ++cameraId;
}

void addTag(const std::string& name, const std::array<double, 3>& ypr,
            const gtsam::Point3& position, PoseMap& poses) {
  static unsigned int tagId = 0;
  const auto rot = gtsam::Rot3::Ypr(ypr[0], ypr[1], ypr[2]);
  poses[name] = {gtsam::Symbol('t', tagId), gtsam::Pose3(rot, position)};
  ++tagId;
}
