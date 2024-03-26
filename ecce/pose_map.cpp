#include <ecce/pose_map.hpp>

void addCamera(const std::string& name, const gtsam::Pose3& pose,
               PoseMap& poses) {
  // Identifier for factor graph symbols
  static unsigned int cameraId = 0;
  poses[name] = {gtsam::Symbol('c', cameraId), pose};
  ++cameraId;
}

void addTag(const std::string& name, const gtsam::Pose3& pose, PoseMap& poses) {
  static unsigned int tagId = 0;
  poses[name] = {gtsam::Symbol('t', tagId), pose};
  ++tagId;
}

int countViews(const PoseMap& poses, const std::string side) {
  int numViews = 0;
  for (const auto& [name, data] : poses) {
    std::stringstream ss;
    ss << side << "_external_camera";
    if (name.find(ss.str()) != std::string::npos) {
      ++numViews;
    }
  }
  return numViews;
}
