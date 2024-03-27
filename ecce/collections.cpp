#include <ecce/collections.hpp>
#include <sstream>

CameraCollection::CameraCollection() : poseMap_(PoseMap()) {}
TagCollection::TagCollection() : poseMap_(PoseMap()), tagSize_(1.0) {}
TagCollection::TagCollection(const double tagSize)
    : poseMap_(PoseMap()), tagSize_(tagSize) {}

void CameraCollection::add(const std::string& name, const gtsam::Pose3& pose) {
  // Identifier for factor graph symbols
  static unsigned int cameraId = 0;
  poseMap_[name] = {gtsam::Symbol('c', cameraId), pose};
  ++cameraId;
}

void TagCollection::add(const std::string& name, const gtsam::Pose3& pose) {
  // Identifier for factor graph symbols
  static unsigned int tagId = 0;
  poseMap_[name] = {gtsam::Symbol('t', tagId), pose};
  ++tagId;
}

std::string CameraCollection::getName(const std::string& type,
                                      const std::string& side,
                                      const int& index) {
  std::stringstream name;
  auto sep = (side == "") ? "" : "_";
  name << side << sep << type << "_camera";
  if (index >= 0) {
    name << "_" << index;
  }
  return name.str();
}

gtsam::Symbol CameraCollection::getSymbol(const std::string& type,
                                          const std::string& side,
                                          const int& index) {
  const std::string name = getName(type, side, index);
  return poseMap_.at(name).first;
}

gtsam::Pose3 CameraCollection::getPose(const std::string& type,
                                       const std::string& side,
                                       const int& index) {
  const std::string name = getName(type, side, index);
  return poseMap_.at(name).second;
}

int CameraCollection::countViews(const std::string& side) {
  int numViews = 0;
  for (const auto& [name, data] : poseMap_) {
    std::stringstream ss;
    ss << side << "_external_camera";
    if (name.find(ss.str()) != std::string::npos) {
      ++numViews;
    }
  }
  return numViews;
}

std::string TagCollection::getName(const std::string& side,
                                   const std::string& zone) {
  std::stringstream name;
  name << side << zone;
  return name.str();
}

gtsam::Symbol TagCollection::getSymbol(const std::string& side,
                                       const std::string& zone) {
  const std::string name = getName(side, zone);
  return poseMap_.at(name).first;
}

gtsam::Pose3 TagCollection::getPose(const std::string& side,
                                    const std::string& zone) {
  const std::string name = getName(side, zone);
  return poseMap_.at(name).second;
}
