#include <ecce/collections.hpp>
#include <iostream>
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
                                      const int& index) const {
  std::stringstream ss;
  auto sep = (side == "") ? "" : "_";
  ss << side << sep << type << "_camera";
  if (index >= 0) {
    ss << "_" << index;
  }

  if (poseMap_.count(ss.str()) == 0) {
    std::cout << "Warning: no camera named " << ss.str() << std::endl;
  }

  return ss.str();
}

gtsam::Symbol CameraCollection::getSymbol(const std::string& type,
                                          const std::string& side,
                                          const int& index) const {
  const std::string name = getName(type, side, index);
  return poseMap_.at(name).first;
}

gtsam::Pose3 CameraCollection::getPose(const std::string& type,
                                       const std::string& side,
                                       const int& index) const {
  const std::string name = getName(type, side, index);
  return poseMap_.at(name).second;
}

int CameraCollection::countViews(const std::string& side) const {
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
                                   const std::string& zone) const {
  std::stringstream ss;
  ss << side << "_" << zone;

  if (poseMap_.count(ss.str()) == 0) {
    std::cout << "Warning: no tag named " << ss.str() << std::endl;
  }

  return ss.str();
}

gtsam::Symbol TagCollection::getSymbol(const std::string& side,
                                       const std::string& zone) const {
  const std::string name = getName(side, zone);
  return poseMap_.at(name).first;
}

gtsam::Pose3 TagCollection::getPose(const std::string& side,
                                    const std::string& zone) const {
  const std::string name = getName(side, zone);
  return poseMap_.at(name).second;
}
