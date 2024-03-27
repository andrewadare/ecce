#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <string>
#include <unordered_map>

using PoseMap =
    std::unordered_map<std::string, std::pair<gtsam::Symbol, gtsam::Pose3>>;

class CameraCollection {
 private:
  // Maps name of camera to {symbol, pose}
  PoseMap poseMap_;

 public:
  // Default constructor
  CameraCollection();

  // Virtual destructor
  virtual ~CameraCollection() {}

  // Add a named, labeled camera pose to the collection
  void add(const std::string& name, const gtsam::Pose3& pose);

  const PoseMap all() const { return poseMap_; }

  // Returns factor graph symbol and camera pose by name
  std::pair<gtsam::Symbol, gtsam::Pose3> at(const std::string& name) const {
    return poseMap_.at(name);
  }

  // Returns name of camera with the given parameters
  std::string getName(const std::string& type, const std::string& side = "",
                      const int& index = -1) const;

  // Returns unique factor graph symbol
  gtsam::Symbol getSymbol(const std::string& type, const std::string& side = "",
                          const int& index = -1) const;

  // Returns camera pose
  gtsam::Pose3 getPose(const std::string& type, const std::string& side = "",
                       const int& index = -1) const;

  // Returns the number of external camera views on the "left" or "right" side
  int countViews(const std::string& side) const;
};

class TagCollection {
 private:
  // Maps name of tag to {symbol, pose}
  PoseMap poseMap_;

  // edge length (all tags same size)
  double tagSize_;

 public:
  // Default constructor
  TagCollection();

  // Provide edge length of tag
  TagCollection(const double tagSize);

  // Virtual destructor
  virtual ~TagCollection() {}

  // Add a named, labeled tag pose to the collection
  void add(const std::string& name, const gtsam::Pose3& pose);

  const PoseMap all() const { return poseMap_; }

  // Returns factor graph symbol and camera pose by name
  std::pair<gtsam::Symbol, gtsam::Pose3> at(const std::string& name) const {
    return poseMap_.at(name);
  }

  // Returns name of tag with the given parameters
  std::string getName(const std::string& side, const std::string& zone) const;

  // Returns unique factor graph symbol
  gtsam::Symbol getSymbol(const std::string& side,
                          const std::string& zone) const;

  // Returns camera pose
  gtsam::Pose3 getPose(const std::string& side, const std::string& zone) const;
};
