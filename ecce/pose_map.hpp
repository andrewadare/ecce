#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <string>
#include <unordered_map>

using PoseMap =
    std::unordered_map<std::string, std::pair<gtsam::Symbol, gtsam::Pose3>>;

// Add a named, labeled camera pose to a PoseMap
void addCamera(const std::string& name, const gtsam::Pose3& pose,
               PoseMap& poses);

// Add a named, labeled fiducial tag pose to a PoseMap
void addTag(const std::string& name, const gtsam::Pose3& pose, PoseMap& poses);

// Count the number of external camera views on the "left" or "right" side
int countViews(const PoseMap& poses, const std::string side);
