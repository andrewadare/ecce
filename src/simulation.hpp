#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <iostream>
#include <sstream>
#include <unordered_map>
#include <vector>

using std::cout, std::endl;
using PoseMap =
    std::unordered_map<std::string, std::pair<gtsam::Symbol, gtsam::Pose3>>;
using ProjectionFactor =
    gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>;

// Returns the 3D corner points of a square fiducial tag (such as an AprilTag or
// Aruco marker) with a known pose. The tag's local coordinate frame is taken to
// be at the center of the tag, with x right, y up, and z to the viewer.
// Following OpenCV's convention, the 3D corner points are listed in TL, TR, BR,
// BL order.
std::vector<gtsam::Point3> tagCorners(const gtsam::Pose3& pose,
                                      const double edgeLength);

// Add a named, labeled camera pose to a PoseMap
void addCamera(const std::string& name, const std::array<double, 3>& ypr,
               const gtsam::Point3& position, PoseMap& poses);

// Add a named, labeled fiducial tag pose to a PoseMap
void addTag(const std::string& name, const std::array<double, 3>& ypr,
            const gtsam::Point3& position, PoseMap& poses);

// Generate ground-truth camera poses for the calibration setup.
// Camera coordinates are in meters and radians. XYZ = (fwd, left, up)
// in the vehicle frame wrt the origin at the center of the rear axle on the
// ground.
PoseMap simulateCameraPoses();

// Generate ground-truth fiducial tag poses for calibration setup
PoseMap simulateTagPoses();

// Transform a 3D point from world coordinates to the optical frame centered at
// the camera's position.
// World system: xyz = forward, left, up (active coords)
// Camera system: xyz = right, down, forward (passive coords)
gtsam::Point3 worldToCamera(const gtsam::Point3& point,
                            const gtsam::Pose3 cameraPose);

// Generate point measurements and add to factor graph
void addMeasurements(const PoseMap& cameraPoses, const PoseMap& tagPoses,
                     gtsam::NonlinearFactorGraph& graph);
