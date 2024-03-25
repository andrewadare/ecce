#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <ecce/geometry.hpp>
#include <ecce/pose_map.hpp>
#include <iostream>
#include <sstream>
#include <vector>

using std::cout, std::endl;
using Camera = gtsam::PinholeCamera<gtsam::Cal3_S2>;
using ProjectionFactor =
    gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>;

// Generate ground-truth fiducial tag poses for calibration setup
PoseMap simulateTagPoses();

// Generate ground-truth camera poses by looking at tags
PoseMap lookAtTags(const PoseMap& tagPoses);

// Camera model: distortion-free standard pinhole with 5 intrinsic parameters
// (fx, fy, skew, principal point). For simplicity, take the onboard
// camera calibration model to be the same as the external cameras.
// Model focal parameters using straight-line rays:
//   tan(fov/2) = image_size / 2f.
gtsam::Cal3_S2::shared_ptr simulateCamera();

// Generate point measurements and add to factor graph
void addMeasurements(const PoseMap& cameraPoses, const PoseMap& tagPoses,
                     gtsam::NonlinearFactorGraph& graph);

// Draw projected points on image and save to <name>.png
void draw(std::vector<std::vector<gtsam::Point2>> imgPoints,
          const std::string& name);
