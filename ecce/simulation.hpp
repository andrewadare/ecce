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
#include <vector>

#include <ecce/geometry.hpp>
#include <ecce/pose_map.hpp>

using std::cout, std::endl;
using ProjectionFactor =
    gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>;

// Generate ground-truth camera poses for the calibration setup.
// Camera coordinates are in meters and radians. XYZ = (fwd, left, up)
// in the vehicle frame wrt the origin at the center of the rear axle on the
// ground.
PoseMap simulateCameraPoses();

// Generate ground-truth fiducial tag poses for calibration setup
PoseMap simulateTagPoses();

// Camera model: distortion-free standard pinhole with 5 intrinsic parameters
// (fx, fy, skew, principal point). For simplicity, take the onboard
// camera calibration model to be the same as the external cameras.
// Model focal parameters using straight-line rays:
//   tan(fov/2) = image_size / 2f.
gtsam::Cal3_S2::shared_ptr simulateCamera();

// Generate point measurements and add to factor graph
void addMeasurements(const PoseMap& cameraPoses, const PoseMap& tagPoses,
                     gtsam::NonlinearFactorGraph& graph);
