#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <ecce/geometry.hpp>
#include <vector>

// Performs perspective n-point pose estimation using correspondence
// between worldPoints and imagePoints.
// This is primarily an interface to cv::solvePnP for gtsam types.
// It assumes no lens distortion, and that imagePoints is provided
// in pixel coordinates.
gtsam::Pose3 pnp(const std::vector<gtsam::Point3>& worldPoints,
                 const std::vector<gtsam::Point2>& imagePoints,
                 const gtsam::Cal3_S2& intrinsics);
