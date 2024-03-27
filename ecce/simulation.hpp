#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <ecce/collections.hpp>
#include <ecce/geometry.hpp>
#include <iostream>
#include <sstream>
#include <vector>

using std::cout, std::endl;

// Generate ground-truth fiducial tag poses for calibration setup
TagCollection simulateTags();

// Generate ground-truth camera poses by looking at tags
CameraCollection simulateCameras(const TagCollection& tags);

// Generate measurement factors and add to the graph
PointMap simulateMeasurements(const CameraCollection& cameras,
                              const TagCollection& tags,
                              gtsam::Cal3_S2::shared_ptr intrinsics,
                              const double& pixelError,
                              gtsam::NonlinearFactorGraph& graph);

// Camera model: distortion-free standard pinhole with 5 intrinsic parameters
// (fx, fy, skew, principal point). For simplicity, take the onboard
// camera calibration model to be the same as the external cameras.
// Model focal parameters using straight-line rays:
//   tan(fov/2) = image_size / 2f.
gtsam::Cal3_S2::shared_ptr simulateCamera();

// Find camera pose in tag frame using known tag geometry and simulated 2D point
// detections
gtsam::Pose3 simulateEstimatedPose(const gtsam::Pose3& tagPose,
                                   const Camera& camera, const double& tagSize);

// Perform perspective N-point estimation by simulating image point
// measurements. Returns the camera pose in the coordinate frame of
// the observed object.
// Definitions:
// pointsOnObject: 3D points in local coordinate frame of observed object,
// e.g. corner points on a fiducial tag with respect to its center.
// pointsInWorld:  corresponding 3D points in the "world" frame, i.e. the
// common parent frame of the observed object and the camera.
gtsam::Pose3 simulatePnP(const std::vector<gtsam::Point3> pointsOnObject,
                         const std::vector<gtsam::Point3> pointsInWorld,
                         const Camera& camera);

// Project corners of square fiducial tag to image points
std::vector<gtsam::Point2> projectTagCorners(const gtsam::Pose3& tagPose,
                                             const Camera& camera,
                                             const double& tagSize,
                                             const double& pixelError = 0);

gtsam::Point2 projectTagCenter(const gtsam::Pose3& tagPose,
                               const Camera& camera,
                               const double& pixelError = 0);

std::string joinNames(const std::string& tagName,
                      const std::string& cameraName);
std::pair<std::string, std::string> splitName(const std::string name);
