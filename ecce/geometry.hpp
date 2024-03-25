#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <unordered_map>
#include <vector>

using Camera = gtsam::PinholeCamera<gtsam::Cal3_S2>;

// Enumerations for interpreting egocentric (x,y,z) coordinates.
// The third direction in each system is implied via the right hand rule.
enum class EgoFrame {
  FWD_LEFT,    // ROS REP 103 / ENU
  FWD_RIGHT,   // aeronautics / NED
  RIGHT_DOWN,  // camera optical
  RIGHT_UP     // OpenGL
};

// Returns a change-of-basis matrix in SO(3) from frame a to b
const gtsam::Matrix33 basisChangeMatrix(EgoFrame a, EgoFrame b);

// Change pose representation from frame a to frame b
gtsam::Pose3 changeBasis(const gtsam::Pose3& pose, EgoFrame a, EgoFrame b);

// Transform a 3D point from world coordinates to the optical frame centered at
// the camera's position.
// World system: xyz = forward, left, up (active coords)
// Camera system: xyz = right, down, forward (passive coords)
gtsam::Point3 worldToCamera(const gtsam::Point3& point,
                            const gtsam::Pose3 cameraPose);

// Returns the 3D corner points of a square fiducial tag (such as an AprilTag or
// Aruco marker) with a known pose. The tag's local coordinate frame is taken to
// be at the center of the tag, with x right, y up, and z to the viewer.
// Following OpenCV's convention, the 3D corner points are listed in TL, TR, BR,
// BL order.
std::vector<gtsam::Point3> tagCorners(const gtsam::Pose3& pose,
                                      const double edgeLength);

// Returns 3 endpoints of a standard basis as viewed from another
// frame. The axes are scaled using `length`.
std::vector<gtsam::Point3> axisPoints(const gtsam::Pose3& pose,
                                        const double length);
