#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <vector>

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
