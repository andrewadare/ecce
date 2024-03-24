#include <ecce/geometry.hpp>

gtsam::Point3 worldToCamera(const gtsam::Point3& point,
                            const gtsam::Pose3 cameraPose) {
  // FLU -> RDF basis change: columns x, y, z -> z, -x, -y.
  gtsam::Matrix33 B;
  B << 0, -1, 0,  //
      0, 0, -1,   //
      1, 0, 0;

  const gtsam::Matrix33 R = B * cameraPose.rotation().matrix().transpose();
  const gtsam::Point3 t = -R * cameraPose.translation();

  return R * point + t;
}

std::vector<gtsam::Point3> tagCorners(const gtsam::Pose3& pose,
                                      const double edgeLength) {
  const double a = edgeLength / 2;

  return {pose.transformFrom(gtsam::Point3(-a, a, 0)),
          pose.transformFrom(gtsam::Point3(a, a, 0)),
          pose.transformFrom(gtsam::Point3(a, -a, 0)),
          pose.transformFrom(gtsam::Point3(-a, -a, 0))};
}
