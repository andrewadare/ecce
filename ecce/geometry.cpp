#include <ecce/geometry.hpp>

namespace basis {

// Standard basis vectors
const gtsam::Point3 e1(1, 0, 0), e2(0, 1, 0), e3(0, 0, 1);

// SO(3) rotations constructed from basis column vectors.
// FWD_LEFT is chosen as the diagonal basis.
const std::unordered_map<EgoFrame, gtsam::Rot3> rotMap = {
    {EgoFrame::FWD_LEFT, gtsam::Rot3(e1, e2, e3)},
    {EgoFrame::FWD_RIGHT, gtsam::Rot3(e1, -e2, -e3)},
    {EgoFrame::RIGHT_DOWN, gtsam::Rot3(e3, -e1, -e2)},
    {EgoFrame::RIGHT_UP, gtsam::Rot3(-e3, -e1, e2)}};

}  // namespace basis

const gtsam::Matrix33 basisChangeMatrix(EgoFrame a, EgoFrame b) {
  return basis::rotMap.at(a).matrix().transpose() *
         basis::rotMap.at(b).matrix();
}

gtsam::Pose3 changeBasis(const gtsam::Pose3& pose, EgoFrame a, EgoFrame b) {
  const gtsam::Matrix33 bRa = basisChangeMatrix(a, b);
  const gtsam::Rot3 R(bRa * pose.rotation().matrix() * bRa.transpose());
  const gtsam::Point3 t(bRa * pose.translation());

  return gtsam::Pose3(R, t);
}

// TODO remove
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
