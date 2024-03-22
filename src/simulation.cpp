#include "simulation.hpp"

std::vector<gtsam::Point3> tagCorners(const gtsam::Pose3& pose,
                                      const double edgeLength) {
  const double a = edgeLength / 2;

  return {pose.transformFrom(gtsam::Point3(-a, a, 0)),
          pose.transformFrom(gtsam::Point3(a, a, 0)),
          pose.transformFrom(gtsam::Point3(a, -a, 0)),
          pose.transformFrom(gtsam::Point3(-a, -a, 0))};
}

void addCamera(const std::string& name, const std::array<double, 3>& ypr,
               const gtsam::Point3& position, PoseMap& poses) {
  // Identifier for factor graph symbols
  static unsigned int cameraId = 0;
  const auto rot = gtsam::Rot3::Ypr(ypr[0], ypr[1], ypr[2]);
  poses[name] = {gtsam::Symbol('c', cameraId), gtsam::Pose3(rot, position)};
  ++cameraId;
}

void addTag(const std::string& name, const std::array<double, 3>& ypr,
            const gtsam::Point3& position, PoseMap& poses) {
  static unsigned int tagId = 0;
  const auto rot = gtsam::Rot3::Ypr(ypr[0], ypr[1], ypr[2]);
  poses[name] = {gtsam::Symbol('t', tagId), gtsam::Pose3(rot, position)};
  ++tagId;
}

PoseMap simulateCameraPoses() {
  PoseMap poses;

  // Onboard camera
  addCamera("onboard_camera", {0, 0, 0}, {2.5, 0, 2}, poses);

  // External cameras
  constexpr size_t numViews = 3;
  const std::array<double, numViews> xs = {-1.0, 0.0, 1.0};
  const std::array<double, numViews> ys = {5.0, 6.0, 7.0};
  const std::array<double, numViews> zs = {1.5, 1.5, 1.5};
  const std::array<double, numViews> yaws = {0.9, 1.12, 1.3};

  std::stringstream ss;
  for (size_t i = 0; i < numViews; ++i) {
    ss.str("");
    ss.clear();
    ss << "left_external_camera_" << i;
    addCamera(ss.str(), {-yaws[i], 0., 0.}, {xs[i], ys[i], zs[i]}, poses);

    ss.str("");
    ss.clear();
    ss << "right_external_camera_" << i;
    addCamera(ss.str(), {yaws[i], 0., 0.}, {xs[i], -ys[i], zs[i]}, poses);
  }

  return poses;
};

PoseMap simulateTagPoses() {
  PoseMap poses;

  // Made-up vehicle parameters in meters
  const double wheelbase = 3.0;
  const double track = 2.0;
  const double wheel_radius = 0.5;

  // Create fiducial tags centered on the wheel hubs.
  // A tag with pose I(4) would lay on the ground with the top edge toward the
  // front of the vehicle.
  // To orient the tags vertically, rotate about x (i.e. roll) 90 degrees.
  // The right tags naturally face outward as desired, but the left tags
  // must rotate 180 degrees about z.
  addTag("right_front_wheel", {0, 0, M_PI / 2},
         {wheelbase, -track / 2, wheel_radius}, poses);
  addTag("right_rear_wheel", {0, 0, M_PI / 2}, {0.0, -track / 2, wheel_radius},
         poses);
  addTag("left_front_wheel", {M_PI, 0, M_PI / 2},
         {wheelbase, track / 2, wheel_radius}, poses);
  addTag("left_rear_wheel", {M_PI, 0, M_PI / 2}, {0.0, track / 2, wheel_radius},
         poses);

  // Tag poses in front of the vehicle are visible to both the onboard camera
  // and the external cameras on the respective side.
  addTag("left_target", {-M_PI / 2, 0, M_PI / 2}, {5.0, 1.5, 1.0}, poses);
  addTag("right_target", {-M_PI / 2, 0, M_PI / 2}, {5.0, -1.5, 1.0}, poses);

  return poses;
}

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

void addMeasurements(const PoseMap& cameraPoses, const PoseMap& tagPoses,
                     gtsam::NonlinearFactorGraph& graph) {
  graph.print();

  // const double tagSize = 0.5;  // Edge length of fiducial markers [m]

  // Camera model: distortion-free standard pinhole with 5 intrinsic parameters
  // (fx, fy, skew, principal point). For simplicity, take the onboard
  // camera calibration model to be the same as the external cameras.
  // Model focal parameters using straight-line rays:
  //   tan(fov/2) = image_size / 2f.
  //
  const double image_width = 1280, image_height = 960;
  const double fov = 90.0 * M_PI / 180.;  // isotropic
  const double fx = 0.5 * image_width / std::tan(0.5 * fov);
  const double fy = 0.5 * image_height / std::tan(0.5 * fov);

  gtsam::Cal3_S2::shared_ptr intrinsics(
      new gtsam::Cal3_S2(fx, fy, 0., image_width / 2, image_height / 2));

  // Isotropic 2x2 image point (u,v) detection uncertainty covariance [px]
  auto uvNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  const std::string sides[] = {"left", "right"};
  const std::string ends[] = {"front", "rear"};

  // Here we loop through external camera views on each side of the vehicle
  // at each tag position in order to project the tag corners to image points.

  for (int i = 0; i < 3; ++i) {  // TODO hardcoded to 3 views per side
    for (const auto& side : sides) {
      std::stringstream cameraName;
      cameraName << side << "_external_camera_" << i;

      const auto [cameraSymbol, cameraPose] = cameraPoses.at(cameraName.str());

      gtsam::PinholeCamera<gtsam::Cal3_S2> camera(cameraPose, *intrinsics);

      for (const auto& end : ends) {
        std::stringstream tagName;
        tagName << side << "_" << end << "_wheel";
        cout << tagName.str() << " -> " << cameraName.str() << endl;

        // TODO add noise
        const auto [tagSymbol, tagPose] = tagPoses.at(tagName.str());

        // Use projected tag center point as the observed landmark
        gtsam::Point2 uv =
            camera.project(worldToCamera(tagPose.translation(), cameraPose));
        cout << uv << endl;

        graph.emplace_shared<ProjectionFactor>(uv, uvNoise, cameraSymbol,
                                               tagSymbol, intrinsics);
      }
    }
  }
}
