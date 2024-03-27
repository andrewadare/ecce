#include <ecce/estimation.hpp>
#include <ecce/simulation.hpp>

using ProjectionFactor =
    gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>;

TagCollection simulateTags() {
  // Tag edge length in meters
  const double tagSize = 0.5;
  TagCollection tags(tagSize);

  // Made-up vehicle parameters in meters
  const double wheelbase = 3.0;
  const double track = 2.0;
  const double wheel_radius = 0.5;

  // Tag orientations on left and right sides of the vehicle
  const auto Rright = gtsam::Rot3::Ypr(M_PI / 2, M_PI, 0);
  const auto Rleft = gtsam::Rot3::Ypr(-M_PI / 2, M_PI, 0);

  // Orientation of tags in front of vehicle
  const auto Rfront = gtsam::Rot3::Ypr(0, M_PI, 0);

  // Wheel-mounted tags: right front/rear and left front/rear
  auto rf = gtsam::Pose3(Rright, {wheelbase, -track / 2, wheel_radius});
  auto rr = gtsam::Pose3(Rright, {0.0, -track / 2, wheel_radius});
  auto lf = gtsam::Pose3(Rleft, {wheelbase, track / 2, wheel_radius});
  auto lr = gtsam::Pose3(Rleft, {0.0, track / 2, wheel_radius});

  // Front tags face back to vehicle with left/right offsets from centerline
  auto fl = gtsam::Pose3(Rfront, {5.0, 1.5, 1.0});
  auto fr = gtsam::Pose3(Rfront, {5.0, -1.5, 1.0});

  // FLU -> RDF
  rf = changeBasis(rf, EgoFrame::FWD_LEFT, EgoFrame::RIGHT_DOWN);
  rr = changeBasis(rr, EgoFrame::FWD_LEFT, EgoFrame::RIGHT_DOWN);
  lf = changeBasis(lf, EgoFrame::FWD_LEFT, EgoFrame::RIGHT_DOWN);
  lr = changeBasis(lr, EgoFrame::FWD_LEFT, EgoFrame::RIGHT_DOWN);
  fl = changeBasis(fl, EgoFrame::FWD_LEFT, EgoFrame::RIGHT_DOWN);
  fr = changeBasis(fr, EgoFrame::FWD_LEFT, EgoFrame::RIGHT_DOWN);

  // 2 sides x 3 longitudinal zones
  tags.add("left_front", fl);
  tags.add("left_front_wheel", lf);
  tags.add("left_rear_wheel", lr);
  tags.add("right_front", fr);
  tags.add("right_front_wheel", rf);
  tags.add("right_rear_wheel", rr);

  return tags;
}

CameraCollection simulateCameras(const TagCollection& tags) {
  CameraCollection cameras;

  // Camera positions
  constexpr size_t numViews = 5;
  const gtsam::Point3 up(0, -1, 0);  // -y in camera frame

  // External camera positions in for 3 views
  // x: distance right of centerline
  // y: distance below ground level
  // z: distance fwd of rear axle
  const std::array<double, numViews> x = {4, 5, 6, 7, 8};
  const std::array<double, numViews> y = {-1.5, -1.5, -1.5, -1.5, -1.5};
  const std::array<double, numViews> z = {-2, -1, 0, 1, 2};

  // Use front wheel tags as the center of attention for Camera::lookAt
  const gtsam::Pose3 rfPose = tags.at("right_front_wheel").second;
  const gtsam::Pose3 lfPose = tags.at("left_front_wheel").second;

  // Left external camera poses
  for (size_t i = 0; i < numViews; ++i) {
    std::stringstream ss;
    ss << "left_external_camera_" << i;
    const auto pose =
        Camera::LookatPose({-x[i], y[i], z[i]}, lfPose.translation(), up);
    cameras.add(ss.str(), pose);
  }

  // Right external camera poses
  for (size_t i = 0; i < numViews; ++i) {
    std::stringstream ss;
    ss << "right_external_camera_" << i;
    const auto pose =
        Camera::LookatPose({x[i], y[i], z[i]}, rfPose.translation(), up);
    cameras.add(ss.str(), pose);
  }

  // Onboard camera
  // The goal is to recover this pose!
  const auto pose = Camera::LookatPose({0, -1, 2}, {0, -1, 5}, up);
  cameras.add("onboard_camera", pose);

  return cameras;
}

gtsam::Cal3_S2::shared_ptr simulateCamera() {
  const double image_width = 1280, image_height = 960;
  const double fov = 75.0 * M_PI / 180.;
  const double fx = 0.5 * image_width / std::tan(0.5 * fov);
  const double fy = 0.5 * image_height / std::tan(0.5 * fov);

  gtsam::Cal3_S2::shared_ptr intrinsics(
      new gtsam::Cal3_S2(fx, fy, 0., image_width / 2, image_height / 2));

  return intrinsics;
}

gtsam::Pose3 simulateEstimatedPose(const gtsam::Pose3& tagPose,
                                   const Camera& camera,
                                   const double& tagSize) {
  // Tag corner points in local tag frame
  std::vector<gtsam::Point3> worldPoints = localTagCorners(tagSize);

  // "Measured" 2D corner points
  std::vector<gtsam::Point2> imagePoints;
  for (const auto& point : tagCorners(tagPose, tagSize)) {
    imagePoints.push_back(camera.project(point));
  }

  // Estimated pose of camera in tag frame
  return pnp(worldPoints, imagePoints, camera.calibration());
}

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
                         const Camera& camera) {
  // Project pointsInWorld to image
  std::vector<gtsam::Point2> imagePoints;
  for (const auto& point : pointsInWorld) {
    imagePoints.push_back(camera.project(point));
  }

  // Estimated pose of onboard camera in front tag frame
  return pnp(pointsOnObject, imagePoints, camera.calibration());
}

void simulateMeasurements(const CameraCollection& cameras,
                          const TagCollection& tags,
                          gtsam::Cal3_S2::shared_ptr intrinsics,
                          gtsam::NonlinearFactorGraph& graph) {
  // Isotropic 2x2 image point (u,v) detection uncertainty covariance [px]
  auto uvNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  // Simulate onboard camera measurements of front tag points
  const auto [ocSymbol, ocPose] = cameras.at("onboard_camera");
  Camera onboardCamera(ocPose, *intrinsics);
  for (const std::string& side : {"left", "right"}) {
    gtsam::Point2 uv =
        onboardCamera.project(tags.getPose(side, "front").translation());
    graph.emplace_shared<ProjectionFactor>(
        uv, uvNoise, ocSymbol, tags.getSymbol(side, "front"), intrinsics);
  }

  // Simulate external camera measurements of tag points on each side
  for (const std::string& side : {"left", "right"}) {
    for (int i = 0; i < cameras.countViews(side); ++i) {
      const auto cameraPose = cameras.getPose("external", side, i);
      const auto cameraSymbol = cameras.getSymbol("external", side, i);
      Camera camera(cameraPose, *intrinsics);

      // Use projected tag center point as the observed landmark
      for (const std::string& zone : {"front", "front_wheel", "rear_wheel"}) {
        const auto tagPose = tags.getPose(side, zone);
        const auto tagSymbol = tags.getSymbol(side, zone);
        gtsam::Point2 uv = camera.project(tagPose.translation());
        graph.emplace_shared<ProjectionFactor>(uv, uvNoise, cameraSymbol,
                                               tagSymbol, intrinsics);
      }
    }
  }
}
