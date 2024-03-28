#include <ecce/estimation.hpp>
#include <ecce/simulation.hpp>

using ProjectionFactor =
    gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>;

TagCollection simulateTags() {
  // NOTE: I made up these coords in a fwd-left-up system, then later decided to
  // use right-down-forward. I called changeBasis instead of retyping
  // everything.

  // Tag edge length in meters
  const double tagSize = 0.5;
  TagCollection tags(tagSize);                          // returned container
  std::unordered_map<std::string, gtsam::Pose3> poses;  // used locally

  // Vehicle parameters in meters
  const double wheelbase = 3.0;
  const double track = 2.0;
  const double wheel_radius = 0.5;

  // Tag orientations on left and right sides of the vehicle
  const auto Rright = gtsam::Rot3::Ypr(M_PI / 2, M_PI, 0);
  const auto Rleft = gtsam::Rot3::Ypr(-M_PI / 2, M_PI, 0);

  // Orientation of tags in front of vehicle
  const auto Rfront = gtsam::Rot3::Ypr(0, M_PI, 0);

  // Wheel-mounted tags: right front/rear and left front/rear
  poses["right_front_wheel"] =
      gtsam::Pose3(Rright, {wheelbase, -track / 2, wheel_radius});
  poses["right_rear_wheel"] =
      gtsam::Pose3(Rright, {0.0, -track / 2, wheel_radius});
  poses["left_front_wheel"] =
      gtsam::Pose3(Rleft, {wheelbase, track / 2, wheel_radius});
  poses["left_rear_wheel"] =
      gtsam::Pose3(Rleft, {0.0, track / 2, wheel_radius});

  // Front tags face back to vehicle with left/right offsets from centerline
  poses["left_front"] = gtsam::Pose3(Rfront, {5.0, 1.5, 1.0});
  poses["right_front"] = gtsam::Pose3(Rfront, {5.0, -1.5, 1.0});

  for (const auto& [name, pose] : poses) {
    const auto rdfPose =
        changeBasis(pose, EgoFrame::FWD_LEFT, EgoFrame::RIGHT_DOWN);
    tags.add(name, rdfPose);
  }

  return tags;
}

CameraCollection simulateCameras(const TagCollection& tags) {
  // All pose coords are in a right-down-forward system

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

  // Onboard camera
  // The goal of extrinsic calibration is to recover this pose!
  const auto pose = Camera::LookatPose({0, -2, 2}, {0, -1, 5}, up);
  cameras.add("onboard_camera", pose);

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

gtsam::Point2 projectTagCenter(const gtsam::Pose3& tagPose,
                               const Camera& camera, const double& pixelError) {
  std::vector<gtsam::Point2> imagePoints;
  std::normal_distribution<double> gauss(0.0, pixelError);
  std::mt19937_64 rng;

  gtsam::Point2 uv = camera.project(tagPose.translation());
  if (pixelError > 0) {
    uv += gtsam::Point2(gauss(rng), gauss(rng));
  }
  return uv;
}

std::vector<gtsam::Point2> projectTagCorners(const gtsam::Pose3& tagPose,
                                             const Camera& camera,
                                             const double& tagSize,
                                             const double& pixelError) {
  std::vector<gtsam::Point2> imagePoints;
  std::normal_distribution<double> gauss(0.0, pixelError);
  std::mt19937_64 rng;

  for (const auto& point : tagCorners(tagPose, tagSize)) {
    gtsam::Point2 uv = camera.project(point);

    if (pixelError > 0) {
      uv += gtsam::Point2(gauss(rng), gauss(rng));
    }

    imagePoints.push_back(uv);
  }
  return imagePoints;
}

gtsam::Pose3 simulateEstimatedPose(const gtsam::Pose3& tagPose,
                                   const Camera& camera,
                                   const double& tagSize) {
  // Tag corner points in local tag frame
  const std::vector<gtsam::Point3> worldPoints = localTagCorners(tagSize);

  // "Measured" 2D corner points
  const auto imagePoints = projectTagCorners(tagPose, camera, tagSize);

  // Estimated pose of camera in tag frame
  return pnp(worldPoints, imagePoints, camera.calibration());
}

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

std::string joinNames(const std::string& tagName,
                      const std::string& cameraName) {
  std::stringstream ss;
  ss << tagName << "_to_" << cameraName;
  return ss.str();
}

std::pair<std::string, std::string> splitName(const std::string name) {
  std::pair<std::string, std::string> names;  // tag, camera
  const std::string delimiter = "_to_";

  size_t i = name.find(delimiter);
  assert(i != std::string::npos);

  names.first = name.substr(0, i);
  names.second = name.substr(i + 4, name.size());

  return names;
}

PointMap simulateMeasurements(const CameraCollection& cameras,
                              const TagCollection& tags,
                              gtsam::Cal3_S2::shared_ptr intrinsics,
                              const double& pixelError,
                              gtsam::NonlinearFactorGraph& graph) {
  // Stores tag corner points projected to image (name => points)
  PointMap pointMap;
  const double tagSize = tags.getTagSize();

  // Isotropic 2x2 image point (u,v) detection uncertainty covariance [px]
  auto uvNoise = gtsam::noiseModel::Isotropic::Sigma(2, pixelError);

  // Simulate onboard camera measurements of front tag points
  const auto [ocSymbol, ocPose] = cameras.at("onboard_camera");
  Camera camera(ocPose, *intrinsics);
  for (const std::string& side : {"left", "right"}) {
    const auto tagPose = tags.getPose(side, "front");
    gtsam::Point2 uv = projectTagCenter(tagPose, camera, pixelError);

    graph.emplace_shared<ProjectionFactor>(
        uv, uvNoise, ocSymbol, tags.getSymbol(side, "front"), intrinsics);

    const auto name =
        joinNames(tags.getName(side, "front"), cameras.getName("onboard"));
    pointMap[name] = projectTagCorners(tagPose, camera, tagSize, pixelError);

    // for (const auto& uv : pointMap[name]) {
    //   graph.emplace_shared<ProjectionFactor>(
    //       uv, uvNoise, ocSymbol, tags.getSymbol(side, "front"), intrinsics);
    // }
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
        gtsam::Point2 uv = projectTagCenter(tagPose, camera, pixelError);

        graph.emplace_shared<ProjectionFactor>(uv, uvNoise, cameraSymbol,
                                               tagSymbol, intrinsics);
        const auto name = joinNames(tags.getName(side, zone),
                                    cameras.getName("external", side, i));
        pointMap[name] =
            projectTagCorners(tagPose, camera, tagSize, pixelError);

        // for (const auto& uv : pointMap[name]) {
        //   graph.emplace_shared<ProjectionFactor>(uv, uvNoise, cameraSymbol,
        //                                          tagSymbol, intrinsics);
        // }
      }
    }
  }

  return pointMap;
}
