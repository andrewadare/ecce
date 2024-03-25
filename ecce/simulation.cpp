#include <ecce/simulation.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

PoseMap simulateTagPoses() {
  PoseMap poses;

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
  addTag("left_front", fl, poses);
  addTag("left_front_wheel", lf, poses);
  addTag("left_rear_wheel", lr, poses);

  addTag("right_front", fr, poses);
  addTag("right_front_wheel", rf, poses);
  addTag("right_rear_wheel", rr, poses);

  return poses;
}

PoseMap lookAtTags(const PoseMap& tagPoses) {
  PoseMap poses;  // for cameras

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

  const auto [rfSymbol, rfPose] = tagPoses.at("right_front_wheel");
  const auto [lfSymbol, lfPose] = tagPoses.at("left_front_wheel");

  // Left external camera poses
  for (size_t i = 0; i < numViews; ++i) {
    std::stringstream ss;
    ss << "left_external_camera_" << i;
    const auto pose =
        Camera::LookatPose({-x[i], y[i], z[i]}, lfPose.translation(), up);
    addCamera(ss.str(), pose, poses);
  }

  // Right external camera poses
  for (size_t i = 0; i < numViews; ++i) {
    std::stringstream ss;
    ss << "right_external_camera_" << i;
    const auto pose =
        Camera::LookatPose({x[i], y[i], z[i]}, rfPose.translation(), up);
    addCamera(ss.str(), pose, poses);
  }

  // Onboard camera
  // The goal is to recover this pose!
  const auto pose = Camera::LookatPose({0, -1, 2}, {0, -1, 5}, up);
  addCamera("onboard_camera", pose, poses);

  return poses;
}

gtsam::Cal3_S2::shared_ptr simulateCamera() {
  const double image_width = 1280, image_height = 960;
  const double fov = 75.0 * M_PI / 180.;  // isotropic
  const double fx = 0.5 * image_width / std::tan(0.5 * fov);
  const double fy = 0.5 * image_height / std::tan(0.5 * fov);

  gtsam::Cal3_S2::shared_ptr intrinsics(
      new gtsam::Cal3_S2(fx, fy, 0., image_width / 2, image_height / 2));

  return intrinsics;
}

void draw(std::vector<std::vector<gtsam::Point2>> imgPoints,
          const std::string& name) {
  cv::Mat im = cv::Mat::zeros(960, 1280, CV_8UC3);

  for (const auto& tag : imgPoints) {
    // TL corner in white
    cv::circle(im, {int(tag[0][0]), int(tag[0][1])}, 6, {255, 255, 255}, -1);

    // Line btwn first two points to check orientation
    cv::line(im, {int(tag[0][0]), int(tag[0][1])},
             {int(tag[1][0]), int(tag[1][1])}, {255, 255, 255});

    for (const auto& p : tag) {
      cv::circle(im, {int(p[0]), int(p[1])}, 4, {0, 255, 0}, -1);
    }
  }

  cv::imwrite(cv::format("%s.png", name.c_str()), im);
}

void addMeasurements(const PoseMap& cameraPoses, const PoseMap& tagPoses,
                     gtsam::NonlinearFactorGraph& graph) {
  // Create a calibration model for projection
  gtsam::Cal3_S2::shared_ptr intrinsics = simulateCamera();

  // Isotropic 2x2 image point (u,v) detection uncertainty covariance [px]
  auto uvNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  // Here we loop through external camera views on each side of the vehicle
  // at each tag position in order to project the tag corners to image points.
  const std::string sides[] = {"left", "right"};
  const std::string tagZones[] = {"front", "front_wheel", "rear_wheel"};
  const int numViews = 5;

  // Simulate onboard camera measurements of front tag points
  const auto [ocSymbol, ocPose] = cameraPoses.at("onboard_camera");
  Camera onboardCamera(ocPose, *intrinsics);
  std::vector<std::vector<gtsam::Point2>> imgPoints;
  for (const auto& side : sides) {
    std::vector<gtsam::Point2> tagPoints;  // one tag
    std::stringstream tagName;
    tagName << side << "_front";
    const auto [tagSymbol, tagPose] = tagPoses.at(tagName.str());
    for (const auto& point : tagCorners(tagPose, 0.5)) {
      tagPoints.push_back(onboardCamera.project(point));
    }

    // Use projected tag center point as the observed landmark
    gtsam::Point2 uv = onboardCamera.project(tagPose.translation());
    tagPoints.push_back(uv);

    graph.emplace_shared<ProjectionFactor>(uv, uvNoise, ocSymbol, tagSymbol,
                                           intrinsics);
    imgPoints.push_back(tagPoints);
  }
  draw(imgPoints, "onboard_camera");

  // Simulate external camera measurements of all tag points on each side
  for (int i = 0; i < numViews; ++i) {
    for (const auto& side : sides) {
      std::stringstream cameraName;
      cameraName << side << "_external_camera_" << i;

      const auto [cameraSymbol, cameraPose] = cameraPoses.at(cameraName.str());

      gtsam::PinholeCamera<gtsam::Cal3_S2> camera(cameraPose, *intrinsics);

      std::vector<std::vector<gtsam::Point2>> imgPoints;
      for (const auto& tagZone : tagZones) {
        std::vector<gtsam::Point2> tagPoints;  // one tag
        std::stringstream tagName;
        tagName << side << "_" << tagZone;
        cout << tagName.str() << " -> " << cameraName.str() << endl;

        // TODO add noise
        const auto [tagSymbol, tagPose] = tagPoses.at(tagName.str());

        for (const auto& point : tagCorners(tagPose, 0.5)) {
          tagPoints.push_back(camera.project(point));
        }

        // Use projected tag center point as the observed landmark
        gtsam::Point2 uv = camera.project(tagPose.translation());
        tagPoints.push_back(uv);

        graph.emplace_shared<ProjectionFactor>(uv, uvNoise, cameraSymbol,
                                               tagSymbol, intrinsics);
        imgPoints.push_back(tagPoints);
      }

      draw(imgPoints, cameraName.str());
    }
  }
}
