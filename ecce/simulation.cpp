#include <ecce/simulation.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

PoseMap simulateCameraPoses() {
  PoseMap poses;

  // External camera coords in vehicle frame (xyz = fwd-left-up)
  constexpr size_t numViews = 3;
  const std::array<double, numViews> xs = {-1.0, 0.0, 1.0};
  const std::array<double, numViews> ys = {5.0, 6.0, 7.0};
  const std::array<double, numViews> zs = {1.5, 1.5, 1.5};
  const std::array<double, numViews> yaws = {0.9, 1.12, 1.3};

  // Left external camera poses
  for (size_t i = 0; i < numViews; ++i) {
    std::stringstream ss;
    ss << "left_external_camera_" << i;
    auto pose = gtsam::Pose3(gtsam::Rot3::Yaw(yaws[i]), {xs[i], ys[i], zs[i]});
    pose = changeBasis(pose, EgoFrame::FWD_LEFT, EgoFrame::RIGHT_DOWN);
    addCamera(ss.str(), pose, poses);
  }

  // Right external camera poses
  for (size_t i = 0; i < numViews; ++i) {
    std::stringstream ss;
    ss << "right_external_camera_" << i;
    auto pose =
        gtsam::Pose3(gtsam::Rot3::Yaw(-yaws[i]), {xs[i], -ys[i], zs[i]});
    pose = changeBasis(pose, EgoFrame::FWD_LEFT, EgoFrame::RIGHT_DOWN);
    addCamera(ss.str(), pose, poses);
  }

  // Onboard camera pose: the goal is to recover this!
  auto pose = gtsam::Pose3(gtsam::Rot3::Identity(), {2.5, 0, 2});
  pose = changeBasis(pose, EgoFrame::FWD_LEFT, EgoFrame::RIGHT_DOWN);
  addCamera("onboard_camera", pose, poses);

  return poses;
};

PoseMap simulateTagPoses() {
  PoseMap poses;

  // Made-up vehicle parameters in meters
  const double wheelbase = 3.0;
  const double track = 2.0;
  const double wheel_radius = 0.5;

  // Tag orientations on left and right sides of the vehicle
  const auto Rleft = gtsam::Rot3::Ypr(M_PI, 0, M_PI / 2);
  const auto Rright = gtsam::Rot3::Ypr(0, 0, M_PI / 2);

  // Orientation of tags in front of vehicle
  const auto Rfront = gtsam::Rot3::Ypr(-M_PI / 2, 0, M_PI / 2);

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

  addTag("right_front_wheel", rf, poses);
  addTag("right_rear_wheel", rr, poses);
  addTag("left_front_wheel", lf, poses);
  addTag("left_rear_wheel", lr, poses);
  addTag("front_left", fl, poses);
  addTag("front_right", fr, poses);

  return poses;
}

PoseMap lookAtTags(const PoseMap& tagPoses) {
  PoseMap poses;  // for cameras

  // Camera positions
  constexpr size_t numViews = 3;
  const gtsam::Point3 up(0, 0, 1);

  // External camera positions in for 3 views
  // x: distance right of centerline
  // y: distance below ground level
  // z: distance fwd of rear axle
  const std::array<double, numViews> x = {5.0, 6.0, 7.0};
  const std::array<double, numViews> y = {-1.5, -1.5, -1.5};
  const std::array<double, numViews> z = {-1.0, 0.0, 1.0};

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

  // TODO onboard camera

  return poses;
}

gtsam::Cal3_S2::shared_ptr simulateCamera() {
  const double image_width = 1280, image_height = 960;
  const double fov = 90.0 * M_PI / 180.;  // isotropic
  const double fx = 0.5 * image_width / std::tan(0.5 * fov);
  const double fy = 0.5 * image_height / std::tan(0.5 * fov);

  gtsam::Cal3_S2::shared_ptr intrinsics(
      new gtsam::Cal3_S2(fx, fy, 0., image_width / 2, image_height / 2));

  return intrinsics;
}

void draw(std::vector<gtsam::Point2> pts, const std::string& name) {
  cv::Mat im(960, 1280, CV_8UC3);

  // TL corner in white
  cv::circle(im, {int(pts[0][0]), int(pts[0][1])}, 6, {255, 255, 255}, -1);

  for (const auto& p : pts) {
    cv::circle(im, {int(p[0]), int(p[1])}, 4, {0, 255, 0}, -1);
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
  const std::string ends[] = {"front", "rear"};

  for (int i = 0; i < 3; ++i) {  // TODO hardcoded to 3 views per side
    for (const auto& side : sides) {
      std::stringstream cameraName;
      cameraName << side << "_external_camera_" << i;

      const auto [cameraSymbol, cameraPose] = cameraPoses.at(cameraName.str());
      // cout << cameraName.str() << endl;
      // cout << cameraPose.translation() << endl;

      gtsam::PinholeCamera<gtsam::Cal3_S2> camera(cameraPose, *intrinsics);

      std::vector<gtsam::Point2> tagPoints;
      for (const auto& end : ends) {
        std::stringstream tagName;
        tagName << side << "_" << end << "_wheel";
        cout << tagName.str() << " -> " << cameraName.str() << endl;

        // TODO add noise
        const auto [tagSymbol, tagPose] = tagPoses.at(tagName.str());

        cout << "tag: " << tagPose.translation() << endl;
        cout << "cam: " << cameraPose.translation() << endl;

        // Use projected tag center point as the observed landmark
        gtsam::Point2 uv = camera.project(tagPose.translation());

        tagPoints.push_back(uv);

        graph.emplace_shared<ProjectionFactor>(uv, uvNoise, cameraSymbol,
                                               tagSymbol, intrinsics);

        // for (const auto& point : tagCorners(tagPose, 0.5)) {
        //   tagPoints.push_back(camera.project(point));
        // }
      }

      draw(tagPoints, cameraName.str());
    }
  }
}
