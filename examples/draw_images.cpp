
#include <ecce/estimation.hpp>
#include <ecce/simulation.hpp>
#include <ecce/visualization.hpp>

void drawImages(const PoseMap& cameraPoses, const PoseMap& tagPoses,
                gtsam::Cal3_S2::shared_ptr intrinsics) {
  // Here we loop through external camera views on each side of the vehicle
  // at each tag position in order to project the tag points to image points.
  const std::string sides[] = {"left", "right"};
  const std::string tagZones[] = {"front", "front_wheel", "rear_wheel"};

  // Simulate onboard camera measurements of front tag points
  const auto [ocSymbol, ocPose] = cameraPoses.at("onboard_camera");
  Camera onboardCamera(ocPose, *intrinsics);

  std::vector<gtsam::Pose3> tagsToDraw;

  // Tag corner points in local tag frame (for PnP estimation)
  auto worldPoints = localTagCorners(0.5);

  // Onboard camera image
  for (const auto& side : sides) {
    std::stringstream tagName;
    tagName << side << "_front";
    const auto [tagSymbol, tagPose] = tagPoses.at(tagName.str());
    tagsToDraw.push_back(tagPose);

    if (true) {  // check PnP
      std::vector<gtsam::Point2> imagePoints;
      for (const auto& point : tagCorners(tagPose, 0.5)) {
        imagePoints.push_back(onboardCamera.project(point));
      }

      // Estimated pose of onboard camera in front tag frame
      auto camToTag = pnp(worldPoints, imagePoints, *intrinsics);
      auto estCameraPose = tagPose * camToTag.inverse();  // in vehicle frame
      assert(estCameraPose.equals(ocPose, 1e-6));
    }
  }
  draw(tagsToDraw, onboardCamera, "onboard_camera");
  tagsToDraw.clear();

  // External camera images
  for (const auto& side : sides) {
    for (int i = 0; i < countViews(cameraPoses, side); ++i) {
      std::stringstream ss;
      ss << side << "_external_camera_" << i;
      // cout << ss.str() << " " << cameraName("external", side, i) << endl;

      const auto [cameraSymbol, cameraPose] = cameraPoses.at(ss.str());
      Camera camera(cameraPose, *intrinsics);

      for (const auto& tagZone : tagZones) {
        std::stringstream tagName;
        tagName << side << "_" << tagZone;
        const auto [tagSymbol, tagPose] = tagPoses.at(tagName.str());
        tagsToDraw.push_back(tagPose);

        // Should see GT tag pose * camera_to_tag == GT camera pose
        if (true) {  // check PnP
          std::vector<gtsam::Point2> imagePoints;
          for (const auto& point : tagCorners(tagPose, 0.5)) {
            imagePoints.push_back(camera.project(point));
          }

          // Estimated pose of external camera in wheel tag frame
          auto camToTag = pnp(worldPoints, imagePoints, *intrinsics);
          auto estCameraPose =
              tagPose * camToTag.inverse();  // in vehicle frame
          assert(estCameraPose.equals(cameraPose, 1e-6));
        }
      }
      draw(tagsToDraw, camera, ss.str());
      tagsToDraw.clear();
    }
  }
}

int main(int argc, char* argv[]) {
  if (argc != 1) {
    cout << "Usage: " << argv[0] << " (no args)" << endl;
    return 1;
  }

  TagCollection tags = simulateTags();
  CameraCollection cameras = simulateCameras(tags);

  // // Calibration model for projection
  gtsam::Cal3_S2::shared_ptr intrinsics = simulateCamera();

  // Draw projected tags to image files
  // TODO refactor to use *Collection
  drawImages(cameras.all(), tags.all(), intrinsics);

  return 0;
}
