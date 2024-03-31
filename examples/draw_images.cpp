
#include <ecce/estimation.hpp>
#include <ecce/simulation.hpp>
#include <ecce/visualization.hpp>

void drawImages(const CameraCollection& cameras, const TagCollection& tags,
                gtsam::Cal3_S2::shared_ptr intrinsics) {
  std::vector<gtsam::Pose3> tagsToDraw;
  // Simulate onboard camera measurements of front tag points
  const auto [ocSymbol, ocPose] = cameras.at("onboard_camera");
  Camera onboardCamera(ocPose, *intrinsics);

  // Edge length of square fiducial tags
  double tagSize = tags.getTagSize();

  // Onboard camera image
  for (const std::string& side : {"left", "right"}) {
    const auto tagPose = tags.getPose(side, "front");
    tagsToDraw.push_back(tagPose);

    // check PnP
    if (true) {
      auto camToTag = simulateEstimatedPose(tagPose, onboardCamera, tagSize);
      auto estCameraPose = tagPose * camToTag.inverse();  // in vehicle frame
      assert(estCameraPose.equals(ocPose, 1e-6));
    }
  }
  draw(tagsToDraw, onboardCamera, "onboard_camera");
  tagsToDraw.clear();

  // External camera images
  for (const std::string& side : {"left", "right"}) {
    for (int i = 0; i < cameras.countViews(side); ++i) {
      const auto cameraPose = cameras.getPose("external", side, i);
      Camera camera(cameraPose, *intrinsics);
      for (const std::string& zone : {"front", "front_wheel", "rear_wheel"}) {
        const auto tagPose = tags.getPose(side, zone);
        tagsToDraw.push_back(tagPose);

        // check PnP
        if (true) {
          auto camToTag = simulateEstimatedPose(tagPose, camera, tagSize);
          auto estCameraPose =
              tagPose * camToTag.inverse();  // in vehicle frame
          assert(estCameraPose.equals(cameraPose, 1e-6));
        }
      }
      draw(tagsToDraw, camera, cameras.getName("external", side, i));
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

  // Calibration model for projection
  gtsam::Cal3_S2::shared_ptr intrinsics = simulateCamera();

  // Draw projected tags to image files
  drawImages(cameras, tags, intrinsics);

  return 0;
}
