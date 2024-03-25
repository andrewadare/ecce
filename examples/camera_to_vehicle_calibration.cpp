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

  // Onboard camera image
  for (const auto& side : sides) {
    std::stringstream tagName;
    tagName << side << "_front";
    const auto [tagSymbol, tagPose] = tagPoses.at(tagName.str());
    tagsToDraw.push_back(tagPose);
  }
  draw(tagsToDraw, onboardCamera, "onboard_camera");
  tagsToDraw.clear();

  // External camera images
  for (const auto& side : sides) {
    for (int i = 0; i < countViews(cameraPoses, side); ++i) {
      std::stringstream cameraName;
      cameraName << side << "_external_camera_" << i;
      const auto [cameraSymbol, cameraPose] = cameraPoses.at(cameraName.str());
      Camera camera(cameraPose, *intrinsics);

      for (const auto& tagZone : tagZones) {
        std::stringstream tagName;
        tagName << side << "_" << tagZone;
        const auto [tagSymbol, tagPose] = tagPoses.at(tagName.str());
        tagsToDraw.push_back(tagPose);
      }
      draw(tagsToDraw, camera, cameraName.str());
      tagsToDraw.clear();
    }
  }
}

int main(int argc, char* argv[]) {
  if (argc != 1) {
    cout << "Usage: " << argv[0] << " (no args)" << endl;
    return 1;
  }

  PoseMap tagPoses = simulateTagPoses();
  PoseMap cameraPoses = lookAtTags(tagPoses);

  // Calibration model for projection
  gtsam::Cal3_S2::shared_ptr intrinsics = simulateCamera();

  // Draw projected tags to image files
  drawImages(cameraPoses, tagPoses, intrinsics);

  gtsam::NonlinearFactorGraph graph;

  addMeasurements(cameraPoses, tagPoses, intrinsics, graph);

  return 0;
}
