#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <ecce/simulation.hpp>
#include <ecce/visualization.hpp>
#include <random>

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

// Model that the wheel markers were mounted on the vehicle with some positional
// accuracy. This anchors them to the vehicle frame and constrains their
// position during optimization.
void addTagPriors(const PoseMap& tagPoses, const double& positionError,
                  gtsam::NonlinearFactorGraph& graph) {
  auto wheelTagUncertainty =
      gtsam::noiseModel::Isotropic::Sigma(3, positionError);

  const std::string sides[] = {"left", "right"};
  const std::string tagZones[] = {"front_wheel", "rear_wheel"};

  for (const auto& side : sides) {
    for (const auto& tagZone : tagZones) {
      std::stringstream tagName;
      tagName << side << "_" << tagZone;
      const auto [tagSymbol, tagPose] = tagPoses.at(tagName.str());
      graph.addPrior(tagSymbol, tagPose.translation(), wheelTagUncertainty);
    }
  }
}

int main(int argc, char* argv[]) {
  if (argc != 1) {
    cout << "Usage: " << argv[0] << " (no args)" << endl;
    return 1;
  }

  std::mt19937_64 rng;

  PoseMap tagPoses = simulateTagPoses();
  PoseMap cameraPoses = lookAtTags(tagPoses);

  // Calibration model for projection
  gtsam::Cal3_S2::shared_ptr intrinsics = simulateCamera();

  // Draw projected tags to image files
  drawImages(cameraPoses, tagPoses, intrinsics);

  // Build a factor graph:
  // - Simulate tag detections in both onboard and external cameras
  // - Model 1cm positional uncertainty on the wheel tag positions
  const double tagError = 0.01;
  gtsam::NonlinearFactorGraph graph;
  addMeasurements(cameraPoses, tagPoses, intrinsics, graph);
  addTagPriors(tagPoses, tagError, graph);
  // graph.print("Graph:\n");

  // Create estimates of all values as a starting point for optimization
  gtsam::Values estimates;
  std::normal_distribution<double> gauss(0.0, tagError);
  for (const auto& [name, entry] : cameraPoses) {
    const auto& [symbol, pose] = entry;
    // TODO: this is GT - use perturbed pose
    estimates.insert<gtsam::Pose3>(symbol, pose);
  }
  for (const auto& [name, entry] : tagPoses) {
    const auto& [symbol, pose] = entry;
    const auto noise = gtsam::Point3(gauss(rng), gauss(rng), gauss(rng));
    estimates.insert<gtsam::Point3>(symbol, pose.translation() + noise);
  }
  estimates.print("Initial Estimates:\n");

  gtsam::Values result =
      gtsam::LevenbergMarquardtOptimizer(graph, estimates).optimize();

  // Throws IndeterminantLinearSystemException.
  // Should resolve when the result has realistic uncertainty.
  //
  // Calculate and print marginal covariances for all variables
  // gtsam::Marginals marginals(graph, result);
  // for (const auto& [key, pose] : result.extract<gtsam::Pose3>()) {
  //   cout << marginals.marginalCovariance(key) << endl;
  // }

  // result.print("Final results:\n");
  cout << "initial error = " << graph.error(estimates) << endl;
  cout << "final error = " << graph.error(result) << endl;

  // Save to graphviz dot file
  // Force-directed placement rendering: "fdp c2v.dot -Tpdf -O"
  graph.saveGraph("c2v.dot", result);

  return 0;
}
