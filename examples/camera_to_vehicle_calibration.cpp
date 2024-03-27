#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <ecce/collections.hpp>
#include <ecce/estimation.hpp>
#include <ecce/simulation.hpp>
#include <random>

// Model that the wheel markers were mounted on the vehicle with some positional
// accuracy. This anchors them to the vehicle frame and constrains their
// position during optimization.
void addTagPriors(const TagCollection& tags, const double& positionError,
                  gtsam::NonlinearFactorGraph& graph) {
  auto wheelTagUncertainty =
      gtsam::noiseModel::Isotropic::Sigma(3, positionError);

  for (const std::string& side : {"left", "right"}) {
    for (const std::string& tagZone : {"front_wheel", "rear_wheel"}) {
      const auto tagSymbol = tags.getSymbol(side, tagZone);
      const auto tagPose = tags.getPose(side, tagZone);

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

  // Simulate noise-free ground truth poses for all fiducial tags and cameras
  TagCollection tags = simulateTags();
  CameraCollection cameras = simulateCameras(tags);

  // Declare empty containers for the measured/estimated counterparts to all
  // ground truth objects.
  TagCollection observedTags(tags.getTagSize());
  CameraCollection observedCameras;

  // Calibration model for projection
  gtsam::Cal3_S2::shared_ptr intrinsics = simulateCamera();

  // Build a factor graph:
  // - Simulate tag detections in both onboard and external cameras
  // - Model 1cm positional uncertainty on the wheel tag positions
  const double tagError = 0.01;
  gtsam::NonlinearFactorGraph graph;

  // Assigns measurement factors to graph and returns corner points for initial
  // camera pose estimation
  PointMap measurements =
      simulateMeasurements(cameras, tags, intrinsics, graph);

  // Find camera poses in tag coordinate frames using 4-point estimation.
  // The corners of a tag with Pose3::Identity() serve as the 3D points.
  std::unordered_map<std::string, gtsam::Pose3> pnpEstimates;
  const auto tagCorners = localTagCorners(tags.getTagSize());
  for (const auto& [name, imagePoints] : measurements) {
    pnpEstimates[name] = pnp(tagCorners, imagePoints, *intrinsics);
    // const auto [tagName, cameraName] = splitName(name);
    // cout << tagName << " " << cameraName << endl;
  }

  addTagPriors(tags, tagError, graph);
  graph.print("Graph:\n");

  // Create estimates of all values as a starting point for optimization
  gtsam::Values estimates;
  std::normal_distribution<double> gauss(0.0, tagError);
  for (const auto& [name, entry] : cameras.all()) {
    const auto& [symbol, pose] = entry;
    // TODO: this is GT - use perturbed pose
    estimates.insert<gtsam::Pose3>(symbol, pose);
  }
  for (const auto& [name, entry] : tags.all()) {
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
