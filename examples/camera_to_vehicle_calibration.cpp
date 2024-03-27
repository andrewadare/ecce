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

  // Model 1cm positional uncertainty on the wheel tag positions
  const double tagError = 0.01;
  std::normal_distribution<double> gauss(0.0, tagError);
  std::mt19937_64 rng;

  gtsam::NonlinearFactorGraph graph;

  // Simulate noise-free ground truth poses for all fiducial tags and cameras
  TagCollection tags = simulateTags();
  CameraCollection cameras = simulateCameras(tags);

  // Declare empty containers for the measured/estimated counterparts to all
  // ground truth objects.
  // TagCollection observedTags(tags.getTagSize());
  // CameraCollection observedCameras;

  // Intrinsic calibration model for camera projection
  gtsam::Cal3_S2::shared_ptr intrinsics = simulateCamera();

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

  // Estimate external camera poses in vehicle frame
  std::unordered_map<std::string, gtsam::Pose3> poseEstimates;
  for (const std::string& side : {"left", "right"}) {
    for (int i = 0; i < cameras.countViews(side); ++i) {
      const auto cameraName = cameras.getName("external", side, i);

      // TODO add noise
      // TODO average front & rear
      const auto frontTagPose = tags.getPose(side, "front_wheel");
      // const auto rearTagPose = tags.getPose(side, "rear_wheel");
      const auto& tagPose = frontTagPose;

      const auto name =
          joinNames(tags.getName(side, "front_wheel"), cameraName);

      // Wheel tag -> external camera
      auto estCameraPose = tagPose * pnpEstimates.at(name).inverse();
      poseEstimates[cameraName] = estCameraPose;

      // True without noise
      assert(estCameraPose.equals(cameras.getPose("external", side, i), 1e-6));
    }
  }

  // Estimate poses of the two front tags in vehicle frame.
  // Ideally we should average all external camera views, but here just
  // use one.
  for (const std::string& side : {"left", "right"}) {
    const auto tagName = tags.getName(side, "front");
    const auto cameraName = cameras.getName("external", side, 0);
    const auto cameraPose = poseEstimates.at(cameraName);
    const auto name = joinNames(tagName, cameraName);

    // Wheel tag -> external camera -> front tag
    auto estTagPose = cameraPose * pnpEstimates.at(name);
    poseEstimates[tagName] = estTagPose;

    assert(estTagPose.equals(tags.getPose(side, "front"), 1e-6));
  }

  // Onboard camera
  for (const std::string& side : {"left", "right"}) {
    const auto tagName = tags.getName(side, "front");
    const auto cameraName = cameras.getName("onboard");
    const auto name = joinNames(tagName, cameraName);
    const auto tagPose = poseEstimates.at(tagName);
    auto estCameraPose = tagPose * pnpEstimates.at(name).inverse();
    poseEstimates[cameraName] = estCameraPose;
    assert(estCameraPose.equals(cameras.getPose("onboard"), 1e-6));
  }

  addTagPriors(tags, tagError, graph);
  graph.print("Graph:\n");

  // Create estimates of all values as a starting point for optimization
  // TODO: this is GT - use observedCameras
  gtsam::Values estimates;
  for (const auto& [name, entry] : cameras.all()) {
    const auto& [symbol, pose] = entry;
    estimates.insert<gtsam::Pose3>(symbol, pose);
  }
  // TODO: use observedTags and remove noise
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
