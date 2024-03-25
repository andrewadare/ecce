#include <ecce/simulation.hpp>

int main(int argc, char* argv[]) {
  if (argc != 1) {
    cout << "Usage: " << argv[0] << " (no args)" << endl;
    return 1;
  }

  PoseMap tagPoses = simulateTagPoses();
  PoseMap cameraPoses = lookAtTags(tagPoses);

  // Calibration model for projection
  gtsam::Cal3_S2::shared_ptr intrinsics = simulateCamera();

  gtsam::NonlinearFactorGraph graph;

  addMeasurements(cameraPoses, tagPoses, intrinsics, graph);

  return 0;
}
