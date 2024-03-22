#include "simulation.hpp"

int main(int argc, char* argv[]) {
  if (argc != 1) {
    cout << "Usage: " << argv[0] << " (no args)" << endl;
    return 1;
  }

  PoseMap cameraPoses = simulateCameraPoses();
  PoseMap tagPoses = simulateTagPoses();

  gtsam::NonlinearFactorGraph graph;

  addMeasurements(cameraPoses, tagPoses, graph);

  return 0;
}
