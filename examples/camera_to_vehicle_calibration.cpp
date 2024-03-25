#include <ecce/simulation.hpp>

int main(int argc, char* argv[]) {
  if (argc != 1) {
    cout << "Usage: " << argv[0] << " (no args)" << endl;
    return 1;
  }

  PoseMap tagPoses = simulateTagPoses();
  PoseMap cameraPoses = lookAtTags(tagPoses);

  gtsam::NonlinearFactorGraph graph;

  addMeasurements(cameraPoses, tagPoses, graph);

  return 0;
}
