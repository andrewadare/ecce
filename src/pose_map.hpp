#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <string>
#include <unordered_map>

using PoseMap =
    std::unordered_map<std::string, std::pair<gtsam::Symbol, gtsam::Pose3>>;

// Add a named, labeled camera pose to a PoseMap
void addCamera(const std::string& name, const std::array<double, 3>& ypr,
               const gtsam::Point3& position, PoseMap& poses);

// Add a named, labeled fiducial tag pose to a PoseMap
void addTag(const std::string& name, const std::array<double, 3>& ypr,
            const gtsam::Point3& position, PoseMap& poses);
