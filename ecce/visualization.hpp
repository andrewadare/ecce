#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>

#include <ecce/geometry.hpp>
#include <string>
#include <vector>

// Draw projected points on image and save to <name>.png
void draw(std::vector<std::vector<gtsam::Point2>>& imgPoints,
          const std::string& name);

// Project tags to image and save to <name>.png
void draw(const std::vector<gtsam::Pose3>& tags, const Camera& camera,
          const std::string& name);
