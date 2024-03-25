#include <ecce/visualization.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

void draw(std::vector<std::vector<gtsam::Point2>> imgPoints,
          const std::string& name) {
  cv::Mat im = cv::Mat::zeros(960, 1280, CV_8UC3);

  for (const auto& tag : imgPoints) {
    // TL corner in white
    cv::circle(im, {int(tag[0][0]), int(tag[0][1])}, 6, {255, 255, 255}, -1);

    // Line btwn first two points to check orientation
    cv::line(im, {int(tag[0][0]), int(tag[0][1])},
             {int(tag[1][0]), int(tag[1][1])}, {255, 255, 255});

    for (const auto& p : tag) {
      cv::circle(im, {int(p[0]), int(p[1])}, 4, {0, 255, 0}, -1);
    }
  }

  cv::imwrite(cv::format("%s.png", name.c_str()), im);
}

void draw(const std::vector<gtsam::Pose3>& tagPoses, const Camera& camera,
          const std::string& name) {
  const double tagSize = 0.5;  // edge length
  cv::Mat im = cv::Mat::zeros(960, 1280, CV_8UC3);
  const std::vector<cv::Scalar> rgb = {{0, 0, 255}, {0, 255, 0}, {255, 0, 0}};

  for (const auto& tagPose : tagPoses) {
    // Project center of tag as origin
    const auto center = camera.project(tagPose.translation());
    const auto origin = cv::Point2i(center[0], center[1]);

    // Draw pose
    const auto e = axisPoints(tagPose, tagSize / 2);
    for (int i = 0; i < 3; ++i) {
      const auto u = camera.project(e[i]);
      cv::line(im, origin, {int(u[0]), int(u[1])}, rgb[i], 2);
    }

    // Project tag corner points
    std::vector<cv::Point2i> corners;
    for (const auto& point : tagCorners(tagPose, tagSize)) {
      const auto p = camera.project(point);
      corners.push_back({int(p[0]), int(p[1])});
    }

    // Check corner point ordering - should be TL, TR, BR, BL
    if (false) {
      // Top left corner in magenta
      cv::circle(im, corners[0], 6, {255, 0, 255}, -1);

      // Line joining first two points
      cv::line(im, corners[0], corners[1], {255, 255, 255});
    }

    // Draw corner points
    for (const cv::Point2i& p : corners) {
      cv::circle(im, p, 2, {255, 255, 255}, -1);
    }

    cv::imwrite(cv::format("%s.png", name.c_str()), im);
  }
}
