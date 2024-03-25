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

// Project tags to image and save to <name>.png
void draw(const std::vector<gtsam::Pose3>& tagPoses, const Camera& camera,
          const std::string& name) {
  cv::Mat im = cv::Mat::zeros(960, 1280, CV_8UC3);

  for (const auto& tagPose : tagPoses) {
    for (const auto& point : tagCorners(tagPose, 0.5)) {
      const auto p = camera.project(point);
      cv::circle(im, {int(p[0]), int(p[1])}, 4, {0, 255, 0}, -1);
    }
    cv::imwrite(cv::format("%s.png", name.c_str()), im);
  }
}
