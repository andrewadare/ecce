#include <ecce/estimation.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

using std::cout, std::endl;

gtsam::Pose3 pnp(const std::vector<gtsam::Point3>& worldPoints,
                 const std::vector<gtsam::Point2>& imagePoints,
                 gtsam::Cal3_S2::shared_ptr intrinsics) {
  // No lens distortion in this model
  cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);

  // Pose params assigned in solvePnP
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

  // Intrinsic camera parameters
  // Either (a) imagePoints should be in normalized coords with K = I(3)
  // or (b) K should have the correct intrinsics.
  // This implementation assumes (b).

  cv::Mat K = cv::Mat::zeros(3, 3, CV_64FC1);
  K.at<double>(0, 0) = intrinsics->fx();
  K.at<double>(1, 1) = intrinsics->fy();
  K.at<double>(0, 2) = intrinsics->px();
  K.at<double>(1, 2) = intrinsics->py();
  K.at<double>(2, 2) = 1;

  // Copy to OpenCV types
  // TODO: any better way?
  std::vector<cv::Point3d> cvWorldPoints;
  std::vector<cv::Point2d> cvImagePoints;
  for (const auto& worldPoint : worldPoints) {
    cvWorldPoints.push_back({worldPoint[0], worldPoint[1], worldPoint[2]});
  }
  for (const auto& imagePoint : imagePoints) {
    cvImagePoints.push_back({imagePoint[0], imagePoint[1]});
  }

  // Start estimation from scratch
  bool useExtrinsicGuess = false;

  // Solve perspective n-point problem.
  cv::solvePnP(cvWorldPoints, cvImagePoints, K, distCoeffs, rvec, tvec,
               useExtrinsicGuess, cv::SOLVEPNP_ITERATIVE);

  return gtsam::Pose3(
      gtsam::Rot3::Rodrigues(rvec.at<double>(0, 0), rvec.at<double>(1, 0),
                             rvec.at<double>(2, 0)),
      {tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)});
}
