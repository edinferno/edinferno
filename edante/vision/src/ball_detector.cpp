/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-01
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The class finds the ball in a segmented image.
*/
#include "vision/ball_detector.hpp"

// Camera
#include <camera/color_table.hpp>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>

// Messages
#include "motion_msgs/GetTransform.h"

using std::vector;

using cv::Mat;
using cv::Mat_;
using cv::Rect;
using cv::Point2d;
using cv::Point3d;


/**
 * @brief Creates the BallDetection publisher and the GetTransform client.
 *
 * @param nh The node handle to be used.
 */
BallDetector::BallDetector(ros::NodeHandle& nh) :
  kf_2d_(kImageKFStateSize, kImageKFMeasurementSize),
  ball_detection_pub_(nh.advertise<vision_msgs::BallDetection>("ball", 5)) {

  // Transition State Matrix A
  // [ 1 0 dT 0  0 0 ]
  // [ 0 1 0  dT 0 0 ]
  // [ 0 0 1  0  0 0 ]
  // [ 0 0 0  1  0 0 ]
  // [ 0 0 0  0  1 0 ]
  // [ 0 0 0  0  0 1 ]
  kf_2d_.transitionMatrix.at<float>(0, 0) = 1;
  kf_2d_.transitionMatrix.at<float>(1, 1) = 1;
  kf_2d_.transitionMatrix.at<float>(2, 2) = 1;
  kf_2d_.transitionMatrix.at<float>(3, 3) = 1;
  kf_2d_.transitionMatrix.at<float>(4, 4) = 1;
  kf_2d_.transitionMatrix.at<float>(5, 5) = 1;
  // TODO(svepe): Update dT at each processing step!
  kf_2d_.transitionMatrix.at<float>(0, 2) = 1.0 / 30.0;
  kf_2d_.transitionMatrix.at<float>(1, 3) = 1.0 / 30.0;

  // Measure Matrix H
  // [ 1 0 0 0 0 0 ]
  // [ 0 1 0 0 0 0 ]
  // [ 0 0 0 0 1 0 ]
  // [ 0 0 0 0 0 1 ]
  kf_2d_.measurementMatrix.at<float>(0, 0) = 1.0f;
  kf_2d_.measurementMatrix.at<float>(1, 1) = 1.0f;
  kf_2d_.measurementMatrix.at<float>(2, 4) = 1.0f;
  kf_2d_.measurementMatrix.at<float>(3, 5) = 1.0f;

  // Process Noise Covariance Matrix Q
  //
  // The result will be smoother with lower values. If the value of Q
  // is high, it means that the measured signal varies quickly and
  // the filter needs to be adaptable. If it is small, then big variations
  // will be attributed to noise in the measure.


  // [ Var(x)   0      0        0         0      0    ]
  // [   0    Var(y)   0        0         0      0    ]
  // [   0      0    Var(v_x)   0         0      0    ]
  // [   0      0      0      Var(Ev_y)   0      0    ]
  // [   0      0      0        0       Var(w)   0    ]
  // [   0      0      0        0         0    Var(h) ]
  kf_2d_.processNoiseCov.at<float>(0, 0) = 1e-2;
  kf_2d_.processNoiseCov.at<float>(1, 1) = 1e-2;
  kf_2d_.processNoiseCov.at<float>(2, 2) = 1e-1;
  kf_2d_.processNoiseCov.at<float>(3, 3) = 1e-1;
  kf_2d_.processNoiseCov.at<float>(4, 4) = 1e-2;
  kf_2d_.processNoiseCov.at<float>(5, 5) = 1e-2;

  // Measures Noise Covariance Matrix R
  //
  // The result will be smoother with higher values. If the value of R is high
  // (compared to Q), it will indicate that the measuring is noisy so it will
  // be filtered more.
  kf_2d_.measurementNoiseCov.at<float>(0, 0) = 1e-1;
  kf_2d_.measurementNoiseCov.at<float>(1, 1) = 1;
  kf_2d_.measurementNoiseCov.at<float>(2, 2) = 1;
  kf_2d_.measurementNoiseCov.at<float>(3, 3) = 1e-1;

  // kf_2d_.errorCovPost = Mat_<float>::eye(kImageKFStateSize,
  //                                        kImageKFStateSize) * 1000;
}
/**
 * @brief Detect the ball in the image.
 *
 * @param image The image to be processed.
 * @param header The header which will be used for the published
 *               ball detection.
 * @param cam_model The calibrated camera model.
 * @param transform The camera frame transformation at frame capture
 */
void BallDetector::ProcessImage(
  cv::Mat& image,
  int horizon,
  const std_msgs::Header& header,
  const image_geometry::PinholeCameraModel& cam_model,
  const std::vector<float>& transform) {
  // Make a subimage under the horizon line
  horizon_ = horizon;
  image_ = Mat(image.rows - horizon_,
               image.cols, CV_8UC1,
               image.data + horizon_ * image.cols);

  // Make only ball pixels white
  ThresholdImage(image_);

  // Update the ball detection info
  ball_detection_.header.stamp = header.stamp;
  ball_detection_.header.frame_id = header.frame_id;

  // Find the ball in the thresholded image
  FindBall(image_, ball_detection_);

  // Calculate the 3D ball position
  EstimateBallPos3D(cam_model, transform, ball_detection_);

  // Publish the detected ball info
  ball_detection_pub_.publish(ball_detection_);
}
/**
 * @brief Threshold the image such that only 'Ball' pixels are white.
 *
 * @param image The image to be thresholded.
 */
void BallDetector::ThresholdImage(cv::Mat& image) {
  int data_len = image.rows * image.step;
  for (int i = 0; i < data_len; ++i) {
    image.data[i] = (image.data[i] == Ball) ? 255 : 0;
  }
}
/**
 * @brief Finds a ball in a thresholded image.
 * @details Uses cv::findcontours to detect objects. Filters them based on
 *          on size, circumference length and circularity. If none of the
 *          contours meets the requirements then the ball is not found.
 *          If multiple balls are present, the larger one is selected.
 *
 * @param image The image to be processed. It should be thresholded such that
 *              only 'Ball' pixels are white.
 * @param ball The results of the analysis are stored here including 2D ball
 *             position and radius if the ball is found.
 */
void BallDetector::FindBall(cv::Mat& image,
                            vision_msgs::BallDetection& ball) {
  cv::findContours(image, contours_, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  double max_area = -1;
  double max_circularity = 0;
  int max_index = -1;
  for (size_t i = 0; i < contours_.size(); ++i) {
    // Skip contours which are too short (noise)
    if (contours_[i].size() < 10) continue;

    // Skip contours which are too small (noise)
    double area = cv::contourArea(contours_[i]);
    if (area < 20) continue;

    // Skip contours which are not circular enough
    double circularity = 4 * M_PI * area / (contours_[i].size() *
                                            contours_[i].size());
    if (circularity < 0.30f) continue;

    // If the contour fits all criteria and it is the larges so far
    if (area > max_area) {
      max_area = area;
      max_circularity = circularity;
      max_index = i;
    }
  }

  Mat_<float> state = kf_2d_.predict();

  if (max_index != -1) {
    Rect r = cv::boundingRect(contours_[max_index]);
    Mat_<float> m(kImageKFMeasurementSize, 1);
    m.at<float>(0) = r.x + r.width / 2;
    m.at<float>(1) = r.y + r.height / 2 + horizon_;
    m.at<float>(2) = r.width;
    m.at<float>(3) = r.height;
    state = kf_2d_.correct(m);
  } else {
    kf_2d_.statePost = kf_2d_.statePre;
    kf_2d_.errorCovPost = kf_2d_.errorCovPre;
  }

  for (int i = 0; i < kf_2d_.errorCovPost.rows; ++i) {
    for (int j = 0; j < kf_2d_.errorCovPost.cols; ++j) {
      if (kf_2d_.errorCovPost.at<float>(i, j) > kMaxStateVariance)
        kf_2d_.errorCovPost.at<float>(i, j) = kMaxStateVariance;
    }
  }

  float pos_var = fabs(kf_2d_.errorCovPost.at<float>(0, 0)) +
                  fabs(kf_2d_.errorCovPost.at<float>(1, 1));

  ball.is_detected = pos_var < kMaxPositionVariance;
  if (ball.is_detected) {
    ball.area = max_area;
    ball.pos_image.x = state.at<float>(0);
    ball.pos_image.y = state.at<float>(1);
    ball.radius = (state.at<float>(4) + state.at<float>(5) + 4) / 4;
    ball.certainty = max_circularity;
  }
}
/**
 * @brief Estimate the 3D ball position based on the known size of the ball.
 *
 * @param cam_model The calibrated camera model.
 * @param transform The camera frame transformation at the time of capturing
 *                  the frame.
 * @param ball The estimated 3D position will be stored here.
 */
void BallDetector::EstimateBallPos3D(
  const image_geometry::PinholeCameraModel& cam_model,
  const std::vector<float>& transform,
  vision_msgs::BallDetection& ball) {
  // Calculate the undistorted 2d position
  Point2d pos_2d = cam_model.rectifyPoint(
                     Point2d(ball.pos_image.x,
                             ball.pos_image.y));
  // Project the point to the z = 1m plane
  Point3d cam_pos_3d = cam_model.projectPixelTo3dRay(pos_2d);

  // Calculate the physical radius of a ball at 1m distance (z = 1m) with
  // the same size in pixels as the detected one. After undistortion the
  // pinhole model is linear, so we should simply rescale such that the
  // resulting size is the same as the actual ball.
  double radius_z1m = cam_model.getDeltaX(ball.radius, 1.0);
  cam_pos_3d *= kBallRadius / radius_z1m;

  // Transform the point from the optical camera frame to the Nao camera frame
  ball.pos_camera.x = cam_pos_3d.z;
  ball.pos_camera.y = -cam_pos_3d.x;
  ball.pos_camera.z = -cam_pos_3d.y;

  // Apply the transformation to get the ball in the robot frame.
  const vector<float>& T = transform;
  ball.pos_robot.x = T[0] * ball.pos_camera.x +
                     T[1] * ball.pos_camera.y +
                     T[2] * ball.pos_camera.z + T[3];
  ball.pos_robot.y = T[4] * ball.pos_camera.x +
                     T[5] * ball.pos_camera.y +
                     T[6] * ball.pos_camera.z + T[7];
  ball.pos_robot.z = T[8] * ball.pos_camera.x +
                     T[9] * ball.pos_camera.y +
                     T[10] * ball.pos_camera.z + T[11];
}
