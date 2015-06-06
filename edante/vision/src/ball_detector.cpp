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

using cv::Rect;
using cv::Point2d;
using cv::Point3d;

const double BallDetector::kBallRadius = 0.03;
/**
 * @brief Creates the BallDetection publisher and the GetTransform client.
 *
 * @param nh The node handle to be used.
 */
BallDetector::BallDetector(ros::NodeHandle& nh) :
  ball_detection_pub_(nh.advertise<vision_msgs::BallDetection>("ball", 5)),
  transform_client_(nh.serviceClient<motion_msgs::GetTransform>(
                      "/motion/get_transform", true)) {
}
/**
 * @brief Detect the ball in the image.
 *
 * @param image The image to be processed.
 * @param cam_info The camera calibration information.
 */
void BallDetector::ProcessImage(const cv::Mat& image,
                                const sensor_msgs::CameraInfo& cam_info) {
  // Copy the input image
  image_ = image.clone();

  // Make only ball pixels white
  ThresholdImage(image_);

  // Update the ball detection info
  ball_detection_.header.stamp = cam_info.header.stamp;
  ball_detection_.header.frame_id = cam_info.header.frame_id;

  // Find the ball in the thresholded image
  FindBall(image_, ball_detection_);

  // Calculate the 3D ball position
  EstimateBallPos3D(cam_info, ball_detection_);

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

  ball.is_detected = (max_index != -1);

  if (ball.is_detected) {
    Rect r = cv::boundingRect(contours_[max_index]);

    ball.area = max_area;
    ball.pos_image.x = r.x + r.width / 2;
    ball.pos_image.y = r.y + r.height / 2;
    ball.radius = (r.width + r.height + 4) / 4;
    ball.certainty = max_circularity;
  }
}
/**
 * @brief Estimate the 3D ball position based on the known size of the ball.
 *
 * @param cam_info Calibration information for the camera which captured
 *                 the image.
 * @param ball The estimated 3D position will be stored here.
 */
void BallDetector::EstimateBallPos3D(
  const sensor_msgs::CameraInfo& cam_info,
  vision_msgs::BallDetection& ball) {
  // Update the camera model
  cam_model_.fromCameraInfo(cam_info);
  // Calculate the undistorted 2d position
  Point2d pos_2d = cam_model_.rectifyPoint(
                     Point2d(ball.pos_image.x,
                             ball.pos_image.y));
  // Project the point to the z = 1m plane
  Point3d cam_pos_3d = cam_model_.projectPixelTo3dRay(pos_2d);

  // Calculate the physical radius of a ball at 1m distance (z = 1m) with
  // the same size in pixels as the detected one. After undistortion the
  // pinhole model is linear, so we should simply rescale such that the
  // resulting size is the same as the actual ball.
  double radius_z1m = cam_model_.getDeltaX(ball.radius, 1.0);
  cam_pos_3d *= kBallRadius / radius_z1m;

  // Transform the point from the optical camera frame to the Nao camera frame
  ball.pos_camera.x = cam_pos_3d.z;
  ball.pos_camera.y = -cam_pos_3d.x;
  ball.pos_camera.z = -cam_pos_3d.y;

  // Get the camera frame to robot frame transformation
  motion_msgs::GetTransform srv;
  if (ball.header.frame_id == "top_camera") {
    srv.request.name = "CameraTop";
  } else {
    srv.request.name = "CameraBottom";
  }
  srv.request.space = 2;  // FRAME_ROBOT
  srv.request.use_sensor_values = true;

  if (!transform_client_.call(srv)) {
    ROS_WARN("Unable to call get_transform.");
    // Set invalid ball position
    ball.pos_robot.x = ball.pos_robot.y = ball.pos_robot.z = 0;
    return;
  }

  // Apply the transformation to get the ball in the robot frame.
  vector<float>& T = srv.response.transform;
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


