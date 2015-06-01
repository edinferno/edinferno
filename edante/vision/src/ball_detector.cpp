/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-01
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The class finds the ball in a segmented image.
*/
#include "vision/ball_detector.hpp"

#include <opencv2/imgproc/imgproc.hpp>

#include "vision/BallDetection.h"

#include "camera/color_table.hpp"


using cv::Rect;
using cv::Point2d;
using cv::Point3d;

BallDetector::BallDetector(ros::NodeHandle& nh) :
  ball_detection_pub_(nh.advertise<vision::BallDetection>("ball", 5)) {

}

void BallDetector::ProcessImage(
  const sensor_msgs::ImageConstPtr& image,
  const sensor_msgs::CameraInfoConstPtr& cam_info) {
  GetMat(image, image_);
  ThresholdImage(image_);


  ball_detection_.header.stamp = image->header.stamp;
  ball_detection_.header.frame_id = image->header.frame_id;
  FindBall(image_, ball_detection_);
  EstimatePos3D(cam_info, ball_detection_);

  ball_detection_pub_.publish(ball_detection_);
}

void BallDetector::GetMat(const sensor_msgs::ImageConstPtr& image,
                          cv::Mat& mat) {
  if ((unsigned)mat.cols != image->width ||
      (unsigned)mat.rows != image->height) {
    mat = cv::Mat(image->height, image->width, CV_8UC1);
  }
  memcpy(mat.data, image->data.data(), image->data.size());
}

void BallDetector::ThresholdImage(cv::Mat& image) {
  int data_len = image.rows * image.step;
  for (int i = 0; i < data_len; ++i) {
    image.data[i] = (image.data[i] == Ball) ? 255 : 0;
  }
}

void BallDetector::FindBall(cv::Mat& image,
                            vision::BallDetection& ball_detection) {
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

  ball_detection.is_detected = (max_index != -1);

  if (ball_detection.is_detected) {
    Rect r = cv::boundingRect(contours_[max_index]);

    ball_detection.area = max_area;
    ball_detection.pos_2d.x = r.x + r.width / 2;
    ball_detection.pos_2d.y = r.y + r.height / 2;
    ball_detection.radius = (r.width + r.height + 4) / 4;
    ball_detection.certainty = max_circularity;
  }
}

void BallDetector::EstimatePos3D(
  const sensor_msgs::CameraInfoConstPtr& cam_info,
  vision::BallDetection& ball_detection) {
  cam_model_.fromCameraInfo(cam_info);
  Point2d pos_2d = cam_model_.rectifyPoint(
                     Point2d(ball_detection.pos_2d.x,
                             ball_detection.pos_2d.y));
  Point3d pos_3d = cam_model_.projectPixelTo3dRay(pos_2d);
  double delta_x = cam_model_.getDeltaX(ball_detection.radius, 1.0);
  pos_3d *= 0.03 / delta_x;
  ball_detection.pos_3d.x = pos_3d.x;
  ball_detection.pos_3d.y = pos_3d.y;
  ball_detection.pos_3d.z = pos_3d.z;
}
