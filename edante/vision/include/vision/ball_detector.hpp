/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-01
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The class finds the ball in a segmented image.
*/
#ifndef BALL_DETECTOR_HPP
#define BALL_DETECTOR_HPP

#include <vector>

#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/core/core.hpp>

#include "vision/BallDetection.h"

class BallDetector {
 public:
  explicit BallDetector(ros::NodeHandle& nh);
  void ProcessImage(const sensor_msgs::ImageConstPtr& image,
                    const sensor_msgs::CameraInfoConstPtr& cam_info);
 private:
  cv::Mat image_;
  std::vector< std::vector<cv::Point> > contours_;
  vision::BallDetection ball_detection_;
  ros::Publisher ball_detection_pub_;
  image_geometry::PinholeCameraModel cam_model_;

  void GetMat(const sensor_msgs::ImageConstPtr& image, cv::Mat& mat);
  void ThresholdImage(cv::Mat& image);
  void FindBall(cv::Mat& image, vision::BallDetection& ball_detection);
  void EstimatePos3D(const sensor_msgs::CameraInfoConstPtr& cam_info,
                     vision::BallDetection& ball_detection);
};
#endif
