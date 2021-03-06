/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-06
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The class finds lines in a segmented image.
*/

#ifndef LINE_DETECTOR_HPP
#define LINE_DETECTOR_HPP

// System
#include <vector>

// OpenCV
#include <opencv2/core/core.hpp>

// ROS
#include <ros/ros.h>

// Messages
#include <sensor_msgs/CameraInfo.h>
#include <vision_msgs/LineDetections.h>

// Local
#include "vision/lsd_opencv.hpp"

class LineDetector {
 public:
  explicit LineDetector(ros::NodeHandle& nh);
  void ProcessImage(const cv::Mat& image,
                    const sensor_msgs::CameraInfo& cam_info);
  void ThresholdImage(cv::Mat& image);
 private:
  cv::Mat image_;
  std::vector<cv::Vec4i> lines_;
  cv::Ptr<cv::LineSegmentDetector> lsd_;
  // Information about the detected lines
  vision_msgs::LineDetections line_detections_;
  // Publisher of the detected lines
  ros::Publisher line_detections_pub_;
  unsigned int frames_count_;
};

#endif
