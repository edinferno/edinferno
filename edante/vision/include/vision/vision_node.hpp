/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-01
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The vision subscribes to the segmented image and
*             executes all the visual processing needed.
*/
#ifndef VISION_NODE_HPP
#define VISION_NODE_HPP

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "vision/ball_detector.hpp"

class VisionNode {
 public:
  VisionNode();
  void Spin();
 private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;

  BallDetector ball_detector_;

  void CameraCallback(const sensor_msgs::ImageConstPtr& image,
                      const sensor_msgs::CameraInfoConstPtr& cam_info);
};

#endif
