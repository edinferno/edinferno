/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-01
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The vision subscribes to the segmented image and
*             executes all the visual processing needed.
*/
#include "vision/vision_node.hpp"

VisionNode::VisionNode() :
  nh_("vision"),
  it_(ros::NodeHandle("camera")),
  camera_sub_(
    it_.subscribeCamera(
      "segmented_image", 1, &VisionNode::CameraCallback, this)),
  ball_detector_(nh_) {
}

void VisionNode::Spin() {
  // Use rate twice the maximum fps which is 30
  ros::Rate rate(60);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}

void VisionNode::CameraCallback(
  const sensor_msgs::ImageConstPtr& image,
  const sensor_msgs::CameraInfoConstPtr& cam_info) {
  ball_detector_.ProcessImage(image, cam_info);
}
