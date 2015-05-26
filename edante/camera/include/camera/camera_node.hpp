/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-25
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: The node which exposes the Nao cameras to ROS.
*/

#ifndef CAMERA_MODULE_HPP
#define CAMERA_MODULE_HPP

#include <memory>
#include <string>

#include <alcommon/almodule.h>
#include <alproxies/alvideodeviceproxy.h>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>

#include "camera/camera.hpp"

class CameraNode : public AL::ALModule {
 public:
  CameraNode(boost::shared_ptr<AL::ALBroker> broker,
             const std::string& module_name);
  ~CameraNode();

 private:
  std::string module_name_;

  // ROS related members
  ros::NodeHandle* nh_;
  image_transport::ImageTransport* it_;
  image_transport::CameraPublisher image_pub_;

  // Message structs
  sensor_msgs::Image image_;

  // NaoQi related members
  AL::ALVideoDeviceProxy* camera_proxy_;

  // Camera instances
  Camera* top_cam_;
  Camera* bot_cam_;

  // Active settings
  const Camera* active_cam_;

  int active_resolution_;
  // AL::kQQVGA  160*120px
  // AL::kQVGA   320*240px
  // AL::kVGA    640*480px
  // AL::k4VGA   1280*960px

  int active_color_space_;
  // AL::kYUV422ColorSpace  0xY’Y’VVYYUU - native format (2 pixels)
  // AL::kYUVColorSpace     0xVVUUYY
  // AL::kRGBColorSpace     0xBBGGRR
  // AL::kHSYColorSpace     0xYYSSHH
  // AL::kBGRColorSpace     0xRRGGBB

  int active_fps_;
  // 1 - 30 FPS
  ros::Rate* active_rate_;

  void Init();
  void Spin();

  void UpdateSensorMsgImage();
};

#endif
