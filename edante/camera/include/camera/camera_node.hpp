/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-25
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: The node which exposes the Nao cameras to ROS.
*/

#ifndef CAMERA_MODULE_HPP
#define CAMERA_MODULE_HPP

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <boost/shared_ptr.hpp>

#include <alcommon/almodule.h>
#include <alproxies/alvideodeviceproxy.h>

class CameraNode : public AL::ALModule {
 public:
  CameraNode(boost::shared_ptr<AL::ALBroker> broker,
             const std::string& name);
  ~CameraNode();
 private:
  std::string name_;

  ros::NodeHandle* nh_;

  AL::ALVideoDeviceProxy* camera_proxy_;

  ros::Publisher image_pub_;
  sensor_msgs::Image image_;


  void Spin();
  void Init();
};

#endif
