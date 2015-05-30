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

// Messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// Services
#include "camera/SetActiveCamera.h"
#include "camera/SetColorSpace.h"
#include "camera/SetFrameRate.h"
#include "camera/SetResolution.h"
#include "camera/SetColorTable.h"
#include "camera/GetColorTable.h"

#include "camera/camera.hpp"
// Remember to change the enum in the camera_color_calibration.hpp
enum PixelClass {
  Nothing = 0,
  Ball = 255,
  GoalAndLines = 192,
  Field = 32,
  TeamRed = 96,
  TeamBlue = 128
};

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
  image_transport::CameraPublisher segmented_image_pub_;
  image_transport::Publisher segmented_rgb_image_pub_;

  // Message structs
  sensor_msgs::Image image_;
  sensor_msgs::Image segmented_image_;
  sensor_msgs::Image segmented_rgb_image_;

  // NaoQi related members
  AL::ALVideoDeviceProxy* camera_proxy_;

  // Camera instances
  Camera* top_cam_;
  Camera* bot_cam_;

  // Active settings
  const Camera* active_cam_;

  // AL::kQQVGA  160*120px
  // AL::kQVGA   320*240px
  // AL::kVGA    640*480px
  // AL::k4VGA   1280*960px
  int active_resolution_;

  // AL::kYUV422ColorSpace  0xY’Y’VVYYUU - native format (2 pixels)
  // AL::kYUVColorSpace     0xVVUUYY
  // AL::kRGBColorSpace     0xBBGGRR
  // AL::kHSYColorSpace     0xYYSSHH
  // AL::kBGRColorSpace     0xRRGGBB
  int active_color_space_;

  // 1 - 30 FPS
  int active_fps_;

  ros::Rate* active_rate_;

  sensor_msgs::CameraInfo active_cam_info_;

  // ROS service servers
  ros::ServiceServer set_active_camera_server_;
  ros::ServiceServer set_resolution_server_;
  ros::ServiceServer set_frame_rate_server_;
  ros::ServiceServer set_color_space_server_;
  ros::ServiceServer set_color_table_server_;
  ros::ServiceServer get_color_table_server_;

  // Color table
  static const char* table_file_name_;
  static const size_t kTableSize = 64;
  static const size_t kTableLen = kTableSize * kTableSize * kTableSize;
  PixelClass table_[kTableSize][kTableSize][kTableSize];


  void Init();
  void LoadColorTable();
  void Spin();

  void SegmentImage(const sensor_msgs::Image& raw, sensor_msgs::Image& seg);
  void ColorSegmentedImage(const sensor_msgs::Image& seg,
                           sensor_msgs::Image& rgb);

  void Update();
  void UpdateImage();
  void UpdateCameraInfo();

  // Service callbacks
  bool set_active_camera(camera::SetActiveCamera::Request&  req,
                         camera::SetActiveCamera::Response& res);
  bool set_resolution(camera::SetResolution::Request&  req,
                      camera::SetResolution::Response& res);
  bool set_frame_rate(camera::SetFrameRate::Request&  req,
                      camera::SetFrameRate::Response& res);
  bool set_color_space(camera::SetColorSpace::Request&  req,
                       camera::SetColorSpace::Response& res);
  bool set_color_table(camera::SetColorTable::Request&  req,
                       camera::SetColorTable::Response& res);
  bool get_color_table(camera::GetColorTable::Request&  req,
                       camera::GetColorTable::Response& res);
};

#endif
