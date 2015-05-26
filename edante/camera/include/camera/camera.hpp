/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-24
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: This is the main class which represents a camera.
*             It provides functionality to start, stop and
*             calibrate the camera.
*/
#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <camera_info_manager/camera_info_manager.h>

class Camera {
 public:
  Camera(ros::NodeHandle& nh, int id, const char* name);
  ~Camera();

  int id() const { return id_; }
  std::string name() const { return name_; }
  std::string frame_id() const { return frame_id_; }

  sensor_msgs::CameraInfo cam_info() const { return cam_info_; }

 private:
  ros::NodeHandle nh_;

  const int id_;
  // AL::kTopCamera
  // AL::kBottomCamera

  std::string name_;
  std::string cam_info_url_;
  std::string frame_id_;

  camera_info_manager::CameraInfoManager* cam_info_manager_;
  sensor_msgs::CameraInfo cam_info_;

  void Init();

};
#endif
