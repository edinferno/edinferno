/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-24
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: Implementation of the Camera class.
*/
#include "camera/camera.hpp"
#include <cstring>


#include <sensor_msgs/image_encodings.h>

Camera::Camera(ros::NodeHandle& nh, int id, const char* name) :
  nh_(nh, name),
  id_(id),
  name_(name),
  cam_info_url_("file:///home/nao/config/camera/${NAME}.yaml"),
  frame_id_(name_ + "_camera") {
  Init();
}

Camera::~Camera() {
  // TODO: Figure out when NaoQi calls _closeModule
  // delete cam_info_manager_;
}

void Camera::Init() {
  cam_info_manager_ = new camera_info_manager::CameraInfoManager(nh_, name_);

  if (!cam_info_manager_->loadCameraInfo(cam_info_url_)) {
    ROS_FATAL("Unable to load %s", cam_info_url_.c_str());
  }
  cam_info_ = cam_info_manager_->getCameraInfo();
}
