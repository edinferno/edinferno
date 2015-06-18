/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-24
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: Implementation of the Camera class.
*/
#include "camera/camera.hpp"

#include <cstring>

#include <sensor_msgs/image_encodings.h>
/**
 * @brief Constructor
 *
 * @param nh The ROS node hande used by the camera
 * @param id ID of the camera - AL::kTopCamera or AL::kBottomCamera
 * @param name Name of camera e.g. "top"
 */
Camera::Camera(ros::NodeHandle& nh, int id, const char* name) :
  nh_(nh, name),
  id_(id),
  name_(name),
  cam_info_url_("file:///home/nao/config/camera/${NAME}.yaml"),
  frame_id_(name_ + "_camera") {
  Init();
}

/**
 * @brief Destructor
 */
Camera::~Camera() {
  delete cam_info_manager_;
}

/**
 * @brief Initialises the camera and reads the camera info.
 */
void Camera::Init() {
  cam_info_manager_ = new camera_info_manager::CameraInfoManager(nh_, name_);
  // Check if the calibration file exists
  if (!cam_info_manager_->loadCameraInfo(cam_info_url_)) {
    ROS_FATAL("Unable to load %s", cam_info_url_.c_str());
  }
}
