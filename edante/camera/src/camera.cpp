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
  frame_id_(name_ + "camera"),
  cap_(new cv::VideoCapture(id_)),
  cam_info_manager_(new camera_info_manager::CameraInfoManager(nh_, name_)),
  it_(new image_transport::ImageTransport(nh_)),
  image_pub_(it_->advertiseCamera("image_raw", 1)) {
}

bool Camera::Init() {
  if (!cap_-> isOpened()) {
    ROS_FATAL("Unable to open camera with ID:%d", id_);
    return false;
  }

  if (!cam_info_manager_->loadCameraInfo(cam_info_url_)) {
    ROS_FATAL("Unable to load %s", cam_info_url_.c_str());
    return false;
  }
  cam_info_ = cam_info_manager_->getCameraInfo();

  cap_->set(CV_CAP_PROP_FRAME_HEIGHT, cam_info_.height);
  cap_->set(CV_CAP_PROP_FRAME_WIDTH, cam_info_.width);

  image_.header.frame_id = frame_id_;
  image_.height = cam_info_.height;
  image_.width = cam_info_.width;
  image_.encoding = sensor_msgs::image_encodings::BGR8;
  image_.is_bigendian = false;
  image_.step = image_.width * 3;
  image_.data.resize(image_.height * image_.step);

  return true;
}

void Camera::SpinOnce() {
  Read();
  Publish();
}

void Camera::Publish() {
  cam_info_.header.frame_id = frame_id_;
  cam_info_.header.stamp = image_.header.stamp;

  image_pub_.publish(image_, cam_info_);
}

void Camera::Read() {
  image_.header.stamp = ros::Time::now();
  cap_->read(mat_);
  memcpy(image_.data.data(), mat_.data, image_.height * image_.step);
}
