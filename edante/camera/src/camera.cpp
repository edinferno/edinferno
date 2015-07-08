/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-24
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: Implementation of the Camera class.
*/
#include "camera/camera.hpp"

#include <cstring>

#include <alvision/alvisiondefinitions.h>

#include <sensor_msgs/image_encodings.h>

int Camera::fps_ = 30;

/**
 * @brief Constructor
 *
 * @param nh The ROS node hande used by the camera
 * @param name Name of camera e.g. "top"
 * @param id ID of the camera (AL::kTopCamera or AL::kBottomCamera)
 * @param resolution Resolution of the camera (AL::kQQVGA, AL::kQVGA,
 *                   AL::kVGA, AL::k4VGA)
 * @param color_space Color space of the camera (AL::kYUV422ColorSpace,
 *                    AL::kYUVColorSpace, AL::kRGBColorSpace,
 *                    AL::kHSYColorSpace, AL::kBGRColorSpace)
 */
Camera::Camera(ros::NodeHandle& nh,
               const char* name,
               const char* frame_name,
               int id,
               int resolution,
               int color_space) :
  nh_(nh, name),
  name_(name),
  frame_name_(frame_name),
  id_(id),
  resolution_(resolution),
  color_space_(color_space),
  cam_info_url_("file:///home/nao/config/camera/${NAME}.yaml") {
  Init();
}

/**
 * @brief Destructor
 */
Camera::~Camera() {
  delete cam_info_manager_;
}

void Camera::SetImage(const AL::ALImage* alimage, ros::Time stamp) {
  image_.header.stamp = stamp;
  cam_info_.header.stamp = stamp;
  memcpy(image_.data.data(), alimage->getData(), image_.data.size());
}

void Camera::SetGreyscaleImage(const AL::ALImage* alimage, ros::Time stamp) {
  const unsigned char* ptr = alimage->getData();
  switch (color_space_) {
    case AL::kYUV422ColorSpace:
      for (size_t i = 0; i < greyscale_image_.data.size(); ++i) {
        greyscale_image_.data[i] = *ptr;
        ptr += 2;
      }
      break;
    case AL::kYUVColorSpace:
      for (size_t i = 0; i < greyscale_image_.data.size(); ++i) {
        greyscale_image_.data[i] = *ptr;
        ptr += 3;
      }
      break;
    case AL::kRGBColorSpace:
      for (size_t i = 0; i < greyscale_image_.data.size(); ++i) {
        greyscale_image_.data[i] = * ptr      * 0.2990 +
                                   *(ptr + 1) * 0.5870 +
                                   *(ptr + 2) * 0.1140;
        ptr += 3;
      }
      break;
    case AL::kHSYColorSpace:
      ptr += 2;
      for (size_t i = 0; i < greyscale_image_.data.size(); ++i) {
        greyscale_image_.data[i] = *ptr;
        ptr += 3;
      }
      break;
    case AL::kBGRColorSpace:
      for (size_t i = 0; i < greyscale_image_.data.size(); ++i) {
        greyscale_image_.data[i] = *(ptr + 2) * 0.2990 +
                                   *(ptr + 1) * 0.5870 +
                                   * ptr      * 0.1140;
        ptr += 3;
      }
      break;
  }
}

void Camera::Update() {
  UpdateImage();
  UpdateCameraInfo();
}

void Camera::UpdateImage() {
  // Set the image frame_id
  image_.header.frame_id = frame_name_;

  // Set the image size
  switch (resolution_) {
    case AL::kQQVGA:
      image_.width = 160;
      image_.height = 120;
      break;
    case AL::kQVGA:
      image_.width = 320;
      image_.height = 240;
      break;
    case AL::kVGA:
      image_.width = 640;
      image_.height = 480;
      break;
    case AL::k4VGA:
      image_.width = 1280;
      image_.height = 960;
      break;
  }

  int pixel_size = 3;
  // Set the image encoding
  switch (color_space_) {
    case AL::kYUV422ColorSpace:
      image_.encoding = "yuv422";
      // Adjust pixel size
      pixel_size = 2;
      break;
    case AL::kYUVColorSpace:
      image_.encoding = "yuv8";
      break;
    case AL::kRGBColorSpace:
      image_.encoding = "rgb8";
      break;
    case AL::kHSYColorSpace:
      image_.encoding = "hsy8";
      break;
    case AL::kBGRColorSpace:
      image_.encoding = "bgr8";
      break;
  }

  // Allocate memory
  image_.is_bigendian = false;
  image_.step = image_.width * pixel_size;
  image_.data.resize(image_.height * image_.step);

  // Greyscale image
  greyscale_image_.header.frame_id = frame_name_;
  greyscale_image_.width = image_.width;
  greyscale_image_.height = image_.height;
  greyscale_image_.encoding = "mono8";
  greyscale_image_.is_bigendian = false;
  greyscale_image_.step = greyscale_image_.width;
  greyscale_image_.data.resize(greyscale_image_.height * greyscale_image_.step);
}

void Camera::UpdateCameraInfo() {
  cam_info_ = cam_info_manager_->getCameraInfo();
  cam_info_.header.frame_id = frame_name_;
  // Check if the stored camera info matches the current settings
  if (cam_info_.width != image_.width ||
      cam_info_.height != image_.height) {
    // The sizes do not match so generate and uncalibrated camera settings
    cam_info_ = sensor_msgs::CameraInfo();
    cam_info_.width = image_.width;
    cam_info_.height = image_.height;
  }
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
  Update();
}
