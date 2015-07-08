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

#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <camera_info_manager/camera_info_manager.h>

/**
 * @brief Represents a physical camera device
 * @details The class represents a physical camera device - top or bottom
 *          camera of the Nao. It provides set_camera_info service in order
 *          to be compatible with the ROS image toolchain.
 */
class Camera {
 public:
  static int fps() { return fps_; }
  static void fps(int fps) { fps_ = fps; }

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
  Camera(ros::NodeHandle& nh,
         const char* name,
         const char* frame_name,
         int id,
         int resolution,
         int color_space);
  /**
   * @brief Destructor
   */
  ~Camera();

  void SetImage(const AL::ALImage* alimage, ros::Time stamp);
  void SetGreyscaleImage(const AL::ALImage* alimage, ros::Time stamp);

  void Update();
  void UpdateImage();
  void UpdateCameraInfo();

  /**
   * @brief Getter of the camera name.
   */
  const std::string& name() const { return name_; }
  /**
   * @brief Getter of the camera frame name
   */
  const std::string& frame_name() const { return frame_name_; }
  /**
   * @brief Getter of the camera ID.
   */
  int id() const { return id_; }
  /**
   * @brief Getter of the camera resolution.
   */
  int resolution() const { return resolution_; }
  /**
   * @brief Getter of the camera color_space.
   */
  int color_space() const { return color_space_; }
  /**
   * @brief Getter of the camera info.
   */
  const sensor_msgs::CameraInfo& cam_info() const { return cam_info_; }

  const sensor_msgs::Image& image() const {
    return image_;
  }

  const sensor_msgs::Image& greyscale_image() const {
    return greyscale_image_;
  }

 private:
  static int fps_;

  // Node Handle
  ros::NodeHandle nh_;

  // Camera name
  std::string name_;

  // Camera sensor frame name
  std::string frame_name_;

  // AL::kTopCamera
  // AL::kBottomCamera
  const int id_;

  // AL::kQQVGA  160*120px
  // AL::kQVGA   320*240px
  // AL::kVGA    640*480px
  // AL::k4VGA   1280*960px
  int resolution_;

  // AL::kYUV422ColorSpace  0xY’Y’VVYYUU - native format (2 pixels)
  // AL::kYUVColorSpace     0xVVUUYY
  // AL::kRGBColorSpace     0xBBGGRR
  // AL::kHSYColorSpace     0xYYSSHH
  // AL::kBGRColorSpace     0xRRGGBB
  int color_space_;

  // Camera settings
  std::string cam_info_url_;

  // Current camera info
  sensor_msgs::CameraInfo cam_info_;

  // Current camera image
  sensor_msgs::Image image_;

  // Greyscale image
  sensor_msgs::Image greyscale_image_;

  // Manager of the camera info (used for camera calibration)
  camera_info_manager::CameraInfoManager* cam_info_manager_;

  /**
   * @brief Initialises the camera and reads the camera info.
   */
  void Init();
};
#endif
