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

/**
 * @brief Represents a physical camera device
 * @details The class represents a physical camera device - top or bottom
 *          camera of the Nao. It provides set_camera_info service in order
 *          to be compatible with the ROS image toolchain.
 */
class Camera {
 public:
  /**
   * @brief Constructor
   *
   * @param nh The ROS node hande used by the camera
   * @param id ID of the camera (AL::kTopCamera or AL::kBottomCamera)
   * @param name Name of camera e.g. "top"
   */
  Camera(ros::NodeHandle& nh, int id, const char* name);
  /**
   * @brief Destructor
   */
  ~Camera();
  /**
   * @brief Getter of the camera ID.
   */
  int id() const { return id_; }
  /**
   * @brief Getter of the camera name.
   */
  std::string name() const { return name_; }
  /**
   * @brief Getter of the camera frame_id.
   */
  std::string frame_id() const { return frame_id_; }
  /**
   * @brief Getter of the camera info.
   */
  sensor_msgs::CameraInfo cam_info() const {
    return cam_info_manager_->getCameraInfo();
  }

 private:
  // Node Handle
  ros::NodeHandle nh_;

  // AL::kTopCamera
  // AL::kBottomCamera
  const int id_;

  // Camera settings
  std::string name_;
  std::string cam_info_url_;
  std::string frame_id_;

  // ROS image toolchain
  camera_info_manager::CameraInfoManager* cam_info_manager_;

  /**
   * @brief Initialises the camera and reads the camera info.
   */
  void Init();
};
#endif
