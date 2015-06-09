/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-25
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: The node which exposes the Nao cameras to ROS.
*/

#ifndef CAMERA_MODULE_HPP
#define CAMERA_MODULE_HPP

// System
#include <string>
#include <vector>

// NaoQi
#include <alcommon/almodule.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alproxies/almotionproxy.h>

// Boost
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>

// Messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// Services
#include <camera_msgs/SetActiveCamera.h>
#include <camera_msgs/SetColorSpace.h>
#include <camera_msgs/SetFrameRate.h>
#include <camera_msgs/SetResolution.h>
#include <camera_msgs/SetColorTable.h>
#include <camera_msgs/GetColorTable.h>

// Local
#include "camera/camera.hpp"
#include "camera/color_table.hpp"


/**
 * @brief Exposes the Nao camera to ROS using the standard tool of
 *        the ROS image toolchain. It supports color table segmentation
 *        and puts the segmented image in the shared memroy for locally
 *        running nodes.
 */
class CameraNode : public AL::ALModule {
 public:
  /**
   * @brief Constructor
   * @details Initialises and starts the camera node.
   *
   * @param broker Broker used for creating the module.
   * @param module_name The name of the module.
   */
  CameraNode(boost::shared_ptr<AL::ALBroker> broker,
             const std::string& module_name);
  /**
   * @brief Destructor
   * @details Stops the module thread and releases memory.
   */
  ~CameraNode();

 private:
  // Module related
  std::string module_name_;
  boost::thread* module_thread_;
  bool is_module_closing_;

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
  AL::ALMotionProxy* motion_proxy_;

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

  // Camera calibration info
  sensor_msgs::CameraInfo active_cam_info_;

  // Camera frame name
  std::string active_camera_frame_name_;

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

  // Shared memory
  static const size_t kShdMemSize = 1280 * 960 + 128;  // k4VGA + misc
  boost::interprocess::shared_memory_object shdmem_;
  boost::interprocess::mapped_region* shdmem_region_;
  uint8_t* shdmem_ptr_;
  boost::interprocess::named_mutex* shdmem_mtx_;

  /**
  * @brief Initialised the module
  * @details The function initialises all components of the modue.
  *          It initialises ROS and advertises the supported services and
  *          topics. Allocates memory for the current images and eventually
  *          starts the spinning thread.
  */
  void Init();
  /**
  * @brief Loads the color table from a file.
  * @details Loads the color table from a file into the table array.
  *          The location of the file is stored in table_file_name_.
  *          Currently it is set to /home/nao/config/camera/table.c64
  */
  void LoadColorTable();
  /**
  * @brief Worker function of the module
  * @details The function reads images from the camera, applies segmentation
  *          and then publishes them over ROS. It runs on a separate thread.
  */
  void Spin();
  /**
  * @brief Write the captured data to shared memory
  * @details Copy the captured image, camera info, camera frame transformation
  *          and head angles to shared memory in orderto optimize access time
  *          for locally running nodes.
  *
  * @param image The image to be copied to memory
  * @param cam_info The camera info to be copied to memory
  * @param transform The camera frame transformation to be copied to memory
  * @param transform The head angles to be copied to memory
  */
  void WriteToSharedMemory(const sensor_msgs::Image& image,
                           const sensor_msgs::CameraInfo& cam_info,
                           const std::vector<float>& transform,
                           const std::vector<float>& head_angles);
  /**
  * @brief Segments the image using the color look up table
  * @details The 3 color channels of a pixel are used as indecies to the color
  *          lookup table in order to classify them.
  * @param raw A YUV442 encoded input image.
  * @param seg The segmented image in MONO8 encoding. The values are determined
  *            by the PixelClass enumeration.
  */
  void SegmentImage(const sensor_msgs::Image& raw, sensor_msgs::Image& seg);
  /**
  * @brief Color the segmented image for visualisation only.
  * @details Each pixel is colored according to its class.
  *
  * @param seg The input segmented image. It should be in MONO8 encoding.
  * @param rgb The output color image.
  */
  void ColorSegmentedImage(const sensor_msgs::Image& seg,
                           sensor_msgs::Image& rgb);
  /**
  * @brief Updates the manually allocated images and camera_info.
  */
  void Update();
  /**
  * @brief Updates the allocated images to reflect the currently active camera settings.
  */
  void UpdateImage();
  /**
  * @brief Updates the currently active camera_info to reflect the active camera settings.
  */
  void UpdateCameraInfo();

  // Service callbacks
  /**
  * @brief Set the currently active camera (top or bottom).
  *
  * @param req Service request.
  * @param res Service response.
  *
  * @return Return true on successful completion.
  */
  bool set_active_camera(camera_msgs::SetActiveCamera::Request&  req,
                         camera_msgs::SetActiveCamera::Response& res);
  /**
  * @brief Set the currently active resolution.
  *
  * @param req Service request.
  * @param res Service response.
  *
  * @return Return true on successful completion.
  */
  bool set_resolution(camera_msgs::SetResolution::Request&  req,
                      camera_msgs::SetResolution::Response& res);
  /**
  * @brief Set the currently active frame rate.
  *
  * @param req Service request.
  * @param res Service response.
  *
  * @return Return true on successful completion.
  */
  bool set_frame_rate(camera_msgs::SetFrameRate::Request&  req,
                      camera_msgs::SetFrameRate::Response& res);
  /**
  * @brief Set the color space in which images are captured.
  *
  * @param req Service request.
  * @param res Service response.
  *
  * @return Return true on successful completion.
  */
  bool set_color_space(camera_msgs::SetColorSpace::Request&  req,
                       camera_msgs::SetColorSpace::Response& res);
  /**
  * @brief Set the color table to be used.
  * @details The service receives a serialised color table and stores
  *          it to a file. The filename is stored in table_file_name_
  *          and currently is /home/nao/config/camera/table.c64
  *          Once the file is created, the table is loaded from there.
  *
  * @param req Service request.
  * @param res Service response.
  *
  * @return Return true on successful completion.
  */
  bool set_color_table(camera_msgs::SetColorTable::Request&  req,
                       camera_msgs::SetColorTable::Response& res);
  /**
  * @brief Returns the currently used color table.
  * @details The current color table is serialised and sent back.
  *
  * @param req Service request.
  * @param res Service response.
  *
  * @return Return true on successful completion.
  */
  bool get_color_table(camera_msgs::GetColorTable::Request&  req,
                       camera_msgs::GetColorTable::Response& res);
};

#endif
