/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-01
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The vision reads the segmented image from the shared
*             memory and executes all the visual processing needed.
*/
#include "vision/vision_node.hpp"

// ROS
#include <image_geometry/pinhole_camera_model.h>

// OpenCV
#include <opencv2/core/core.hpp>

using std::vector;

using boost::interprocess::open_only;
using boost::interprocess::read_only;
using boost::interprocess::mapped_region;
using boost::interprocess::named_mutex;

using cv::Mat;

using ros::serialization::IStream;
using ros::serialization::Serializer;

/**
 * @brief Initialises the vision node.
 * @details Creates the underlying NodeHandle and allocates the shared
 *          memory segment used for communication with local nodes.
 */
VisionNode::VisionNode() :
  nh_("vision"),
  shdmem_(open_only, "active_camera", read_only),
  shdmem_mtx_(open_only, "active_camera"),
  horizon_estimator_(nh_),
  ball_detector_(nh_),
  line_detector_(nh_),
  head_tracker_(nh_) {
  shdmem_region_ = new mapped_region(shdmem_, read_only);
  shdmem_ptr_ = static_cast<uint8_t*>(shdmem_region_->get_address());
}
/**
 * @brief Checks for a new frame and processes it.
 */
void VisionNode::Spin() {
  sensor_msgs::Image image;
  sensor_msgs::CameraInfo cam_info;
  image_geometry::PinholeCameraModel cam_model;
  vector<float> transform(16);
  vector<float> head_angles(2);  // yaw, pitch
  int horizon_level;

  // Use rate as the maximum fps which is 30
  ros::Rate rate(30);
  while (ros::ok()) {
    if (ReadFromSharedMemory(image, cam_info, transform, head_angles)) {

      // Update the camera model
      cam_model.fromCameraInfo(cam_info);

      // Calculate the horizon level
      horizon_estimator_.HorizonLevel(5,  // 5m
                                      cam_model,
                                      transform,
                                      head_angles[0],  // yaw
                                      horizon_level);

      // Get a shared cv::Mat from the image message
      Mat mat = Mat(image.height, image.width, CV_8UC1, image.data.data());
      ball_detector_.ProcessImage(mat, horizon_level, image.header, cam_model,
                                  transform);
      head_tracker_.Track(ball_detector_.ball(), head_angles[1]);

      // Disable line tracking for now
      // line_detector_.ProcessImage(mat, cam_mode);
    }
    ros::spinOnce();
    rate.sleep();
  }
}
/**
 * @brief Retrieves the camera data stored in the shared memory.
 * @details Retreives the current image, camera info, camera frame
 *          transform, and head angles stored in the shared memory
 *          segment by the camera node.
 *
 * @param image The retreived image.
 * @param cam_info The retreived camera_info.
 * @param transform The retreived camera frame transform.
 * @param head_angles The retreived head angles.
 *
 * @return [description]
 */
bool VisionNode::ReadFromSharedMemory(sensor_msgs::Image& image,
                                      sensor_msgs::CameraInfo& cam_info,
                                      std::vector<float>& transform,
                                      std::vector<float>& head_angles) {
  if (!shdmem_mtx_.try_lock()) return false;
  uint8_t* ptr = shdmem_ptr_;

  uint32_t cam_info_size;
  memcpy(&cam_info_size, ptr, sizeof(cam_info_size));
  ptr += sizeof(cam_info_size);

  IStream cam_info_stream(ptr, cam_info_size);
  Serializer<sensor_msgs::CameraInfo>::read(cam_info_stream, cam_info);
  ptr += cam_info_size;

  uint32_t image_size;
  memcpy(&image_size, ptr, sizeof(image_size));
  ptr += sizeof(image_size);

  IStream image_stream(ptr, image_size);
  Serializer<sensor_msgs::Image>::read(image_stream, image);
  ptr += image_size;

  memcpy(transform.data(), ptr, sizeof(float) * transform.size());
  ptr += sizeof(float) * transform.size();

  memcpy(head_angles.data(), ptr, sizeof(float) * head_angles.size());
  shdmem_mtx_.unlock();

  return true;
}
