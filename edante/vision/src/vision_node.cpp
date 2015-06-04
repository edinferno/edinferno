/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-01
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The vision reads the segmented image from the shared
*             memory and executes all the visual processing needed.
*/
#include "vision/vision_node.hpp"

using boost::interprocess::open_only;
using boost::interprocess::read_only;
using boost::interprocess::mapped_region;
using boost::interprocess::named_mutex;

using ros::serialization::IStream;
using ros::serialization::Serializer;

/**
 * @brief Initialises the vision node.
 * @details Creates the underlying NodeHandle and allocates the shared
 *          memory segment used for communication with local nodes.
 */
VisionNode::VisionNode() :
  nh_("vision"),
  shdmem_(open_only, "camera_image", read_only),
  shdmem_mtx_(open_only, "camera_image_mutex"),
  ball_detector_(nh_),
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
  // Use rate as the maximum fps which is 30
  ros::Rate rate(30);
  while (ros::ok()) {
    if (SharedMemoryToCamera(image, cam_info)) {
      ball_detector_.ProcessImage(image, cam_info);
      head_tracker_.Track(ball_detector_.ball());
    }
    ros::spinOnce();
    rate.sleep();
  }
}
/**
 * @brief Retrieves the camera data stored in the shared memory.
 * @details Retreives the current camera info and image stored
 *          in the shared memory segment by the camera node.
 *
 * @param image The retreived image.
 * @param cam_info The retreived camera_info.
 *
 * @return [description]
 */
bool VisionNode::SharedMemoryToCamera(sensor_msgs::Image& image,
                                      sensor_msgs::CameraInfo& cam_info) {
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
  shdmem_mtx_.unlock();

  return true;
}
