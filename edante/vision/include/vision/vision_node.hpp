/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-01
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The vision subscribes to the segmented image and
*             executes all the visual processing needed.
*/
#ifndef VISION_NODE_HPP
#define VISION_NODE_HPP

// System
#include <vector>

// Boost
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

// ROS
#include <ros/ros.h>

// Messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "vision/ball_detector.hpp"
#include "vision/line_detector.hpp"
#include "vision/head_tracker.hpp"

/**
 * @brief Reads the segmented image and the corresponding camera infro
 *        from the shared memory and runs all the visual processing needed.
 */
class VisionNode {
 public:
  /**
   * @brief Initialises the vision node.
   * @details Creates the underlying NodeHandle and allocates the shared
   *          memory segment used for communication with local nodes.
   */
  VisionNode();
  /**
   * @brief Checks for a new frame and processes it.
   */
  void Spin();

 private:
  ros::NodeHandle nh_;

  // Shared memory
  static const size_t kShdMemSize = 1280 * 960 + 128;  // k4VGA + misc
  boost::interprocess::shared_memory_object shdmem_;
  boost::interprocess::mapped_region* shdmem_region_;
  uint8_t* shdmem_ptr_;
  boost::interprocess::named_mutex shdmem_mtx_;

  // Detects the ball and publishes information
  BallDetector ball_detector_;

  // Detectes lines and publishes information
  LineDetector line_detector_;

  // Head tracking
  HeadTracker head_tracker_;

  /**
   * @brief Retrieves the camera data stored in the shared memory.
   * @details Retreives the current camera info and image stored
   *          in the shared memory segment by the camera node.
   *
   * @param image The retreived image.
   * @param cam_info The retreived camera_info.
   * @param transform The camera frame transform during image capture
   *
   * @return [description]
   */
  bool SharedMemoryToCamera(sensor_msgs::Image& image,
                            sensor_msgs::CameraInfo& cam_info,
                            std::vector<float>& transform);
};

#endif
