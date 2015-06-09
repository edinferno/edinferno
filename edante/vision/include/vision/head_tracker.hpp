/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-04
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: Implements head tracking of an object in the image.
*/
#ifndef HEAD_TRACKER_HPP
#define HEAD_TRACKER_HPP

#include <ros/ros.h>

#include <camera_msgs/SetActiveCamera.h>
#include <motion_msgs/ChangeAngles.h>
#include <vision_msgs/BallDetection.h>
#include <vision_msgs/StartHeadTracking.h>
#include <vision_msgs/StopHeadTracking.h>




class HeadTracker {
 public:
  enum TrackedObjectType {None = -1, Ball = 0};
  /**
   * @brief Initialises the services
   *
   * @param nh The node handle to be used
   */
  explicit HeadTracker(ros::NodeHandle& nh);
  /**
   * @brief This function moves the head toward the object of interest
   * @details The object of interest is selected based on the value of
   *          object_type_. Currently, only ball is supported.
   *
   * @param ball The detected ball by the ball detector.
   * @param head_pitch The current pitch angle of the head.
   */
  void Track(const vision_msgs::BallDetection& ball, float head_pitch);

 private:
  // Service servers
  ros::ServiceServer start_head_tracking_server_;
  ros::ServiceServer stop_head_tracking_server_;
  ros::ServiceClient set_active_camera_client_;
  ros::ServiceClient change_angles_client_;
  // Preallocated messages
  camera_msgs::SetActiveCamera set_active_camera_srv_;
  motion_msgs::ChangeAngles change_angles_srv_;

  // Whether tracking is enabled
  bool is_enabled_;
  // The type of object to track (only Ball is currently supported)
  TrackedObjectType object_type_;

  // Service callbacks
  bool start_head_tracking(vision_msgs::StartHeadTracking::Request&  req,
                           vision_msgs::StartHeadTracking::Response& res);
  bool stop_head_tracking(vision_msgs::StopHeadTracking::Request&  req,
                          vision_msgs::StopHeadTracking::Response& res);
};

#endif

