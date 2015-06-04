/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-04
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: Implements head tracking of an object in the image.
*/
#include "vision/head_tracker.hpp"

/**
 * @brief Initialises the services
 *
 * @param nh The node handle to be used
 */
HeadTracker::HeadTracker(ros::NodeHandle& nh) :
  start_head_tracking_server_(
    nh.advertiseService("start_head_tracking",
                        &HeadTracker::start_head_tracking,
                        this)),
  stop_head_tracking_server_(
    nh.advertiseService("stop_head_tracking",
                        &HeadTracker::stop_head_tracking,
                        this)),
  change_angles_client_(
    nh.serviceClient<motion_msgs::ChangeAngles>("/motion/change_angles",
                                                true)),
  is_enabled_(false) {
  // Preallocate the change angles message
  change_angles_srv_.request.names.push_back("HeadYaw");
  change_angles_srv_.request.names.push_back("HeadPitch");
  change_angles_srv_.request.changes.push_back(0.0f);
  change_angles_srv_.request.changes.push_back(0.0f);
  change_angles_srv_.request.fraction_max_speed = 0.8f;
}
/**
 * @brief This function moves the head toward the object of interest
 * @details The object of interest is selected based on the value of
 *          object_type_. Currently, only ball is supported.
 *
 * @param ball The detected ball by the ball detector.
 */
void HeadTracker::Track(const vision_msgs::BallDetection& ball) {
  if (!is_enabled_) return;

  // Check where should the head look
  geometry_msgs::Point32 track_pos;
  switch (object_type_) {
    case Ball: {
      // If tracking the ball, but it is not visible - do nothing.
      if (!ball.is_detected) return;
      track_pos = ball.pos_camera;
      break;
    }
    default:
      return;
  }
  // Calculate the angles to the ball in the camera frame.
  float yaw = atan2(track_pos.y, track_pos.x);
  float pitch = atan2(track_pos.z, track_pos.x);
  // Apply proportional control. Those coefficiants lead to
  // slighly underdamped behaviour, but provide fast reaction.
  change_angles_srv_.request.changes[0] = 0.20 * yaw;
  change_angles_srv_.request.changes[1] = -0.20 * pitch;
  // Send the command to motion
  change_angles_client_.call(change_angles_srv_);
}
bool HeadTracker::start_head_tracking(
  vision_msgs::StartHeadTracking::Request& req,
  vision_msgs::StartHeadTracking::Response& res) {
  is_enabled_ = true;
  object_type_ = static_cast<TrackedObjectType>(req.object_type);
  res.result = true;
  return true;
}

bool HeadTracker::stop_head_tracking(
  vision_msgs::StopHeadTracking::Request& req,
  vision_msgs::StopHeadTracking::Response& res) {
  if (object_type_ == static_cast<TrackedObjectType>(req.object_type)) {
    is_enabled_ = false;
    object_type_ = None;
    res.result = true;
  } else {
    res.result = false;
  }
  return true;
}

