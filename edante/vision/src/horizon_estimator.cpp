/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-09
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The class estimates where the horizon line is in the image.
*/
#include "vision/horizon_estimator.hpp"

// OpenCV
#include <opencv2/core/core.hpp>

// Messages
#include <std_msgs/Int32.h>

using std::vector;
using cv::Point2d;
using cv::Point3d;

/**
 * @brief Initialises the horizon publisher.
 *
 * @param nh The ROS node handle which is used to advertise the publisher.
 */
HorizonEstimator::HorizonEstimator(ros::NodeHandle& nh) :
  horizon_pub_(nh.advertise<std_msgs::Int32>("horizon", 1)) {
}
/**
 * @brief Calculate the level of the horizon line in the image at a given
 *        distance.
 *
 * @param distance The distance at which the horizon level to be calculated
 * @param cam_model The calibrated camera model used for projecting back on
 *                  the image.
 * @param transform The camera frame transform when the image was captured.
 * @param head_yaw The head yaw angle when the image was captured.
 * @param horizon_level The calculated horizon level (image row) in the image.
 */
void HorizonEstimator::HorizonLevel(
  float distance,
  const image_geometry::PinholeCameraModel& cam_model,
  const std::vector<float>& transform,
  float head_yaw,
  int& horizon_level) {
  // Get the robot frame point (distance, 0, 0) into camera frame coordinates.
  Point3d robot_frame;
  robot_frame.x = cos(head_yaw) * distance;
  robot_frame.y = sin(head_yaw) * distance;
  robot_frame.z = 0;

  // Use the inverse transform i.e. from robot frame to camera frame
  Point3d cam_frame;
  cam_frame.x = transform[0] * robot_frame.x +
                transform[4] * robot_frame.y - transform[3];
  cam_frame.y = transform[1] * robot_frame.x +
                transform[5] * robot_frame.y - transform[7];
  cam_frame.z = transform[2] * robot_frame.x +
                transform[6] * robot_frame.y - transform[11];

  // Get the camera frame point into optical camera frame coordinates.
  Point3d optical_frame;
  optical_frame.x = -cam_frame.y;
  optical_frame.y = -cam_frame.z;
  optical_frame.z = cam_frame.x;

  // Project the point back onto the image
  Point2d image_frame = cam_model.project3dToPixel(optical_frame);
  image_frame = cam_model.unrectifyPoint(image_frame);

  // The horizon level is the row
  horizon_level = static_cast<int>(image_frame.y);

  // Publish the result if needed
  if (horizon_pub_.getNumSubscribers() > 0) {
    std_msgs::Int32 msg;
    msg.data = horizon_level;
    horizon_pub_.publish(msg);
  }
}
