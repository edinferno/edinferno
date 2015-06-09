/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-09
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The class estimates where the horizon line is in the image.
*/
#ifndef HORIZON_ESTIMATOR_HPP
#define HORIZON_ESTIMATOR_HPP

// System
#include <vector>

// ROS
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>

class HorizonEstimator {
 public:
  /**
   * @brief Initialises the horizon publisher.
   *
   * @param nh The ROS node handle which is used to advertise the publisher.
   */
  explicit HorizonEstimator(ros::NodeHandle& nh);
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
  void HorizonLevel(float distance,
                    const image_geometry::PinholeCameraModel& cam_model,
                    const std::vector<float>& transform,
                    float head_yaw,
                    int& horizon_level);

 private:
  ros::Publisher horizon_pub_;
};

#endif
