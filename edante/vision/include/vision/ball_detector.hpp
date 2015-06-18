/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-01
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The class finds the ball in a segmented image.
*/
#ifndef BALL_DETECTOR_HPP
#define BALL_DETECTOR_HPP

// System
#include <vector>

// OpenCV
#include <opencv2/core/core.hpp>

// ROS
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>

// Messages
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/BallDetection.h>

class BallDetector {
 public:
  /**
   * @brief Creates the BallDetection publisher and the GetTransform client.
   *
   * @param nh The node handle to be used.
   */
  explicit BallDetector(ros::NodeHandle& nh);
  /**
   * @brief Detect the ball in the image.
   *
   * @param image The image to be processed.
   * @param header The header which will be used for the published
   *               ball detection.
   * @param cam_model The calibrated camera model.
   * @param transform The camera frame transformation at frame capture
   */
  void ProcessImage(const cv::Mat& image,
                    const std_msgs::Header& header,
                    const image_geometry::PinholeCameraModel& cam_model,
                    const std::vector<float>& transform);
  /**
   * @brief Getter of the current ball detection
   */
  const vision_msgs::BallDetection& ball() const { return ball_detection_; }

 private:
  // Ball radius in meters
  static const double kBallRadius;

  // The processed image
  cv::Mat image_;
  // Detected contours
  std::vector< std::vector<cv::Point> > contours_;
  // Information about the detected ball
  vision_msgs::BallDetection ball_detection_;
  // Publisher of the detected ball
  ros::Publisher ball_detection_pub_;

  /**
   * @brief Threshold the image such that only 'Ball' pixels are white.
   *
   * @param image The image to be thresholded.
   */
  void ThresholdImage(cv::Mat& image);
  /**
   * @brief Finds a ball in a thresholded image.
   * @details Uses cv::findcontours to detect objects. Filters them based on
   *          on size, circumference length and circularity. If none of the
   *          contours meets the requirements then the ball is not found.
   *          If multiple balls are present, the larger one is selected.
   *
   * @param image The image to be processed. It should be thresholded such that
   *              only 'Ball' pixels are white.
   * @param ball The results of the analysis are stored here including 2D ball
   *             position and radius if the ball is found.
   */
  void FindBall(cv::Mat& image, vision_msgs::BallDetection& ball);
  /**
   * @brief Estimate the 3D ball position based on the known size of the ball.
   *
   * @param cam_model The calibrated camera model.
   * @param transform The camera frame transformation at the time of capturing
   *                  the frame.
   * @param ball The estimated 3D position will be stored here.
   */
  void EstimateBallPos3D(const image_geometry::PinholeCameraModel& cam_model,
                         const std::vector<float>& transform,
                         vision_msgs::BallDetection& ball);
};
#endif
