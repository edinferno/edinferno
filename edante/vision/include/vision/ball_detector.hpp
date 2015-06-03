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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
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
   * @param cam_info The camera calibration information.
   */
  void ProcessImage(const sensor_msgs::Image& image,
                    const sensor_msgs::CameraInfo& cam_info);

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
  // Camera model used for estimating 3D ball position
  image_geometry::PinholeCameraModel cam_model_;
  // Persistent client used to obtain the camera to robot frame transformation
  ros::ServiceClient transform_client_;

  /**
   * @brief Makes a cv::Mat object from an image by copying memory.
   *
   * @param image The input image
   * @param mat The output cv::Mat
   */
  void GetMat(const sensor_msgs::Image& image, cv::Mat& mat);
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
   * @param cam_info Calibration information for the camera which captured
   *                 the image.
   * @param ball The estimated 3D position will be stored here.
   */
  void EstimateBallPos3D(const sensor_msgs::CameraInfo& cam_info,
                         vision_msgs::BallDetection& ball);
};
#endif
