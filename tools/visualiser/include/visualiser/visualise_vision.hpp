/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-06
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: Overalay vision related information on each frame and publish it.
*/
#ifndef VISUALISE_VISION_HPP
#define VISUALISE_VISION_HPP

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <vision_msgs/BallDetection.h>
#include <vision_msgs/LineDetections.h>

class VisualiseVision {
 public:
  explicit VisualiseVision(ros::NodeHandle& nh);
  void Spin();
 private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher image_pub_;
  ros::Subscriber ball_sub_;
  ros::Subscriber lines_sub_;

  sensor_msgs::Image image_;
  vision_msgs::BallDetection ball_;
  vision_msgs::LineDetections lines_;

  cv::Mat mat_;

  void ImageCallback(const sensor_msgs::ImageConstPtr& image);
  void BallCallback(const vision_msgs::BallDetectionConstPtr& ball);
  void LinesCallback(const vision_msgs::LineDetectionsConstPtr& lines);
  void YUVImageToBGRMat(const sensor_msgs::Image& image, cv::Mat& mat);
  void YUVtoRGB(uint8_t y, uint8_t u, uint8_t v,
                uint8_t& r, uint8_t& g, uint8_t& b);
  void PublishVisualisation(const std_msgs::Header& header, const cv::Mat& mat);

  void DrawBall(const vision_msgs::BallDetection& ball, cv::Mat& mat);
  void DrawLines(const vision_msgs::LineDetections& lines, cv::Mat& mat);
};

#endif
