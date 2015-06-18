/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-06
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: Add file description...
*/
#include "visualiser/visualise_vision.hpp"

using cv::Point;
using cv::Scalar;

VisualiseVision::VisualiseVision(ros::NodeHandle& nh) :
  it_(image_transport::ImageTransport(nh)),
  image_sub_(
    it_.subscribe("/camera/image", 1,
                  &VisualiseVision::ImageCallback, this)),
  image_pub_(
    nh.advertise<sensor_msgs::Image>("vision", 1)),
  horizon_sub_(
    nh.subscribe("/vision/horizon", 1,
                 &VisualiseVision::HorizonCallback, this)),
  ball_sub_(
    nh.subscribe("/vision/ball", 1,
                 &VisualiseVision::BallCallback, this)),
  lines_sub_(
    nh.subscribe("/vision/lines", 1,
                 &VisualiseVision::LinesCallback, this)),
  horizon_(-1) {
}

void VisualiseVision::Spin() {
  YUVImageToBGRMat(image_, mat_);
  DrawHorizon(horizon_, mat_);
  DrawBall(ball_, mat_);
  DrawLines(lines_, mat_);
  PublishVisualisation(image_.header, mat_);
}

void VisualiseVision::ImageCallback(
  const sensor_msgs::ImageConstPtr& image) {
  image_ = *image;
}
void VisualiseVision::HorizonCallback(
  const std_msgs::Int32ConstPtr& horizon) {
  horizon_ = horizon->data;
}
void VisualiseVision::BallCallback(
  const vision_msgs::BallDetectionConstPtr& ball) {
  ball_ = *ball;
}
void VisualiseVision::LinesCallback(
  const vision_msgs::LineDetectionsConstPtr& lines) {
  lines_ = *lines;
}

void VisualiseVision::YUVImageToBGRMat(const sensor_msgs::Image& image,
                                       cv::Mat& mat) {
  if (mat.cols != image.width || mat.rows != image.height) {
    mat = cv::Mat(image.height, image.width, CV_8UC3);
  }
  size_t num_pixels = image.width * image.height * 3;
  for (size_t i = 0, j = 0; i < num_pixels; i += 6, j += 4) {
    YUVtoRGB(image.data[j + 0], image.data[j + 1], image.data[j + 3],
             mat.data[i + 2], mat.data[i + 1], mat.data[i + 0]);

    YUVtoRGB(image.data[j + 2], image.data[j + 1], image.data[j + 3],
             mat.data[i + 5], mat.data[i + 4], mat.data[i + 3]);
  }
}

void VisualiseVision::YUVtoRGB(uint8_t y, uint8_t u, uint8_t v,
                               uint8_t& r, uint8_t& g, uint8_t& b) {
  double y_ = y, u_ = u, v_ = v;
  double r_, g_, b_;

  r_ = y_ + (1.370705 * (v_ - 128));
  g_ = y_ - (0.698001 * (v_ - 128)) - (0.337633 * (u_ - 128));
  b_ = y_ + (1.732446 * (u_ - 128));

  r = (r_ < 0) ? 0 : (r_ > 255) ? 255 : r_;
  g = (g_ < 0) ? 0 : (g_ > 255) ? 255 : g_;
  b = (b_ < 0) ? 0 : (b_ > 255) ? 255 : b_;
}

void VisualiseVision::PublishVisualisation(const std_msgs::Header& header,
                                           const cv::Mat& mat) {
  sensor_msgs::Image msg;
  msg.header = header;
  msg.width = mat.cols;
  msg.height = mat.rows;
  msg.encoding = "bgr8";
  msg.step = mat.cols * 3;
  msg.data.resize(msg.height * msg.step);
  memcpy(msg.data.data(), mat.data, msg.data.size());
  image_pub_.publish(msg);
}

void VisualiseVision::DrawHorizon(int level, cv::Mat& mat) {
  cv::line(mat, Point(0, level), Point(mat.cols, level), Scalar(64, 255, 64));
}

void VisualiseVision::DrawBall(const vision_msgs::BallDetection& ball,
                               cv::Mat& mat) {
  cv::circle(mat, Point(ball.pos_image.x, ball.pos_image.y),
             ball.radius, Scalar(0, 255, 255), 1);

  cv::circle(mat, Point(ball.pos_image.x, ball.pos_image.y),
             2, Scalar(0, 255, 255), -1);
}

void VisualiseVision::DrawLines(const vision_msgs::LineDetections& lines,
                                cv::Mat& mat) {
  for (size_t i = 0; i < lines.lines.size(); ++i) {
    const vision_msgs::LineDetection& l = lines.lines[i];
    cv::line(mat, Point(l.x1, l.y1), Point(l.x2, l.y2), Scalar(255, 0, 0));
  }
}
