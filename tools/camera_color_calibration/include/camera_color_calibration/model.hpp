/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-29
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: The camera_color_calibration application follows the
*             Model View Controller (MVC) approach and this class
*             represents the Model which implements the program logic.
*/
#ifndef MODEL_HPP
#define MODEL_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>

#include "camera_color_calibration/camera_color_calibration.hpp"

class Controller;

class Model {
 public:
  explicit Model(Controller* controller);
  void Build(int argc, char** argv);
  void AddNewPixelClass(double x, double y, PixelClass pixel_class);
  bool LoadTable();
  bool SendTable();


 private:
  Controller* controller_;
  ros::NodeHandle* nh_;
  image_transport::ImageTransport* it_;
  image_transport::Subscriber image_sub_;

  sensor_msgs::Image raw_image_;
  sensor_msgs::Image seg_image_;
  // Color table
  static const size_t kTableSize = 64;
  static const size_t kTableLen = kTableSize * kTableSize * kTableSize;
  PixelClass table_[kTableSize][kTableSize][kTableSize];

  void SegmentImage(const sensor_msgs::Image& raw,
                    sensor_msgs::Image& seg,
                    PixelClass table[kTableSize][kTableSize][kTableSize]);

  void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
};
#endif
