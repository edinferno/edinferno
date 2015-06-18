/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-06
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The class finds lines in a segmented image.
*/
# include "vision/line_detector.hpp"

// Camera
#include <camera/color_table.hpp>

// OpenCV
#include <opencv2/highgui/highgui.hpp>

LineDetector::LineDetector(ros::NodeHandle& nh) :
  lsd_(cv::createLineSegmentDetectorPtr()),
  line_detections_pub_(nh.advertise<vision_msgs::LineDetections>("lines", 5)),
  frames_count_(0) {
}

void LineDetector::ProcessImage(
  const cv::Mat& image,
  const image_geometry::PinholeCameraModel& cam_model) {
  ++frames_count_;
  if (frames_count_ % 30 != 1) {
    return;
  }

  image_ = image.clone();
  ThresholdImage(image_);
  lsd_->detect(image_, lines_);
  line_detections_.lines.resize(lines_.size());
  for (size_t i = 0; i < lines_.size(); ++i) {
    line_detections_.lines[i].x1 = lines_[i][0];
    line_detections_.lines[i].y1 = lines_[i][1];
    line_detections_.lines[i].x2 = lines_[i][2];
    line_detections_.lines[i].y2 = lines_[i][3];
  }
  line_detections_pub_.publish(line_detections_);
}
void LineDetector::ThresholdImage(cv::Mat& image) {
  int data_len = image.rows * image.step;
  for (int i = 0; i < data_len; ++i) {
    image.data[i] = (image.data[i] == GoalAndLines) ? 255 : 0;
  }
}


