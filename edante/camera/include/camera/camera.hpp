/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-24
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: This is the main class which represents a camera.
*             It provides functionality to start, stop and
*             calibrate the camera.
*/
#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

class Camera {
 public:
  Camera(ros::NodeHandle& nh, int id, const char* name);

  bool Init();
  void SpinOnce();

 private:
  void Read();
  void Publish();

  ros::NodeHandle nh_;

  const int id_;

  std::string name_;
  std::string cam_info_url_;
  std::string frame_id_;

  std::auto_ptr<cv::VideoCapture> cap_;

  std::auto_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;
  sensor_msgs::CameraInfo cam_info_;

  std::auto_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher image_pub_;

  sensor_msgs::Image image_;
  cv::Mat mat_;
};
#endif
