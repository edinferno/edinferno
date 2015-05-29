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



class Controller;

class Model {
 public:
  explicit Model(Controller* controller);
  void Build(int argc, char** argv);
 private:
  Controller* controller_;
  ros::NodeHandle* nh_;
  image_transport::ImageTransport* it_;
  image_transport::Subscriber image_sub_;
};
#endif
