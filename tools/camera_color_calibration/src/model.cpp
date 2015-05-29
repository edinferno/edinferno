/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-29
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: The camera_color_calibration application follows the
*             Model View Controller (MVC) approach and this class
*             represents the Model which implements the program logic.
*/
#include "camera_color_calibration/model.hpp"
#include "camera_color_calibration/controller.hpp"

Model::Model(Controller* controller) {
  controller_ = controller;
}

void Model::Build(int argc, char** argv) {
  // Initialise ROS
  ros::init(argc, argv, "camera_color_calibration_node");

  // Create NodeHandle
  nh_ = new ros::NodeHandle("camera");

  // Create ImageTransport
  it_ = new image_transport::ImageTransport(*nh_);

  // Subscribe to camera images
  image_sub_ = it_->subscribe("image", 1, &Controller::ImageCallback,
                              controller_);
}
