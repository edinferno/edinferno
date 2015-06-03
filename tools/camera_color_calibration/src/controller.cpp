/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-28
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: The camera_color_calibration application follows the
*             Model View Controller (MVC) approach and this class
*             implements the controller which bridges the View
*             (UI interface) and the Model (program logic).
*/
#include "camera_color_calibration/controller.hpp"

#include <ros/ros.h>

Controller::Controller(std::string app_name) :
  app_name_(app_name) {
}

void Controller::CreateView(int argc, char** argv, std::string filename) {
  view_ = new View(this);
  view_->Build(argc, argv, app_name_, filename);
}

void Controller::CreateModel(int argc, char** argv) {
  model_ = new Model(this);
  model_->Build(argc, argv);
}

bool Controller::SpinOnce() {
  ros::spinOnce();
  // Make sure the timer is not stopped
  return true;
}

void Controller::OnNewPixelClass(double x, double y, PixelClass pixel_class) {
  model_->AddNewPixelClass(x, y, pixel_class);
}

void Controller::OnSwitchCamera() {
  model_->SwitchCamera();
}

bool Controller::OnSendTable() {
  return model_->SendTable();
}

void Controller::OnNewRawImage(const sensor_msgs::Image& msg) {
  view_->SetRawImage(msg);
  view_->RedrawArea("raw");
}

void Controller::OnNewSegmentedImage(const sensor_msgs::Image& msg) {
  view_->SetSegmentedImage(msg);
  view_->RedrawArea("segmented");
}

