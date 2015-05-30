/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-28
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: The camera_color_calibration application follows the
*             Model View Controller (MVC) approach and this class
*             implements the controller which bridges the View
*             (UI interface) and the Model (program logic).
*/
#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <string>
#include <gtkmm.h>

#include <sensor_msgs/Image.h>

#include "camera_color_calibration/view.hpp"
#include "camera_color_calibration/model.hpp"
#include "camera_color_calibration/camera_color_calibration.hpp"


class Controller {
 public:
  explicit Controller(std::string app_name);

  void CreateView(int argc, char** argv, std::string filename);
  void CreateModel(int argc, char** argv);

  int Run() { return view_->Show(); }

  bool SpinOnce();

  // View callbacks
  void OnNewPixelClass(double x, double y, PixelClass pixel_class);
  bool OnSendTable();
  // Model callbacks
  void OnNewRawImage(const sensor_msgs::Image& msg);
  void OnNewSegmentedImage(const sensor_msgs::Image& msg);


 private:
  std::string app_name_;
  View* view_;
  Model* model_;
  sensor_msgs::Image image_;
};

#endif
