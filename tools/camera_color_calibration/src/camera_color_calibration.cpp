/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-28
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: This is tool is for calibrating the cameras on the Nao. It relies on
*             edante to be running on the robot.
*/


#include <ros/ros.h>
#include <ros/package.h>

#include "camera_color_calibration/controller.hpp"

int main(int argc, char* argv[]) {
  // Create the application controller
  Controller app("edinferno.camera.color.calibration");

  // Get path to the UI glade file and create the view.
  std::string path = ros::package::getPath("camera_color_calibration");
  path += "/ui/ui.glade";
  app.CreateView(argc, argv, path);
  app.CreateModel(argc, argv);

  return app.Run();
}
