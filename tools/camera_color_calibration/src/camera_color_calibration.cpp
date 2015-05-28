/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-28
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: This is tool is for calibrating the cameras on the Nao. It relies on
*             edante to be running on the robot.
*/
#include <gtkmm.h>

#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char* argv[]) {
  Glib::RefPtr<Gtk::Application> app =
    Gtk::Application::create(argc, argv, "edinferno.camera.color.calibration");


  // Load the GtkBuilder file and instantiate its widgets:
  Glib::RefPtr<Gtk::Builder> refBuilder = Gtk::Builder::create();
  try {
    std::string path = ros::package::getPath("camera_color_calibration");
    refBuilder->add_from_file(path + "/ui/ui.glade");
  } catch (const Glib::FileError& ex) {
    ROS_FATAL_STREAM("FileError: " << ex.what());
    return 1;
  } catch (const Glib::MarkupError& ex) {
    ROS_FATAL_STREAM("MarkupError: " << ex.what());
    return 1;
  } catch (const Gtk::BuilderError& ex) {
    ROS_FATAL_STREAM("BuilderError: " << ex.what());
    return 1;
  }

  Gtk::Window* p_window = 0;
  refBuilder->get_widget("main_window", p_window);

  return app->run(*p_window);
}
