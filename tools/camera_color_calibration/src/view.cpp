/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-29
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: The camera_color_calibration application follows the
*             Model View Controller (MVC) approach and this class
*             implements the View provides the user interface.
*/

#include "camera_color_calibration/view.hpp"
#include "camera_color_calibration/controller.hpp"

#include <ros/ros.h>

View::View(Controller* controller) {
  controller_ = controller;
}

void View::Build(int argc, char** argv,
                 std::string app_id,
                 std::string filename) {
  app_ = Gtk::Application::create(argc, argv, app_id);

  builder_ = Gtk::Builder::create();
  builder_->add_from_file(filename);
  builder_->get_widget("main_window", main_window_);

  ConnectToolButton("grey");
  ConnectToolButton("orange");
  ConnectToolButton("white");
  ConnectToolButton("green");
  ConnectToolButton("red");
  ConnectToolButton("blue");

  ConnectDrawingArea("raw");
  ConnectDrawingArea("segmented");


  Glib::signal_timeout().connect(
    sigc::mem_fun(*controller_,
                  &Controller::SpinOnce),
    30);  // Timeout value in ms
}
void View::SetRawImage(const sensor_msgs::Image& image) {
  raw_pixbuf_ = Gdk::Pixbuf::create(Gdk::COLORSPACE_RGB, false, 8,
                                    image.width, image.height);
  guint8* pixels = raw_pixbuf_->get_pixels();
  if (image.encoding == "bgr8") {
    for (size_t i = 0; i < image.data.size(); i += 3) {
      pixels[i]     = image.data[i + 2];
      pixels[i + 1] = image.data[i + 1];
      pixels[i + 2] = image.data[i];
    }
  } else if (image.encoding == "yuv422") {
    size_t num_pixels = image.width * image.height * 3;
    for (size_t i = 0, j = 0; i < num_pixels; i += 6, j += 4) {
      YUVtoRGB(image.data[j + 0], image.data[j + 1], image.data[j + 3],
               pixels[i + 0], pixels[i + 1], pixels[i + 2]);

      YUVtoRGB(image.data[j + 2], image.data[j + 1], image.data[j + 3],
               pixels[i + 3], pixels[i + 4], pixels[i + 5]);
    }
  }
}

void View::RedrawArea(std::string name) {
  Gtk::DrawingArea* area;
  builder_->get_widget(name + "_area", area);
  area->queue_draw();
}

void View::ConnectToolButton(std::string name) {
  Gtk::RadioToolButton* tool_button;
  builder_->get_widget(name + "_tool_button", tool_button);
  tool_button->signal_toggled().connect(
    sigc::bind<std::string>(
      sigc::mem_fun(*this,
                    &View::OnToolbarButtonClicked),
      name));
}

void View::ConnectDrawingArea(std::string name) {
  Gtk::DrawingArea* area;
  builder_->get_widget(name + "_area", area);
  area->add_events(Gdk::BUTTON_PRESS_MASK);
  area->signal_button_press_event().connect(
    sigc::bind<std::string>(
      sigc::mem_fun(*this,
                    &View::OnButtonPressEvent),
      name));

  area->signal_draw().connect(
    sigc::bind<std::string>(
      sigc::mem_fun(*this,
                    &View::OnDrawImage),
      name));

}

void View::YUVtoRGB(uint8_t y, uint8_t u, uint8_t v,
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

void View::OnToolbarButtonClicked(std::string name) {
  ROS_INFO_STREAM("Clicked:" << name);
}

bool View::OnButtonPressEvent(GdkEventButton* event,
                              std::string name) {
  ROS_INFO("Click on %s @ [%lf, %lf]", name.c_str(), event->x, event->y);
  return true;
}

bool View::OnDrawImage(const Cairo::RefPtr<Cairo::Context>& cr,
                       std::string name) {
  if (name == "raw") {
    if (raw_pixbuf_ == 0) return true;
    Gdk::Cairo::set_source_pixbuf(cr, raw_pixbuf_, 0, 0);
    cr->paint();
  } else if (name == "segmented") {
    if (seg_pixbuf_ == 0) return true;
    Gdk::Cairo::set_source_pixbuf(cr, seg_pixbuf_, 0, 0);
    cr->paint();
  }
  return true;
}
