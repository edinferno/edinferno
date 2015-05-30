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
  current_class_ = Nothing;
}

void View::Build(int argc, char** argv,
                 std::string app_id,
                 std::string filename) {
  app_ = Gtk::Application::create(argc, argv, app_id);

  builder_ = Gtk::Builder::create();
  builder_->add_from_file(filename);
  builder_->get_widget("main_window", main_window_);

  ConnectRadioToolButton("grey");
  ConnectRadioToolButton("orange");
  ConnectRadioToolButton("white");
  ConnectRadioToolButton("green");
  ConnectRadioToolButton("red");
  ConnectRadioToolButton("blue");
  ConnectToolButton("send");

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

  size_t num_pixels = image.width * image.height * 3;
  for (size_t i = 0, j = 0; i < num_pixels; i += 6, j += 4) {
    YUVtoRGB(image.data[j + 0], image.data[j + 1], image.data[j + 3],
             pixels[i + 0], pixels[i + 1], pixels[i + 2]);

    YUVtoRGB(image.data[j + 2], image.data[j + 1], image.data[j + 3],
             pixels[i + 3], pixels[i + 4], pixels[i + 5]);
  }

  Gtk::DrawingArea* area;
  builder_->get_widget("raw_area", area);
  raw_pixbuf_ = raw_pixbuf_->scale_simple(area->get_width(),
                                          area->get_height(),
                                          Gdk::INTERP_BILINEAR);
}

void View::SetSegmentedImage(const sensor_msgs::Image& image) {
  seg_pixbuf_ = Gdk::Pixbuf::create(Gdk::COLORSPACE_RGB, false, 8,
                                    image.width, image.height);
  guint8* pixels = seg_pixbuf_->get_pixels();

  size_t num_pixels = image.width * image.height;
  for (size_t i = 0, j = 0; i < num_pixels; ++i, j += 3) {
    switch (image.data[i]) {
      case Nothing: {
        // Gray
        pixels[j]     = 128;
        pixels[j + 1] = 128;
        pixels[j + 2] = 128;
        break;
      }
      case Ball: {
        // Orange
        pixels[j]     = 255;
        pixels[j + 1] = 128;
        pixels[j + 2] = 0;
        break;
      }
      case GoalAndLines: {
        pixels[j]     = 255;
        pixels[j + 1] = 255;
        pixels[j + 2] = 255;
        break;
      }
      case Field: {
        pixels[j]     = 64;
        pixels[j + 1] = 255;
        pixels[j + 2] = 64;
        break;
      }
      case TeamRed: {
        pixels[j]     = 255;
        pixels[j + 1] = 0;
        pixels[j + 2] = 0;
        break;
      }
      case TeamBlue: {
        pixels[j]     = 0;
        pixels[j + 1] = 0;
        pixels[j + 2] = 255;
        break;
      }
    }
  }

  Gtk::DrawingArea* area;
  builder_->get_widget("segmented_area", area);
  seg_pixbuf_ = seg_pixbuf_->scale_simple(area->get_width(),
                                          area->get_height(),
                                          Gdk::INTERP_BILINEAR);
}

void View::RedrawArea(std::string name) {
  Gtk::DrawingArea* area;
  builder_->get_widget(name + "_area", area);
  area->queue_draw();
}

void View::ConnectRadioToolButton(std::string name) {
  Gtk::RadioToolButton* tool_button;
  builder_->get_widget(name + "_tool_button", tool_button);
  tool_button->signal_toggled().connect(
    sigc::bind<std::string>(
      sigc::mem_fun(*this,
                    &View::OnToolbarButtonClicked),
      name));
}

void View::ConnectToolButton(std::string name) {
  Gtk::ToolButton* tool_button;
  builder_->get_widget(name + "_tool_button", tool_button);
  tool_button->signal_clicked().connect(
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
  if (name == "grey") {
    current_class_ = Nothing;
  } else if (name == "orange") {
    current_class_ = Ball;
  } else if (name == "white") {
    current_class_ = GoalAndLines;
  } else if (name == "green") {
    current_class_ = Field;
  } else if (name == "red") {
    current_class_ = TeamRed;
  } else if (name == "blue") {
    current_class_ = TeamBlue;
  } else if (name == "send") {
    controller_->OnSendTable();
  }
}

bool View::OnButtonPressEvent(GdkEventButton* event,
                              std::string name) {
  Gtk::DrawingArea* area;
  builder_->get_widget(name + "_area", area);
  // Right mouse click always clears
  PixelClass pixel_class = (event->button == 3) ? Nothing : current_class_;
  controller_->OnNewPixelClass(event->x / area->get_width(),
                               event->y / area->get_height(),
                               pixel_class);
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
