/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-29
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: The camera_color_calibration application follows the
*             Model View Controller (MVC) approach and this class
*             implements the View provides the user interface.
*/
#ifndef VIEW_HPP
#define VIEW_HPP

#include <string>
#include <gtkmm.h>

#include <sensor_msgs/Image.h>

class Controller;

class View {
 public:
  explicit View(Controller* controller);
  void Build(int argc, char** argv, std::string app_id, std::string filename);
  int Show() { return app_->run(*main_window_); }

  void SetRawImage(const sensor_msgs::Image& image);
  void RedrawArea(std::string name);

 private:
  Controller* controller_;
  Glib::RefPtr<Gtk::Application> app_;
  Glib::RefPtr<Gtk::Builder> builder_;
  Gtk::Window* main_window_;
  Glib::RefPtr<Gdk::Pixbuf> raw_pixbuf_;
  Glib::RefPtr<Gdk::Pixbuf> seg_pixbuf_;

  void ConnectToolButton(std::string name);
  void ConnectDrawingArea(std::string name);

  void YUVtoRGB(uint8_t y, uint8_t u, uint8_t v,
                uint8_t& r, uint8_t& g, uint8_t& b);

  // UI callbacks
  void OnToolbarButtonClicked(std::string name);
  bool OnButtonPressEvent(GdkEventButton* event, std::string name);
  bool OnDrawImage(const Cairo::RefPtr<Cairo::Context>& cr, std::string name);

};
#endif
