/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-01
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: Color table definitions
*/

// TODO(svepe): Use this header for the camera color calibration.
// If you change the enum, remember to update camera_color_calibration.hpp
enum PixelClass {
  Nothing = 0,
  Ball = 255,
  GoalAndLines = 192,
  Field = 32,
  TeamRed = 96,
  TeamBlue = 128
};
