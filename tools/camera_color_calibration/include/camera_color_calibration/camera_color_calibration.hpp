/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-29
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: Contains color calibration related information used by several classes
*/

#ifndef CAMERA_COLOR_CALIBRATION_HPP
#define CAMERA_COLOR_CALIBRATION_HPP

enum PixelClass {
  Nothing = 0,
  Ball = 255,
  GoalAndLines = 192,
  Field = 32,
  TeamRed = 96,
  TeamBlue = 128
};

#endif
