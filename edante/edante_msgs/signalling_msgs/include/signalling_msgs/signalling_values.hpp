/**
 * @file      signalling_values.hpp
 * @brief     Contains signalling constants and enums
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-09
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef SIGNALLING_VALUES_HPP
#define SIGNALLING_VALUES_HPP

namespace Colors {
static const int BLACK = 0x000000;
static const int BLUE = 0x0000FF;
static const int GREEN = 0x00FF00;
static const int YELLOW = 0xFFFF00;
static const int RED = 0xFF0000;
static const int WHITE = 0xFFFFFF;
}
namespace GameState {
enum Game_State {
  INITIAL = 0,
  READY = 1,
  SET = 2,
  PLAYING = 3,
  FINISHED = 4,
  PENALIZED = 5
};
}
namespace MonitorMode {
enum Monitor_Mode {
  DISABLE = 0,
  BOTTOM_CAMERA = 1,
  TOP_CAMERA = 2,
  BALL_SEEN = 3,
  PTAM_ACTIVE = 4,
  BALL_LOST = 5,
  PTAM_LOST = 6
};
}

#endif /* SIGNALLING_VALUES_HPP_ */
