/**
 * @file      signalling_values.hpp
 * @brief     Contains signalling constants and enums
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-09
 * @copyright (MIT) 2015 Edinferno
 */

namespace Colors {
static const int BLACK = 0x000000;
static const int BLUE = 0x0000FF;
static const int GREEN = 0x00FF00;
static const int YELLOW = 0xFFFF00;
static const int RED = 0xFF0000;
static const int WHITE = 0xFFFFFF;
}
namespace DebugState {
enum Debug_State {
  BALL_SEEN = 0,
  BALL_NOT_SEEN
};
}
namespace GameState {
enum Game_State {
  INITIAL = 0,
  READY = 1,
  SET = 2,
  PENALIZED = 3,
  PLAYING = 4,
  FINISHED = 5
};
}
