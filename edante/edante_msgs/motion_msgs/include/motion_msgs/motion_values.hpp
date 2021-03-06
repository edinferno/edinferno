/**
 * @file      motion_values.hpp
 * @brief     Define common motion values
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-19
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef MOTION_VALUES_HPP
#define MOTION_VALUES_HPP

const float PI = 3.1415927;
const float HALF_PI = 1.5707963;

#define FRAME_TORSO 0
#define FRAME_WORLD 1
#define FRAME_ROBOT 2

#define AXIS_MASK_X 1
#define AXIS_MASK_Y 2
#define AXIS_MASK_XY 3
#define AXIS_MASK_Z 4
#define AXIS_MASK_WX 8
#define AXIS_MASK_WY 16
#define AXIS_MASK_WZ 32
#define AXIS_MASK_WYWZ 48
#define AXIS_MASK_VEL 7
#define AXIS_MASK_ROT 56
#define AXIS_MASK_ALL 63
#define AXIS_MASK_NONE 0

#endif /* MOTION_VALUES_HPP */
