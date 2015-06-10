/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-10
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The class implements the functionality for the particles
*             which represent robot pose hypotheses.
*/
#include "vision/pose_particle.hpp"

#include <cmath>

using cv::Point2i;
using cv::Point2d;
using cv::Point3d;

PoseParticle::PoseParticle(cv::Point2f xy, float theta) :
  xy_(xy), theta_(theta), cos_theta(cos(-theta)), sin_theta(sin(-theta)) {
}
void PoseParticle::Project(FieldModel& field_model,
                           const image_geometry::PinholeCameraModel& cam_model,
                           std::vector<float> cam_transform,
                           std::vector<cv::Point2i>& lines,
                           std::vector<cv::Point2i>& field) {
  for (size_t i = 0; i < field_model.lines.size(); ++i) {
    lines.push_back(ProjectPoint(field_model.lines[i],
                                 cam_model,
                                 cam_transform));
  }
  for (size_t i = 0; i < field_model.field.size(); ++i) {
    lines.push_back(ProjectPoint(field_model.field[i],
                                 cam_model,
                                 cam_transform));
  }
}

cv::Point2i PoseParticle::ProjectPoint(
  cv::Point3d pt,
  const image_geometry::PinholeCameraModel& cam_model,
  std::vector<float> cam_transform) {
  // Get the field point in the robot frame given the particle pose.
  Point3d trans_pt(0, 0, 0);
  trans_pt.x = pt.x - xy_.x;
  trans_pt.y = pt.y - xy_.y;

  Point3d robot_frame(0, 0, 0);
  robot_frame.x = trans_pt.x * cos_theta - trans_pt.y * sin_theta;
  robot_frame.y = trans_pt.x * sin_theta + trans_pt.y * cos_theta;

  // Use the inverse transform i.e. from robot frame to camera frame
  Point3d cam_frame;
  cam_frame.x = cam_transform[0] * robot_frame.x +
                cam_transform[4] * robot_frame.y - cam_transform[3];
  cam_frame.y = cam_transform[1] * robot_frame.x +
                cam_transform[5] * robot_frame.y - cam_transform[7];
  cam_frame.z = cam_transform[2] * robot_frame.x +
                cam_transform[6] * robot_frame.y - cam_transform[11];

  // Get the camera frame point into optical camera frame coordinates.
  Point3d optical_frame;
  optical_frame.x = -cam_frame.y;
  optical_frame.y = -cam_frame.z;
  optical_frame.z = cam_frame.x;

  // Project the point back onto the image
  Point2d image_frame = cam_model.project3dToPixel(optical_frame);
  image_frame = cam_model.unrectifyPoint(image_frame);
  return Point2i(image_frame.x, image_frame.y);
}
