/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-10
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The class implements the functionality for the particles
*             which represent robot pose hypotheses.
*/
#ifndef POSE_PARTICLE_HPP
#define POSE_PARTICLE_HPP

// System
#include <vector>

// OpenCV
#include <opencv2/core/core.hpp>

// ROS
#include <image_geometry/pinhole_camera_model.h>

// Local
#include "vision/field_model.hpp"

class PoseParticle {
 public:
  PoseParticle(cv::Point2f xy, float theta);
  void Project(FieldModel& field_model,
               const image_geometry::PinholeCameraModel& cam_model,
               std::vector<float> cam_transform,
               std::vector<cv::Point2i>& lines,
               std::vector<cv::Point2i>& field);

  bool ProjectPoint(cv::Point3d pt,
                    const image_geometry::PinholeCameraModel& cam_model,
                    std::vector<float> cam_transform,
                    cv::Point2i& proj_pt);
 private:
  cv::Point2f xy_;
  float theta_;
  float cos_theta;
  float sin_theta;
};

#endif
