/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-10
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: Creates a set of 3D points representing the field.
*/

#ifndef FIELD_MODEL_HPP
#define FIELD_MODEL_HPP

// System
#include <cmath>
#include <vector>

// OpenCV
#include <opencv2/core/core.hpp>

class FieldModel {
 public:
  ////////////////////////
  // Official dimension //
  ////////////////////////
  // static const float kFieldLength          = 9.0f;   // A
  // static const float kFieldWidth           = 6.0f;   // B
  // static const float kLineWidth            = 0.05f;  // C
  // static const float kPenaltyMarkSize      = 0.1f;   // D
  // static const float kPenaltyAreaLength    = 0.6f;   // E
  // static const float kPenaltyAreaWidth     = 2.2f;   // F
  // static const float kPenaltyMarkDistance  = 1.3f;   // G
  // static const float kCenterCircleDiameter = 1.5f;   // H
  // static const float kBorderStripWidth     = 0.7f;   // I
  // static const float kGoalWidth            = 1.5f;   // J
  // static const float kGoalHeight           = 0.8f;   // K
  // static const float kGoalDepth            = 0.5f;   // L
  // static const float kGoalBeamWidth        = 0.1f;   // M

  ///////////////////
  // InSpace pitch //
  ///////////////////
  static const float kFieldLength          = 6.6f;   // A
  static const float kFieldWidth           = 4.62f;  // B
  static const float kLineWidth            = 0.05f;  // C
  static const float kPenaltyMarkSize      = 0.13f;  // D
  static const float kPenaltyAreaLength    = 0.61f;  // E
  static const float kPenaltyAreaWidth     = 2.21f;  // F
  static const float kPenaltyMarkDistance  = 1.78f;  // G
  static const float kCenterCircleDiameter = 1.47f;  // H
  static const float kBorderStripWidth     = 0.7f;   // I
  static const float kGoalWidth            = 1.5f;   // J
  static const float kGoalHeight           = 0.8f;   // K
  static const float kGoalDepth            = 0.5f;   // L
  static const float kGoalBeamWidth        = 0.1f;   // M

  FieldModel(double max_sample_distance = 1.0,
             double max_sample_angle = M_PI / 3);

  std::vector<cv::Point3d> lines;
  std::vector<cv::Point3d> field;

 private:
  void SampleLine(cv::Point3d from, cv::Point3d to,
                  double max_distance,
                  std::vector<cv::Point3d>& points);

  void SampleCircle(cv::Point3d center, double r, double angle_step,
                    std::vector<cv::Point3d>& points);

  void SampleField(std::vector<cv::Point3d>& points);
};

#endif
