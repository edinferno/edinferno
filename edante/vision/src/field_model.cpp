/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-10
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: Creates a set of 3D points representing the field.
*/
#include "vision/field_model.hpp"

using cv::Point3d;

FieldModel::FieldModel(double max_sample_distance,
                       double max_sample_angle) {
  Point3d from, to;

  // Field main lines
  // Line 1
  from = Point3d(-kFieldLength / 2, kFieldWidth / 2, 0);
  to = Point3d(kFieldLength / 2, kFieldWidth / 2, 0);
  SampleLine(from, to, max_sample_distance, lines);
  // Line 2
  from = Point3d(-kFieldLength / 2, -kFieldWidth / 2, 0);
  to = Point3d(kFieldLength / 2, -kFieldWidth / 2, 0);
  SampleLine(from, to, max_sample_distance, lines);
  // Line 3
  from = Point3d(-kFieldLength / 2, -kFieldWidth / 2, 0);
  to = Point3d(-kFieldLength / 2, kFieldWidth / 2, 0);
  SampleLine(from, to, max_sample_distance, lines);
  // Line 4
  from = Point3d(kFieldLength / 2, -kFieldWidth / 2, 0);
  to = Point3d(kFieldLength / 2, kFieldWidth / 2, 0);
  SampleLine(from, to, max_sample_distance, lines);
  // Line 5
  from = Point3d(0, -kFieldWidth / 2, 0);
  to = Point3d(0, kFieldWidth / 2, 0);
  SampleLine(from, to, max_sample_distance, lines);

  // Penalty areas
  // Line 6
  from = Point3d(-kFieldLength / 2 + kPenaltyAreaLength,
                 -kPenaltyAreaWidth / 2, 0);
  to = Point3d(-kFieldLength / 2 + kPenaltyAreaLength,
               kPenaltyAreaWidth / 2, 0);
  SampleLine(from, to, max_sample_distance, lines);

  // Line 7
  from = Point3d(-kFieldLength / 2,
                 kPenaltyAreaWidth / 2, 0);
  to = Point3d(-kFieldLength / 2 + kPenaltyAreaLength,
               kPenaltyAreaWidth / 2, 0);
  SampleLine(from, to, max_sample_distance, lines);

  // Line 8
  from = Point3d(-kFieldLength / 2,
                 -kPenaltyAreaWidth / 2, 0);
  to = Point3d(-kFieldLength / 2 + kPenaltyAreaLength,
               -kPenaltyAreaWidth / 2, 0);
  SampleLine(from, to, max_sample_distance, lines);

  // Line 9
  from = Point3d(kFieldLength / 2 - kPenaltyAreaLength,
                 -kPenaltyAreaWidth / 2, 0);
  to = Point3d(kFieldLength / 2 - kPenaltyAreaLength,
               kPenaltyAreaWidth / 2, 0);
  SampleLine(from, to, max_sample_distance, lines);

  // Line 10
  from = Point3d(kFieldLength / 2,
                 kPenaltyAreaWidth / 2, 0);
  to = Point3d(kFieldLength / 2 - kPenaltyAreaLength,
               kPenaltyAreaWidth / 2, 0);
  SampleLine(from, to, max_sample_distance, lines);

  // Line 11
  from = Point3d(kFieldLength / 2,
                 -kPenaltyAreaWidth / 2, 0);
  to = Point3d(kFieldLength / 2 - kPenaltyAreaLength,
               -kPenaltyAreaWidth / 2, 0);
  SampleLine(from, to, max_sample_distance, lines);

  // Center circle
  SampleCircle(Point3d(0, 0, 0), kCenterCircleDiameter / 2,
               max_sample_angle, lines);

  // Penalty marks
  lines.push_back(Point3d(-kFieldLength / 2 + kPenaltyMarkDistance, 0, 0));
  lines.push_back(Point3d(kFieldLength / 2 - kPenaltyMarkDistance, 0, 0));

  SampleField(field);
}

void FieldModel::SampleLine(cv::Point3d from, cv::Point3d to,
                            double max_distance,
                            std::vector<cv::Point3d>& points) {
  double line_length = cv::norm(to - from);

  int n_points = ceil(line_length / max_distance) + 1;

  if (n_points < 3) n_points = 3;

  Point3d delta = Point3d((to - from) * (1.0 / (n_points - 1)));

  points.push_back(from);

  for (int i = 1; i < n_points - 1; ++i) {
    points.push_back(from + i * delta);
  }

  points.push_back(to);
}

void FieldModel::SampleCircle(cv::Point3d center, double r, double angle_step,
                              std::vector<cv::Point3d>& points) {
  double angle = 0;
  do {
    points.push_back(Point3d(r * cos(angle), r * sin(angle), 0));
    angle += angle_step;
  } while (angle < 2 * M_PI);
}

void FieldModel::SampleField(std::vector<cv::Point3d>& points) {
  // Penalty zones
  points.push_back(Point3d(kFieldLength / 2 - kPenaltyAreaLength / 2, 0, 0));
  points.push_back(Point3d(-kFieldLength / 2 + kPenaltyAreaLength / 2, 0, 0));

  // Above penalty zones
  points.push_back(Point3d(kFieldLength / 2 - kPenaltyAreaLength / 2,
                           kFieldWidth / 4, 0));
  points.push_back(Point3d(-kFieldLength / 2 + kPenaltyAreaLength / 2,
                           kFieldWidth / 4, 0));

  // Below penalty zones
  points.push_back(Point3d(kFieldLength / 2 - kPenaltyAreaLength / 2,
                           -kFieldWidth / 4, 0));
  points.push_back(Point3d(-kFieldLength / 2 + kPenaltyAreaLength / 2,
                           -kFieldWidth / 4, 0));

  // Field half center
  points.push_back(Point3d(kFieldLength / 4, 0, 0));
  points.push_back(Point3d(-kFieldLength / 4, 0, 0));

  // Above field half center
  points.push_back(Point3d(kFieldLength / 4,
                           kFieldWidth / 4, 0));
  points.push_back(Point3d(-kFieldLength / 4,
                           kFieldWidth / 4, 0));

  // Below field half center
  points.push_back(Point3d(kFieldLength / 4,
                           -kFieldWidth / 4, 0));
  points.push_back(Point3d(-kFieldLength / 4,
                           -kFieldWidth / 4, 0));
}
