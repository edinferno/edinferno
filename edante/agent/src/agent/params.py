# @file      params.py
# @brief     Ready Game State smach
# @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
# @date      2015-06-05
# @copyright (MIT) 2015 Edinferno

# import roslib; roslib.load_manifest('agent')
import rospy
import smach
import smach_ros

from geometry_msgs.msg import Pose2D

class Params():
  def __init__(self):
    self.opp_penalty = rospy.get_param('/field_poses/opp_penalty')
    self.OPP_PENALTY = Pose2D();
    self.OPP_PENALTY.x, self.OPP_PENALTY.y, self.OPP_PENALTY.theta = self.opp_penalty['x'], self.opp_penalty['y'], self.opp_penalty['theta'];
