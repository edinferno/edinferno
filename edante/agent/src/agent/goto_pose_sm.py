# @file      goto_pose_sm.py
# @brief     Ready Game State smach
# @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
# @date      2015-06-05
# @copyright (MIT) 2015 Edinferno

# import roslib; roslib.load_manifest('agent')
import rospy
import smach
import smach_ros

from actionlib import *
from actionlib_msgs.msg import *
# from params import *
from motion_planning_msgs.msg import StandUpAction, StandUpGoal
from navigation_msgs.msg import WalkToPoseAction, WalkToPoseGoal
from navigation_msgs.msg import TurnToPoseAction, TurnToPoseGoal
from navigation_msgs.msg import AlignToPoseAction, AlignToPoseGoal
from geometry_msgs.msg import Pose2D

class GotoPoseSM(smach.StateMachine):
    def __init__(self):
        print(p.OPP_PENALTY)
        smach.StateMachine.__init__(self,outcomes=['succeeded','aborted', 'preempted'])
        with self:
            # Turn towards target pose
            smach.StateMachine.add('TURN_TO_POSE',
                    smach_ros.SimpleActionState('navigation/turn_to_pose',
                        TurnToPoseAction,
                        goal = TurnToPoseGoal(target_pose=OPP_PENALTY)),
                   {'succeeded':'WALK_TO_POSE',
                   'aborted':'STAND_UP',
                   'preempted':'preempted'})
            # Walks directly towards target pose
            smach.StateMachine.add('WALK_TO_POSE',
                    smach_ros.SimpleActionState('navigation/walk_to_pose',
                        WalkToPoseAction,
                        goal = WalkToPoseGoal(target_pose=OPP_PENALTY)),
                   {'succeeded':'ALIGN_TO_POSE',
                   'aborted':'STAND_UP',
                   'preempted':'preempted'})
            # Aligns to the required pose
            smach.StateMachine.add('ALIGN_TO_POSE',
                    smach_ros.SimpleActionState('navigation/align_to_pose',
                        AlignToPoseAction,
                        goal = AlignToPoseGoal(target_pose=OPP_PENALTY)),
                   {'succeeded':'succeeded',
                   'aborted':'STAND_UP',
                   'preempted':'preempted'})
            # Stand up
            smach.StateMachine.add('STAND_UP',
                    smach_ros.SimpleActionState('motion_planning/stand_up',
                        StandUpAction,
                        goal = StandUpGoal(stand_up=True)),
                   {'succeeded':'TURN_TO_POSE',
                   'aborted':'STAND_UP',
                   'preempted':'preempted'})
