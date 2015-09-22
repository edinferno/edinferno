# @file      finished_sm.py
# @brief     Finished Game State smach
# @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
# @date      2015-06-05
# @copyright (MIT) 2015 Edinferno

# import roslib; roslib.load_manifest('agent')
import rospy
import smach
import smach_ros

from actionlib import *
from actionlib_msgs.msg import *
from motion_planning_msgs.msg import SitRestAction, SitRestGoal

class FinishedSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['succeeded','aborted', 'preempted'])
        with self:

            # Sit rest
            smach.StateMachine.add('SIT_REST',
                    smach_ros.SimpleActionState('motion_planning/sit_rest',
                        SitRestAction,
                        goal = SitRestGoal(sit_rest=True)),
                   {'succeeded':'succeeded',
                   'aborted':'aborted',
                   'preempted':'preempted'})
