# @file      playing_sm.py
# @brief     Playing Game State smach
# @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
# @date      2015-06-05
# @copyright (MIT) 2015 Edinferno

# import roslib; roslib.load_manifest('agent')
import rospy
import smach
import smach_ros

from actionlib import *
from actionlib_msgs.msg import *
from navigation_msgs.msg import WalkToBallAction, WalkToBallGoal
from navigation_msgs.msg import SearchForBallAction, SearchForBallGoal
from motion_planning_msgs.msg import SetupAction, SetupGoal
from motion_planning_msgs.msg import StandUpAction, StandUpGoal
from motion_planning_msgs.msg import SitDownAction, SitDownGoal
from motion_planning_msgs.msg import SitRestAction, SitRestGoal
from motion_planning_msgs.msg import TransitionAction, TransitionGoal
from motion_planning_msgs.msg import KickStrongAction, KickStrongGoal

class PlayingSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['succeeded','aborted', 'preempted'])
        with self:

            # Stand up
            smach.StateMachine.add('STAND_UP',
                    smach_ros.SimpleActionState('motion_planning/stand_up',
                        StandUpAction,
                        goal = StandUpGoal(stand_up=True)),
                   {'succeeded':'SEARCH_FOR_BALL',
                   'aborted':'STAND_UP',
                   'preempted':'preempted'})

            # Search for ball
            smach.StateMachine.add('SEARCH_FOR_BALL',
                    smach_ros.SimpleActionState('navigation/search_for_ball',
                        SearchForBallAction,
                        goal = SearchForBallGoal(start_search=True)),
                   {'succeeded':'WALK_TO_BALL',
                   'aborted':'SEARCH_FOR_BALL',
                   'preempted':'preempted'})

            # Walk to ball
            smach.StateMachine.add('WALK_TO_BALL',
                    smach_ros.SimpleActionState('navigation/walk_to_ball',
                        WalkToBallAction,
                        goal = WalkToBallGoal(start_walk=True)),
                   {'succeeded':'KICK_STRONG',
                   'aborted':'SEARCH_FOR_BALL',
                   'preempted':'preempted'})

            # Sit rest
            smach.StateMachine.add('KICK_STRONG',
                    smach_ros.SimpleActionState('motion_planning/kick_strong',
                        KickStrongAction,
                        goal = KickStrongGoal(kick_type=1)),
                   {'succeeded':'SEARCH_FOR_BALL',
                   'aborted':'STAND_UP',
                   'preempted':'preempted'})
