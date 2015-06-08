#!/usr/bin/env python
##
# @file      adhoc.py
# @brief     Agent game state machine
# @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
# @date      2015-06-05
# @copyright (MIT) 2015 Edinferno

import roslib; roslib.load_manifest('agent')
import rospy
import smach
import smach_ros
from actionlib import *
from actionlib_msgs.msg import *
from navigation_msgs.msg import WalkToBallAction, WalkToBallGoal
from navigation_msgs.msg import SearchForBallAction, SearchForBallGoal
from motion_planning_msgs.msg import StandUpAction, StandUpGoal
from motion_planning_msgs.msg import SitDownAction, SitDownGoal
from motion_planning_msgs.msg import SitRestAction, SitRestGoal

# import agent # import all the package from src/agent
# needs the __init__.py well-formatted to work

# main
def main():
    rospy.init_node('agent_game_smach')

    # Create a SMACH state machine
    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    # Open the container
    with sm0:
        # Stand up
        smach.StateMachine.add('STAND_UP',
                smach_ros.SimpleActionState('motion_planning/stand_up',
                    StandUpAction,
                    goal = StandUpGoal(stand_up=True)),
               {'succeeded':'SEARCH_FOR_BALL',
               'aborted':'STAND_UP',
               'preempted':'preempted'})

        # Sit down
        smach.StateMachine.add('SIT_DOWN',
                smach_ros.SimpleActionState('motion_planning/sit_down',
                    SitDownAction,
                    goal = SitDownGoal(sit_down=True)),
               {'succeeded':'succeeded',
               'aborted':'aborted',
               'preempted':'preempted'})

        # Sit rest
        smach.StateMachine.add('SIT_REST',
                smach_ros.SimpleActionState('motion_planning/sit_rest',
                    SitRestAction,
                    goal = SitRestGoal(sit_rest=True)),
               {'succeeded':'succeeded',
               'aborted':'aborted',
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
               {'succeeded':'SIT_DOWN',
               'aborted':'SEARCH_FOR_BALL',
               'preempted':'preempted'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('smach_viewer', sm0, '/AGENT_GAME')
    sis.start()

    # Execute SMACH plan
    outcome = sm0.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

    rospy.signal_shutdown('All done.')

if __name__ == '__main__':
    main()
