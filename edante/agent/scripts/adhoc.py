#!/usr/bin/env python
##
# @file      game.py
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

# import agent # import all the package from src/agent
# needs the __init__.py well-formatted to work

# main
def main():
    rospy.init_node('agent_game_smach')

    # Create a SMACH state machine
    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    # Open the container
    with sm0:
        # Add states to the container
        smach.StateMachine.add('WALK_TO_BALL',
                smach_ros.SimpleActionState('navigation/walk_to_ball',
                    WalkToBallAction,
                    goal = WalkToBallGoal(start_walk=True)),
               {'succeeded':'succeeded',
               'aborted':'aborted',
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
