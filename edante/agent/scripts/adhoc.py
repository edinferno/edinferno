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

# define state StateSetup
class StateSetup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['complete'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SETUP')
        return 'complete'

# main
def main():
    rospy.init_node('agent_game_smach')

    # Create a SMACH state machine
    game_sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    # Open the container
    with game_sm:

        # Create the Initial SMACH state machine
        initial_sm = smach.StateMachine(outcomes=['penalized', 'ready'])

        with initial_sm:
            # Initial state setup
            smach.StateMachine.add('SETUP', StateSetup(),
                                   transitions={'complete':'ready'})
        # Initial state machine description
        smach.StateMachine.add('INITIAL', initial_sm,
                               transitions={'penalized':'PENALIZED',
                                            'ready':'READY'})

        # Create the Ready SMACH state machine
        ready_sm = smach.StateMachine(outcomes=['penalized', 'set'])

        with ready_sm:
            # Ready state setup
            smach.StateMachine.add('SETUP', StateSetup(),
                                   transitions={'complete':'set'})
        # Ready state machine description
        smach.StateMachine.add('READY', ready_sm,
                               transitions={'penalized':'PENALIZED',
                                            'set':'SET'})

        # Create the Set SMACH state machine
        set_sm = smach.StateMachine(outcomes=['penalized', 'playing'])

        with set_sm:
            # Set state setup
            smach.StateMachine.add('SETUP', StateSetup(),
                                   transitions={'complete':'playing'})
        # Set state machine description
        smach.StateMachine.add('SET', set_sm,
                               transitions={'penalized':'PENALIZED',
                                            'playing':'PLAYING'})

        # Create the Penalized SMACH state machine
        penalized_sm = smach.StateMachine(outcomes=['ready', 'set', 'playing'])

        with penalized_sm:
            # Penalized state setup
            smach.StateMachine.add('SETUP', StateSetup(),
                                   transitions={'complete':'playing'})
        # Penalized state machine description
        smach.StateMachine.add('PENALIZED', penalized_sm,
                               transitions={'ready':'READY',
                                            'set':'SET',
                                            'playing':'PLAYING'})

        # Create the Finished SMACH state machine
        finished_sm = smach.StateMachine(outcomes=['complete'])

        with finished_sm:
            # Initial state setup
            smach.StateMachine.add('SETUP', StateSetup(),
                                   transitions={'complete':'complete'})
        # Initial state machine description
        smach.StateMachine.add('FINISHED', finished_sm,
                               transitions={'complete':'succeeded'})

        # Create the Playing SMACH state machine
        playing_sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted', 'ready', 'penalized'])

        with playing_sm:
            # Playing state setup
            smach.StateMachine.add('SETUP', StateSetup(),
                                   transitions={'complete':'STAND_UP'})
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
        # Playing state machine description
        smach.StateMachine.add('PLAYING', playing_sm,
                               transitions={'succeeded':'FINISHED',
                                            'penalized':'PENALIZED',
                                            'ready':'READY'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('smach_viewer', game_sm, '/AGENT_GAME')
    sis.start()

    # Execute SMACH plan
    outcome = game_sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

    rospy.signal_shutdown('All done.')

if __name__ == '__main__':
    main()
