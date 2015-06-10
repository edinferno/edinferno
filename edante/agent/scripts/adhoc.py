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
from motion_planning_msgs.msg import SetupAction, SetupGoal
from motion_planning_msgs.msg import StandUpAction, StandUpGoal
from motion_planning_msgs.msg import SitDownAction, SitDownGoal
from motion_planning_msgs.msg import SitRestAction, SitRestGoal
from motion_planning_msgs.msg import AwaitTransitionAction, AwaitTransitionGoal

# gets called when ANY child state terminates
def playing_child_term_cb(outcome_map):

  # terminate all running states if FOO finished with outcome 'outcome3'
  if outcome_map['AWAIT_TRANSITION'] == 'ready':
    return True

  # terminate all running states if BAR finished
  if outcome_map['AWAIT_TRANSITION'] == 'penalized':
    return True

  # in all other case, just keep running, don't terminate anything
  return False


# gets called when ALL child states are terminated
def playing_all_term_cb(outcome_map):
    if outcome_map['AWAIT_TRANSITION'] == 'ready':
        return 'ready'
    else:
        return 'penalized'

class Playing_sm(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['succeeded','aborted', 'preempted'])
        with self:
            # Stand up
            smach.StateMachine.add('STAND_UP',
                    smach_ros.SimpleActionState('motion_planning/stand_up',
                        StandUpAction,
                        goal = StandUpGoal(stand_up=True)),
                   {'succeeded':'succeeded',
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

# main
def main():
    rospy.init_node('agent_game_smach')

    # GAME STATES
    INITIAL = 0
    READY=1
    SET=2
    PENALIZED=3
    PLAYING=4
    FINISHED=5

    # Create the main game SMACH state machine
    game_sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    with game_sm:
        # ======================================================================
        # Create the Initial SMACH state machine
        initial_sm = smach.StateMachine(outcomes=['penalized', 'ready', 'succeeded', 'aborted', 'preempted'])
        with initial_sm:
            # Initial state setup
            smach.StateMachine.add('SETUP',
                    smach_ros.SimpleActionState('motion_planning/setup',
                        SetupAction,
                        goal = SetupGoal(state=INITIAL)),
                   {'succeeded':'AWAIT_TRANSITION'})

            # Initial state setup
            smach.StateMachine.add('AWAIT_TRANSITION',
                    smach_ros.SimpleActionState('motion_planning/await_transition',
                        AwaitTransitionAction,
                        goal = AwaitTransitionGoal(state=INITIAL)),
                   {'succeeded':'penalized'})
        # Initial state machine description
        smach.StateMachine.add('INITIAL', initial_sm,
                               transitions={'penalized':'PENALIZED',
                                            'ready':'READY'})

        # ======================================================================
        # Create the Ready SMACH state machine
        ready_sm = smach.StateMachine(outcomes=['penalized', 'set', 'succeeded','aborted','preempted'])

        with ready_sm:
            # Ready state setup
            smach.StateMachine.add('SETUP',
                    smach_ros.SimpleActionState('motion_planning/setup',
                        SetupAction,
                        goal = SetupGoal(state=READY)),
                   {'succeeded':'set'})
        # Ready state machine description
        smach.StateMachine.add('READY', ready_sm,
                               transitions={'penalized':'PENALIZED',
                                            'set':'SET'})

        # ======================================================================
        # Create the Set SMACH state machine
        set_sm = smach.StateMachine(outcomes=['penalized', 'playing', 'succeeded','aborted','preempted'])
        with set_sm:
            # Set state setup
            smach.StateMachine.add('SETUP',
                    smach_ros.SimpleActionState('motion_planning/setup',
                        SetupAction,
                        goal = SetupGoal(state=SET)),
                   {'succeeded':'playing'})
        # Set state machine description
        smach.StateMachine.add('SET', set_sm,
                               transitions={'penalized':'PENALIZED',
                                            'playing':'PLAYING'})

        # ======================================================================
        # Create the Penalized SMACH state machine
        penalized_sm = smach.StateMachine(outcomes=['ready', 'set', 'playing', 'succeeded','aborted','preempted'])
        with penalized_sm:
            # Penalized state setup
            smach.StateMachine.add('SETUP',
                    smach_ros.SimpleActionState('motion_planning/setup',
                        SetupAction,
                        goal = SetupGoal(state=PENALIZED)),
                   {'succeeded':'AWAIT_TRANSITION'})
            # Await button or game controller transition
            smach.StateMachine.add('AWAIT_TRANSITION',
                    smach_ros.SimpleActionState('motion_planning/await_transition',
                        AwaitTransitionAction,
                        goal = AwaitTransitionGoal(state=PENALIZED)),
                   {'succeeded':'playing'})
        # Penalized state machine description
        smach.StateMachine.add('PENALIZED', penalized_sm,
                               transitions={'ready':'READY',
                                            'set':'SET',
                                            'playing':'PLAYING'})

        # ======================================================================
        # Create the Finished SMACH state machine
        finished_sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

        with finished_sm:
            # Initial state setup
            smach.StateMachine.add('SETUP',
                    smach_ros.SimpleActionState('motion_planning/setup',
                        SetupAction,
                        goal = SetupGoal(state=FINISHED)),
                   {'succeeded':'succeeded'})
        # Initial state machine description
        smach.StateMachine.add('FINISHED', finished_sm,
                               transitions={'succeeded':'succeeded'})

        # ======================================================================
        # Create the Playing SMACH state machine
        play_state_sm = smach.StateMachine(outcomes=['ready', 'penalized', 'succeeded','aborted','preempted'])

        with play_state_sm:
            # Playing state setup
            smach.StateMachine.add('SETUP',
                    smach_ros.SimpleActionState('motion_planning/setup',
                        SetupAction,
                        goal = SetupGoal(state=PLAYING)),
                   {'succeeded':'PLAY_CC'})

            # creating the concurrence state machine
            play_cc = smach.Concurrence(outcomes=['ready', 'penalized', 'succeeded'],
                             default_outcome='penalized',
                             # input_keys=['sm_input'],
                             # output_keys=['sm_output'],
                             child_termination_cb = playing_child_term_cb,
                             outcome_cb = playing_all_term_cb)

            with play_cc:

                smach.Concurrence.add('AWAIT_TRANSITION',
                    smach_ros.SimpleActionState('motion_planning/await_transition',
                        AwaitTransitionAction,
                        goal = AwaitTransitionGoal(state=PLAYING)),
                   {'ready':'ready',
                   'penalized':'penalized'})

                smach.Concurrence.add('PLAYING_SM', Playing_sm())

            smach.StateMachine.add('PLAY_CC', play_cc,
                               transitions={'penalized':'penalized',
                                            'ready':'ready'})

        # Playing state machine description
        smach.StateMachine.add('PLAYING', play_state_sm,
                               transitions={'succeeded':'FINISHED',
                                            'penalized':'PENALIZED',
                                            'ready':'READY'})
        # ======================================================================
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
