#!/usr/bin/env python
##
# @file      adhoc.py
# @brief     Adhoc agent main state machine
# @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
# @date      2015-06-05
# @copyright (MIT) 2015 Edinferno

import roslib; roslib.load_manifest('agent')
import rospy
import smach
import smach_ros
from actionlib import *
from actionlib_msgs.msg import *
from agent import *
from std_msgs.msg import Bool
from motion_planning_msgs.msg import SetupAction, SetupGoal
from motion_planning_msgs.msg import TransitionAction, TransitionGoal

def transition_cb(userdata, status, result):
    if status == GoalStatus.SUCCEEDED:
        userdata.outcome = result.outcome
        return result.outcome

# gets called when ANY child state terminates
def transition_child_term_cb(outcome_map):
    if outcome_map['TRANSITION'] == 'stand_up':
        return True
    elif outcome_map['TRANSITION'] == 'initial':
        return True
    elif outcome_map['TRANSITION'] == 'ready':
        return True
    elif outcome_map['TRANSITION'] == 'set':
        return True
    elif outcome_map['TRANSITION'] == 'penalized':
        return True
    elif outcome_map['TRANSITION'] == 'playing':
        return True
    elif outcome_map['TRANSITION'] == 'finished':
        return True

    return False

# gets called when ALL child states are terminated
def transition_all_term_cb(outcome_map):
    print " Trans: " + outcome_map['TRANSITION']
    if outcome_map['TRANSITION'] == 'stand_up':
        return 'stand_up'
    elif outcome_map['TRANSITION'] == 'initial':
        return 'initial'
    elif outcome_map['TRANSITION'] == 'ready':
        return 'ready'
    elif outcome_map['TRANSITION'] == 'set':
        return 'set'
    elif outcome_map['TRANSITION'] == 'penalized':
        return 'penalized'
    elif outcome_map['TRANSITION'] == 'playing':
        return 'playing'
    elif outcome_map['TRANSITION'] == 'finished':
        return 'finished'
    else:
        return 'aborted'

def main():
    rospy.init_node('agent_game_smach')

    pub = rospy.Publisher('/smach/online', std_msgs.msg.Bool, queue_size=1, latch=True)

    pub.publish(std_msgs.msg.Bool(True))
    # GAME STATES
    INITIAL = 0
    READY=1
    SET=2
    PLAYING=3
    FINISHED=4
    PENALIZED=5

    # Create the main game SMACH state machine
    game_sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    with game_sm:
        # ======================================================================
        # Create the Initial SMACH state machine
        initial_state_sm = smach.StateMachine(outcomes=['initial','ready','set','penalized','playing','finished','aborted','preempted'])

        with initial_state_sm:
            # Initial state setup
            smach.StateMachine.add('SETUP',
                    smach_ros.SimpleActionState('motion_planning/setup',
                        SetupAction,
                        goal = SetupGoal(state=INITIAL)),
                   {'succeeded':'INITIAL_CC'})

            # creating the concurrence state machine
            initial_cc = smach.Concurrence(outcomes=['stand_up','initial','ready','set','penalized','playing','finished', 'aborted', 'preempted'],
                             default_outcome='aborted',
                             # input_keys=['sm_input'],
                             # output_keys=['sm_output'],
                             child_termination_cb = transition_child_term_cb,
                             outcome_cb = transition_all_term_cb)

            with initial_cc:

                smach.Concurrence.add('TRANSITION',
                    smach_ros.SimpleActionState('motion_planning/transition',
                        TransitionAction,
                        result_cb=transition_cb,
                        outcomes = ['stand_up','initial','ready','set','penalized','playing','finished','aborted', 'preempted'],
                        goal = TransitionGoal(state=INITIAL),
                        output_keys=['outcome']))

                smach.Concurrence.add('INITIAL_SM', InitialSM())

            smach.StateMachine.add('INITIAL_CC', initial_cc,
                                   transitions={'stand_up':'INITIAL_CC'})

        # Initial state machine description
        smach.StateMachine.add('INITIAL', initial_state_sm,
                               transitions={'initial':'INITIAL',
                                            'ready':'READY',
                                            'set':'SET',
                                            'penalized':'PENALIZED',
                                            'playing':'PLAYING',
                                            'finished':'FINISHED'})

        # ======================================================================
        # Create the Ready SMACH state machine
        ready_state_sm = smach.StateMachine(outcomes=['initial','ready','set','penalized','playing','finished','aborted','preempted'])

        with ready_state_sm:
            # Ready state setup
            smach.StateMachine.add('SETUP',
                    smach_ros.SimpleActionState('motion_planning/setup',
                        SetupAction,
                        goal = SetupGoal(state=READY)),
                   {'succeeded':'READY_CC'})

            # creating the concurrence state machine
            ready_cc = smach.Concurrence(outcomes=['stand_up','initial','ready','set','penalized','playing','finished', 'aborted'],
                             default_outcome='aborted',
                             # input_keys=['sm_input'],
                             # output_keys=['sm_output'],
                             child_termination_cb = transition_child_term_cb,
                             outcome_cb = transition_all_term_cb)

            with ready_cc:

                smach.Concurrence.add('TRANSITION',
                    smach_ros.SimpleActionState('motion_planning/transition',
                        TransitionAction,
                        result_cb=transition_cb,
                        outcomes = ['stand_up','initial','ready','set','penalized','playing','finished','aborted'],
                        goal = TransitionGoal(state=READY),
                        output_keys=['outcome']))

                smach.Concurrence.add('READY_SM', ReadySM())

            smach.StateMachine.add('READY_CC', ready_cc,
                                   transitions={'stand_up':'READY_CC'})

        # Ready state machine description
        smach.StateMachine.add('READY', ready_state_sm,
                               transitions={'initial':'INITIAL',
                                            'ready':'READY',
                                            'set':'SET',
                                            'penalized':'PENALIZED',
                                            'playing':'PLAYING',
                                            'finished':'FINISHED'})

        # ======================================================================
        # Create the Set SMACH state machine
        set_state_sm = smach.StateMachine(outcomes=['initial','ready','set','penalized','playing','finished','aborted','preempted'])

        with set_state_sm:
            # Set state setup
            smach.StateMachine.add('SETUP',
                    smach_ros.SimpleActionState('motion_planning/setup',
                        SetupAction,
                        goal = SetupGoal(state=SET)),
                   {'succeeded':'SET_CC'})

            # creating the concurrence state machine
            set_cc = smach.Concurrence(outcomes=['stand_up','initial','ready','set','penalized','playing','finished', 'aborted'],
                             default_outcome='aborted',
                             # input_keys=['sm_input'],
                             # output_keys=['sm_output'],
                             child_termination_cb = transition_child_term_cb,
                             outcome_cb = transition_all_term_cb)

            with set_cc:

                smach.Concurrence.add('TRANSITION',
                    smach_ros.SimpleActionState('motion_planning/transition',
                        TransitionAction,
                        result_cb=transition_cb,
                        outcomes = ['stand_up','initial','ready','set','penalized','playing','finished','aborted'],
                        goal = TransitionGoal(state=SET),
                        output_keys=['outcome']))

                smach.Concurrence.add('SET_SM', SetSM())

            smach.StateMachine.add('SET_CC', set_cc,
                                   transitions={'stand_up':'SET_CC'})

        # Set state machine description
        smach.StateMachine.add('SET', set_state_sm,
                               transitions={'initial':'INITIAL',
                                            'ready':'READY',
                                            'set':'SET',
                                            'penalized':'PENALIZED',
                                            'playing':'PLAYING',
                                            'finished':'FINISHED'})

        # ======================================================================
        # Create the Penalized SMACH state machine
        penalized_state_sm = smach.StateMachine(outcomes=['initial','ready','set','penalized','playing','finished','aborted','preempted'])

        with penalized_state_sm:
            # Penalized state setup
            smach.StateMachine.add('SETUP',
                    smach_ros.SimpleActionState('motion_planning/setup',
                        SetupAction,
                        goal = SetupGoal(state=PENALIZED)),
                   {'succeeded':'PENALIZED_CC'})

            # creating the concurrence state machine
            penalized_cc = smach.Concurrence(outcomes=['stand_up','initial','ready','set','penalized','playing','finished', 'aborted'],
                             default_outcome='aborted',
                             # input_keys=['sm_input'],
                             # output_keys=['sm_output'],
                             child_termination_cb = transition_child_term_cb,
                             outcome_cb = transition_all_term_cb)

            with penalized_cc:

                smach.Concurrence.add('TRANSITION',
                    smach_ros.SimpleActionState('motion_planning/transition',
                        TransitionAction,
                        result_cb=transition_cb,
                        outcomes = ['stand_up','initial','ready','set','penalized','playing','finished','aborted'],
                        goal = TransitionGoal(state=PENALIZED),
                        output_keys=['outcome']))

                smach.Concurrence.add('PENALIZED_SM', PenalizedSM())

            smach.StateMachine.add('PENALIZED_CC', penalized_cc,
                                   transitions={'stand_up':'PENALIZED_CC'})

        # Penalized state machine description
        smach.StateMachine.add('PENALIZED', penalized_state_sm,
                               transitions={'initial':'INITIAL',
                                            'ready':'READY',
                                            'set':'SET',
                                            'penalized':'PENALIZED',
                                            'playing':'PLAYING',
                                            'finished':'FINISHED'})

        # ======================================================================
        # Create the Finished SMACH state machine
        finished_state_sm = smach.StateMachine(outcomes=['initial','ready','set','penalized','playing','finished','aborted','preempted'])

        with finished_state_sm:
            # Finished state setup
            smach.StateMachine.add('SETUP',
                    smach_ros.SimpleActionState('motion_planning/setup',
                        SetupAction,
                        goal = SetupGoal(state=FINISHED)),
                   {'succeeded':'FINISHED_CC'})

            # creating the concurrence state machine
            finished_cc = smach.Concurrence(outcomes=['stand_up','initial','ready','set','penalized','playing','finished','aborted'],
                             default_outcome='aborted',
                             # input_keys=['sm_input'],
                             # output_keys=['sm_output'],
                             child_termination_cb = transition_child_term_cb,
                             outcome_cb = transition_all_term_cb)

            with finished_cc:

                smach.Concurrence.add('TRANSITION',
                    smach_ros.SimpleActionState('motion_planning/transition',
                        TransitionAction,
                        result_cb=transition_cb,
                        outcomes = ['stand_up','initial','ready','set','penalized','playing','finished','aborted'],
                        goal = TransitionGoal(state=FINISHED),
                        output_keys=['outcome']))

                smach.Concurrence.add('FINISHED_SM', FinishedSM())

            smach.StateMachine.add('FINISHED_CC', finished_cc,
                                   transitions={'stand_up':'FINISHED_CC'})

        # Finished state machine description
        smach.StateMachine.add('FINISHED', finished_state_sm,
                               transitions={'initial':'INITIAL',
                                            'ready':'READY',
                                            'set':'SET',
                                            'penalized':'PENALIZED',
                                            'playing':'PLAYING',
                                            'finished':'FINISHED'})
        # ======================================================================
        # Create the Playing SMACH state machine
        play_state_sm = smach.StateMachine(outcomes=['initial','ready','set','penalized','playing','finished','aborted','preempted'])

        with play_state_sm:
            # Playing state setup
            smach.StateMachine.add('SETUP',
                    smach_ros.SimpleActionState('motion_planning/setup',
                        SetupAction,
                        goal = SetupGoal(state=PLAYING)),
                   {'succeeded':'PLAY_CC'})

            # creating the concurrence state machine
            play_cc = smach.Concurrence(outcomes=['stand_up','initial','ready','set','penalized','playing','finished', 'aborted', 'preempted'],
                             default_outcome='aborted',
                             # input_keys=['sm_input'],
                             # output_keys=['sm_output'],
                             child_termination_cb = transition_child_term_cb,
                             outcome_cb = transition_all_term_cb)

            with play_cc:

                smach.Concurrence.add('TRANSITION',
                    smach_ros.SimpleActionState('motion_planning/transition',
                        TransitionAction,
                        result_cb=transition_cb,
                        outcomes = ['stand_up','initial','ready','set','penalized','playing','finished','aborted', 'preempted'],
                        goal = TransitionGoal(state=PLAYING),
                        output_keys=['outcome']))

                smach.Concurrence.add('PLAYING_SM', PlayingSM())

            smach.StateMachine.add('PLAY_CC', play_cc,
                                   transitions={'stand_up':'PLAY_CC'})

        # Playing state machine description
        smach.StateMachine.add('PLAYING', play_state_sm,
                               transitions={'initial':'INITIAL',
                                            'ready':'READY',
                                            'set':'SET',
                                            'penalized':'PENALIZED',
                                            'playing':'PLAYING',
                                            'finished':'FINISHED'})
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
