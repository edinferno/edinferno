# @file      agent.py
# @brief     Agent game state machine
# @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
# @date      2015-06-05
# @copyright (MIT) 2015 Edinferno

# import roslib; roslib.load_manifest('agent')
import rospy
import smach
import smach_ros

from actionlib import *
from actionlib_msgs.msg import *
# from agent_msgs.msg import TestAction, TestGoal

# Create a trivial action server
# class TestServer:
#     def __init__(self,name):
#         self._sas = SimpleActionServer(name,
#                 TestAction,
#                 execute_cb=self.execute_cb)

#     def execute_cb(self, msg):
#         if msg.goal == 0:
#             self._sas.set_succeeded()
#         elif msg.goal == 1:
#             self._sas.set_aborted()
#         elif msg.goal == 2:
#             self._sas.set_preempted()
