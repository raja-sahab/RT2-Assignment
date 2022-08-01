#! /usr/bin/env python
"""
.. module:: autonomous_driving
    :platform: Unix
    :synopsis: Python module for autonomous driving modality.

.. moduleauthor:: Raja Farooq Dilshad

Action client:
    /move_base

This module implements the autonomous driving modality, which allows the robot to reach a specified position in space.
The program sends a goal to the action server /move_base, receiving feedbacks and monitoring the status until the goal 
is reached or canceled. Thanks to the gmapping algorithm, the robot can create a map of the surrounding environment 
during its tours. The move_base node implements the action server to control the robot through the shortest path to 
reach the given position. The user can also require to cancel the current goal or to send a new one.

"""
# Run with: roslaunch final_assignment assignment.launch

### LIBRARIES ###
import rospy
import actionlib

### MESSAGES ###
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

### CODE ###
class autonomous_driving:
    """This class manages the autonomous driving mode, communicating with 
    the /move_base node through action server.

    """
    def __init__(self, user_interface):
        """This method initialises the autonomous_driving class.

        Args:
           user_interface(windows_organiser): class for printing on the user interface.

        """

        self.goal_counter = 0       # Stores the number of the goal.
        self.feedback_counter = 0   # Stores the number of feedback received.
        self.is_active = False      # True if it is reaching a goal.

        # Available commands is autonomous driving is active.
        self.title = R"""    c - to cancel goal
    n - to insert a new goal"""

        # Creates the MoveBaseAction client.
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # Stores the windows_organiser class as ui.
        self.ui = user_interface

    def active_cb(self):
        """This method executed when the communication starts.

        """
        self.goal_counter += 1   # Increments the goal counter.
        
        if self.ui != None:
            self.ui.win_info.addstr(2, 1, "Action Server is processing goal n "+str(self.goal_counter)+"... ")
            self.ui.win_info.refresh()


    def feedback_cb(self, feedback):
        """This method is executed when a feedback is received.

        """
        self.feedback_counter += 1   # Increments the feedback counter.
        
        if self.ui != None:
            # Prints on the info window.
            self.ui.win_info.addstr(3, 1, "Feedback for goal n "+str(self.goal_counter+1)+" received.          ")
            self.ui.win_info.addstr(3, 31 + self.feedback_counter % 10, ">")
            self.ui.win_info.refresh()


    def done_cb(self, status, result):
        """This method is executed when the communication ends.

        Args:
            status(int): status returned by the action server.

        """
        self.is_active = False   # The action client communication is not active.
        
        if self.ui == None:
            return
            
        self.ui.clear_modes()    # Clears the mode windows, c and n commands are no more available.
        
        # Prints on the info window the status returned by the action server communication.
        if status == 2:
            self.ui.win_info.addstr(4, 1, "Goal n "+str(self.goal_counter)+" received a cancel request.")
            self.ui.win_info.refresh()
            return

        if status == 3:
            self.ui.win_info.addstr(4, 1, "Goal n "+str(self.goal_counter)+" reached.                  ")
            self.ui.win_info.refresh()
            return

        if status == 4:
            self.ui.win_info.addstr(4, 1, "Goal n "+str(self.goal_counter)+" was aborted.              ")
            self.ui.win_info.refresh()
            return

        if status == 5:
            self.ui.win_info.addstr(4, 1, "Goal n "+str(self.goal_counter)+" has been rejected.        ")
            self.ui.win_info.refresh()
            return

        if status == 8:
            self.ui.win_info.addstr(4, 1, "Goal n "+str(self.goal_counter)+" received a cancel request.")
            self.ui.win_info.refresh()
            return

    def reach_goal(self, x, y):
        """This method sends a goal to the move base node, starting the action client communication.

        Args:
            x(float): x goal position of the robot
            y(float): y goal position of the robot

        """
        self.is_active = True   # Processing the goal.

        # Waits until the action server has started.
        self.client.wait_for_server()
        
        if self.ui != None:
            # Prints the new commands available on the modes window.
            self.ui.win_modes.addstr(3, 0, self.title)
            self.ui.win_modes.refresh()

        # Creates a goal to send to the action server.
        goal = MoveBaseGoal()

        # Fills the elements of the goal.
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.orientation.w = 1
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
   
        # Sends the goal to the action server.
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

    def cancel_goal(self):
        """This method sends a cancel request to the move_base server.

        """
        
        self.is_active = False
        self.client.cancel_goal()
        if self.ui != None:
            self.ui.clear_modes()
