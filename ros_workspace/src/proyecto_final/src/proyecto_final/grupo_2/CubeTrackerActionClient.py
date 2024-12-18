#!/usr/bin/python3
import rospy
from math import pi
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
import actionlib
import cv2

from proyecto_final.vision.grupo_2.cube_tracker import CubeTracker
from proyecto_final.msg import CubosAction, CubosActionFeedback, CubosGoal, CubosActionResult, IdCubos
from proyecto_final.msg import IdCubos



class CubeTrackerActionClient(object):
    
    def __init__(self, node_activate:bool=False):
        if node_activate:
            rospy.init_node('cube_tracker_client_py')
        self.action_client = actionlib.SimpleActionClient("CubeTrackerActionServer", CubosAction)
    
    def send_goal(self, goal:int=1):
        goal_msg = CubosGoal(order=goal)
        #goal_msg.order = goal

        print("[INFO] Waiting for server")
        self.action_client.wait_for_server()

        # Returns future to goal handle; client runs feedback_callback after sending the goal
        print("[INFO] Sending goal")
        self.action_client.send_goal(goal_msg)
        
        print("[INFO] Waiting for result")
        self.action_client.wait_for_result()
        
        print("[INFO] Getting result")
        return self.action_client.get_result()

            
            
if __name__ == "__main__":
    cube_tracker_action = CubeTrackerActionClient(True)
    print(cube_tracker_action.send_goal())
    
