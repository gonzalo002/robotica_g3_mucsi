#! /usr/bin/env python
import rospy, actionlib
# Importación de mensajes ROS
import numpy as np
from proyecto_final.msg import FigurasAction, FigurasGoal, FigurasResult
from proyecto_final.msg import CubosAction, CubosGoal, CubosResult
from proyecto_final.msg import RLAction, RLGoal, RLResult
from proyecto_final.funciones_auxiliares import crear_mensaje


class MasterClient:
    def __init__(self, node_activate:bool=False):
        if node_activate:
            rospy.init_node('master_client_py')

    def obtain_figure(self, order:int=1)->RLResult:
        name = "FigureMakerActionServer"
        action_client = actionlib.SimpleActionClient(name, FigurasAction)
        goal_msg = FigurasGoal(order=order)
        
        resultado:FigurasResult = self._secuencia_action_client(action_client, name, goal_msg)
        if len(resultado.figure_3d) != 0:
            return np.array(resultado.figure_3d).reshape(resultado.shape_3d)
        else:
            return np.array([[[]]])

    def obtain_cube_pose(self, goal:int=1):
        name = "CubeTrackerActionServer"
        action_client = actionlib.SimpleActionClient(name, CubosAction)
        goal_msg = CubosGoal(order=goal)
        
        return self._secuencia_action_client(action_client, name, goal_msg)

    def obtain_cube_order(self, goal:int=1):
        name = "RLActionServer"
        action_client = actionlib.SimpleActionClient(name, RLAction)
        goal_msg = RLGoal(order=goal)
        
        return self._secuencia_action_client(action_client, name, goal_msg)

    
    def _secuencia_action_client(self, action_client, name, goal_msg):
        crear_mensaje(f"Waiting for {name} server", "INFO", "MasterClient")
        action_client.wait_for_server()

        crear_mensaje(f"Sending goal to {name}", "SUCCESS", "MasterClient")
        action_client.send_goal(goal_msg)
        
        crear_mensaje(f"Waiting for result from {name}", "INFO", "MasterClient")
        action_client.wait_for_result()
        
        crear_mensaje(f"Getting result from {name}", "SUCCESS", "MasterClient")
        return action_client.get_result()
        
if __name__ == '__main__':
    master = MasterClient(True)
    print(master.obtain_cube_pose())
