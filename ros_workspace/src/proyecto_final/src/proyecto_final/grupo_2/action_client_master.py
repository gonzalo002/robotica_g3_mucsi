#! /usr/bin/env python
import sys, rospy, actionlib
# Importación de mensajes ROS
from proyecto_final.msg import FigurasAction, FigurasGoal, FigurasResult
from proyecto_final.msg import CubosAction, CubosGoal, CubosResult
from proyecto_final.msg import RLAction, RLGoal, RLResult

# --- CÓDIGO COLORES ---
c = {
    "ERROR": "\033[31m",      # Rojo
    "SUCCESS": "\033[32m",     # Verde
    "WARN": "\033[33m",  # Amarillo
    "RESET": "\033[0m"       # Restablecer al color predeterminado
}

class MasterClient:
    def __init__(self, node_activate:bool=False):
        if node_activate:
            rospy.init_node('master_client_py')

    def obtain_figure(self, order:int=1)->RLResult:
        name = "FigurasActionServer"
        action_client = actionlib.SimpleActionClient(name, FigurasAction)
        goal_msg = FigurasGoal(order=order)

        return self._secuencia_action_client(action_client, name, goal_msg)

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
        print(f"[INFO] Waiting for {name} server")
        action_client.wait_for_server()

        print(f"{c['SUCCESS']}[SUCCESS] Sending goal to {name}{c['RESET']}")
        action_client.send_goal(goal_msg)
        
        print(f"[INFO] Waiting for result from {name}")
        action_client.wait_for_result()
        
        print(f"{c['SUCCESS']}[SUCCESS] Getting result from {name}{c['RESET']}")
        return action_client.get_result()
        
if __name__ == '__main__':
    master = MasterClient(True)
    print(master.obtain_cube_pose())
