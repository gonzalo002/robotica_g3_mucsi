#!/usr/bin/python3
import rospy
import math
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from copy import deepcopy
import actionlib
import sys
sys.path.append("/home/laboratorio/ros_workspace/src/")
from proyecto_final.msg import RLAction
from proyecto_final.msg import IdCubos


class RLActionServer(object):
    # create messages that are used to publish feedback/result
    _feedback = RLAction().action_feedback
    _result = RLAction().action_result
    
    def __init__(self, name):
        self._action_name = name
        rospy.init_node('nodo_camara')
        action = RLAction()
        self._as = actionlib.SimpleActionServer(self._action_name, RLAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
    

    def execute_cb(self, goal):
        # Diccionario con las listas de los cubos por colores
        color_map = {0: [], 1: [], 2: [], 3: []}

        # Agrupar cubos por color
        for i, cubo in enumerate(goal.cubes_position):
            dict_cubo = {"PosicionLista": i, "Pose": cubo}
            color_map[cubo.color].append(dict_cubo.pose)

        # Ordenar las listas por distancia para cada color
        for color in color_map:
            color_map[color] = sorted(color_map[color], key=self._calcular_distancia)

        # Lista final de cubos seleccionados
        selected_cubes = []

        # Iterar por el orden de colores en goal.cubes_order
        for color in goal.cubes_order:
            # Verificar si hay cubos disponibles para ese color
            if color_map[color]:
                # Tomar el cubo más cercano (el primero de la lista)
                selected_cubes.append(color_map[color].pop(0))
        
        
    def _calcular_distancia(self, cubo):
        info_cubo:Pose = cubo["Pose"]
        x = info_cubo.position.x
        y = info_cubo.position.y
        return math.sqrt(x**2 + y**2)
            
            
if __name__ == "__main__":
    CubeTrackerActionServer("itsasne")