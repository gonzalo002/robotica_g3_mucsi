import rospy
import math
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from copy import deepcopy
import actionlib
from proyecto_final.msg import RLAction
import sys
sys.path.append("/home/laboratorio/ros_workspace/src/proyecto_final/scripts/vision")
from src.proyecto_final.msg import IdCubos


class CubeTrackerActionServer(object):
    # create messages that are used to publish feedback/result
    _feedback = RLAction().action_feedback
    _result = RLAction().action_result
    
    def __init__(self, name):
        self._action_name = name
        rospy.init_node('nodo_camara')
        action = RLAction()
        self._as = actionlib.SimpleActionServer(self._action_name, RLAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
    

    def execute_cb(self, goal:RLAction().action_goal):
        
        # Realizar listas por colores
        red_cube = []
        blue_cube = []
        green_cube = []
        yellow_cube = []

        for i, cubo in enumerate(goal.cubes_position):
            dict_cubo = {"PosicionLista": i, "Pose": cubo}
            if cubo.color == 0:
                red_cube.append(dict_cubo.pose)
            elif cubo.color == 1:
                blue_cube.append(dict_cubo.pose)
            elif cubo.color == 2:
                green_cube.append(dict_cubo.pose)
            elif cubo.color == 3:
                yellow_cube.append(dict_cubo.pose)

        # Ordenar las listas por distancia
        red_cube = sorted(red_cube, key=self._calcular_distancia)
        blue_cube = sorted(blue_cube, key=self._calcular_distancia)
        green_cube = sorted(green_cube, key=self._calcular_distancia)
        yellow_cube = sorted(yellow_cube, key=self._calcular_distancia)

        color_map = {
            0: red_cube,
            1: blue_cube,
            2: green_cube,
            3: yellow_cube
        }

        # Lista final de cubos seleccionados
        selected_cubes = []

        # Iterar por el orden de colores
        for color in goal.cubes_order:
            # Verificar si quedan cubos de ese color
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