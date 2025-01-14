#! /usr/bin/env python
import rospy, actionlib

# Importación de mensajes ROS
import numpy as np
from proyecto_final.msg import FigurasAction, FigurasGoal, FigurasResult
from proyecto_final.msg import CubosAction, CubosGoal, CubosResult
from proyecto_final.msg import RLAction, RLGoal, RLResult
from proyecto_final.funciones_auxiliares import crear_mensaje
from proyecto_final.vision.generacion_figura import FigureGenerator



class MasterClient:
    """
    Clase que se encarga de realizar la comunicación con los servidores de acción
        @method obtain_figure: Obtiene la figura 3D
        @method obtain_cube_pose: Obtiene la posición de los cubos
        @method obtain_cube_order: Obtiene el orden de los cubos
        @method _secuencia_action_client: Método privado que realiza la secuencia de comunicación con el servidor de acción
    """
    def __init__(self, node_activate:bool=False):
        """
        Constructor de la clase
            @param node_activate: Activa el nodo de ROS
        """
        self.message = None
        self.message_type = None
        if node_activate:
            rospy.init_node('master_client_py')

    def obtain_figure(self, order:int=1) -> FigurasResult:
        """
        Método que se encarga de obtener la figura 3D
            @param order: Orden de la figura
            @return FigurasResult: Resultado de la figura 3D
        """
        name = "FigureMakerActionServer"
        action_client = actionlib.SimpleActionClient(name, FigurasAction)
        goal_msg = FigurasGoal(order=order)
        
        resultado:FigurasResult = self._secuencia_action_client(action_client, name, goal_msg)
        if len(resultado.figure_3d) != 0:
            return np.array(resultado.figure_3d).reshape(resultado.shape_3d)
        else:
            return np.array([[[]]])

    def obtain_cube_pose(self, goal:int=1) -> CubosResult:
        """
        Método que se encarga de obtener la posición de los cubos
            @param order: Orden de la figura
            @return CubosResult: Resultado de la posición de los cubos
        """
        name = "CubeTrackerActionServer"
        action_client = actionlib.SimpleActionClient(name, CubosAction)
        goal_msg = CubosGoal(order=goal)
        
        return self._secuencia_action_client(action_client, name, goal_msg)

    def obtain_cube_order(self, goal:tuple) -> RLResult:
        """
        Método que se encarga de obtener el orden de los cubos
            @param goal: Tupla que contiene la posición de los cubos y el orden de los cubos
            @return RLResult: Resultado del orden de los cubos
        """
        name = "RLActionServer"
        action_client = actionlib.SimpleActionClient(name, RLAction)
        goal_msg = RLGoal(cubes_position=goal[0], cubes_order=goal[1])
        
        return self._secuencia_action_client(action_client, name, goal_msg)

    
    def _secuencia_action_client(self, action_client, name, goal_msg):
        """
        Método privado que realiza la secuencia de comunicación con el servidor de acción
            @param action_client: Cliente de acción
            @param name: Nombre del servidor de acción
        """
        self.message_type = "INFO"
        self.message = crear_mensaje(f"Waiting for {name} server", self.message_type, "MasterClient")
        action_client.wait_for_server()

        self.message_type = "SUCCESS"
        self.message = crear_mensaje(f"Sending goal to {name}", self.message_type, "MasterClient")
        action_client.send_goal(goal_msg)
        
        self.message_type = "INFO"
        self.message = crear_mensaje(f"Waiting for result from {name}", self.message_type, "MasterClient")
        action_client.wait_for_result()
        
        self.message_type = "SUCCESS"
        self.message = crear_mensaje(f"Getting result from {name}", self.message_type, "MasterClient")

        return action_client.get_result()
