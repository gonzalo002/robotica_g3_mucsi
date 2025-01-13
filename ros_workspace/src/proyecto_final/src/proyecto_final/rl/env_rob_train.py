#!/usr/bin/python3

# Librerías estándar de Python
import os
from copy import deepcopy
from typing import List, Dict, Tuple, Any, Union
from math import pi

# Librerías de terceros
import gymnasium as gym
import numpy as np
from stable_baselines3.common.utils import set_random_seed
from tf.transformations import quaternion_from_euler
from proyecto_final.msg import IdCubos

# Librerias de ROS
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState

# Importaciones locales 
from proyecto_final.control_robot import ControlRobot


class ROSEnv(gym.Env):
    """
    Clase que define el entorno de entrenamiento para el robot.
        @method __init__ - Método constructor
        @method _get_obs - Método para obtener la observación
        @method _get_info - Método para obtener la información adicional
        @method _sample_new_figure - Método para muestrear una nueva figura
        @method _sample_new_cube_value - Método para muestrear un nuevo valor de cubo
        @method _añadir_cubos_a_planificacion - Método para añadir cubos a la planificación
        @method _replace_unknow_colors - Método para reemplazar los colores desconocidos
        @method _empty_workspace - Método para vaciar el espacio de trabajo
        @method _discard_cube - Método para descartar un cubo
        @method step - Método para realizar un paso en el entorno de entrenamiento
        @method reset - Método para reiniciar el entorno de entrenamiento
    """
    def __init__(self, num_cubos_max: int, verbose:bool = True) -> None:
        """
        Método constructor de la clase ROSEnv.
            @param num_cubos_max (int) - Número máximo de cubos
            @param verbose (bool) - Indica si se desea mostrar información en pantalla
        """
        super(ROSEnv, self).__init__()
        self.control_robot = ControlRobot("robot", train_env=True)

        self.abs_path = '/'.join(os.path.dirname(os.path.abspath(__file__)).split('/')[:os.path.dirname(os.path.abspath(__file__)).split('/').index('proyecto_final')+1])

        self.cube_limit = 25 # Se pone un limite de 25 cubos, de esta forma un mismo agente puede funcionar con hasta 25 cubos
        self.n_cubos = num_cubos_max
        self.verbose = verbose

        self.action_space = gym.spaces.Discrete(self.cube_limit)

        self.observation_space = gym.spaces.Box( # Espacio de observación (25 cubos + color a buscar)
            low=np.array([[-1]* 8]  * (self.cube_limit+1)),
            high=np.array([[-1]* 8] * (self.cube_limit+1)),
            dtype=np.float64
        )

        # Variables de Simulación
        self.cube_size = 0.025 # Tamaño del cubo
        self.cube_separation = 0.02 # Separación entre cubos
        self.needed_cubes = {i: 0 for i in range(4)} # Cubos necesarios
        self.available_cubes = [0]*4 # Cubos disponibles
        self.all_colors_found = False # Indica si se han encontrado todos los colores   
        self.robot_workspace_values = {"max_x": 0.25, "min_x": -0.25, "max_y": 0.4, "min_y": 0.17, "max_alpha": pi/4, "min_alpha": -pi/4}

        self.j_link_1:JointState = self.control_robot.read_from_yaml(f'{self.abs_path}/data/trayectorias/master_positions', 'J_LINK_1')
        self.j_home:JointState = self.control_robot.read_from_yaml(f'{self.abs_path}/data/trayectorias/master_positions', 'J_HOME')
        self.p_discard_origin:Pose = self.control_robot.read_from_yaml(f'{self.abs_path}/data/trayectorias/master_positions', 'P_DISCARD_ORIGIN')
        self.p_discard_origin_2:Pose = self.control_robot.read_from_yaml(f'{self.abs_path}/data/trayectorias/master_positions', 'P_DISCARD_2_ORIGIN')       
        self.p_figure_origin:Pose = self.control_robot.read_from_yaml(f'{self.abs_path}/data/trayectorias/master_positions', 'P_MATRIX_ORIGIN')
        
        margen = 1.8
        # Suponemos que la base de la figura es de 2x2 de maxima
        self.workspace_range = {'x_max': self.p_figure_origin.position.x+((self.cube_size + self.cube_separation)*(2+margen)), 
                                'x_min': self.p_figure_origin.position.x-((self.cube_size + self.cube_separation)*margen), 
                                'y_max': self.p_figure_origin.position.y+((self.cube_size + self.cube_separation)*(2+margen)), 
                                'y_min': self.p_figure_origin.position.y-((self.cube_size + self.cube_separation)*margen)}

        # Variables de Entorno
        self.cubos:List[IdCubos] = [] # Lista de cubos
        self.pseudo_rands_cubos = [] # Lista de valores pseudo-aleatorios para los cubos
        self.colors_found = False # Indica si se han encontrado todos los colores
        self.figure_order = [] # Orden de los colores de la figura
        self.cubos_recogidos = 0 # Número de cubos recogidos
        self.discarded_cubes:list = [0, 0, 0, 0] # RGBY
        self.action_buffer = set() # Set para las acciones realizadas
        self.orden_cubos = [] # Orden de los cubos recogidos
        self.total_time = 0  # Total time taken for all actions
        self.failed_attempts = 0  # Number of failed attempts

        # Variables de Mensajes
        self.n_steps = 0 # Número de pasos
        self.n_trials = 0 # Número de intentos

        # Variables de Control
        self.observation = np.array([[-1]* 8]  * (self.cube_limit+1)) # Cubos + Color de la Figura a coger
        self.reward = 0.0 # Recompensa
        self.info = {} # Información adicional
        self.done = False # Indica si el episodio ha terminado
        self.failed = False # Indica si ha fallado

        # Condiciones Iniciales
        self.control_robot.reset_planning_scene()
        self.control_robot.move_group.set_joint_value_target(self.j_home)

    def _get_obs(self):
        """
        Obtener la observación del entorno.
        """
        pose:IdCubos
        for i, cubo in enumerate(self.cubos):
            observation = np.array([-1.0] * 8)
            pose:Pose = cubo.pose
            observation[0] = pose.position.x
            observation[1] = pose.position.y
            observation[2] = pose.position.z
            observation[3] = pose.orientation.x
            observation[4] = pose.orientation.y
            observation[5] = pose.orientation.z
            observation[6] = pose.orientation.w
            observation[7] = cubo.color

            self.observation[i] = observation

        if self.cubos_recogidos < self.n_cubos:
            self.observation[-1, -1] = self.figure_order[self.cubos_recogidos]
    
    def _get_info(self):
        """
        Obtener la información adicional del entorno.
        """
        self.info = {"orden_cubos": self.orden_cubos}
        
    def _sample_new_figure(self):
        """
        Generar una nueva figura.
        """
        for _ in range(self.n_cubos):
            self.figure_order.append(np.random.randint(0,4))
            if np.random.random() > 0.85:
                self.figure_order[-1] = 4

        # self.figure_order = np.random.randint(0, 4, size=self.num_cubos_max)
        unique, counts = np.unique(self.figure_order, return_counts=True)
        
        # Actualizar el diccionario con los valores presentes en figure_order
        self.needed_cubes.update(dict(zip(unique, counts)))

    def _sample_new_cube_value(self, max_x: float, min_x: float, 
                            max_y: float, min_y: float, 
                            max_alpha: float, min_alpha: float) -> None:
        """
        Genera un nuevo valor pseudo-aleatorio para un cubo.
            @param max_x (float) - Máximo valor en X
            @param min_x (float) - Mínimo valor en X
            @param max_y (float) - Máximo valor en Y
            @param min_y (float) - Mínimo valor en Y
            @param max_alpha (float) - Máximo valor en Alpha
            @param min_alpha (float) - Mínimo valor en Alpha
        """
        
        # Verificar que los valores máximos sean mayores que los mínimos
        if max_x < min_x or max_y < min_y or max_alpha < min_alpha:
            raise ValueError("max values must be greater than min values")
        
        success = False # Indica si se ha generado un valor válido
        
        while not success:
            # x y oz
            pseudo_rands = np.random.rand(4)
            pseudo_rands[0] = np.interp(pseudo_rands[0], [0, 1], [min_x, max_x])
            pseudo_rands[1] = np.interp(pseudo_rands[1], [0, 1], [min_y, max_y])

            if (pseudo_rands[0] ** 2) + (pseudo_rands[1] ** 2) <= 0.48:
                if self.pseudo_rands_cubos != []:
                     # Verificar que el nuevo cubo esté a una distancia mínima de 0.03 metros de los cubos existentes
                    success = True
                    for pose in self.pseudo_rands_cubos:
                        dist = np.sqrt((pose[0] - pseudo_rands[0]) ** 2 + (pose[1] - pseudo_rands[1]) ** 2)
                        if dist < 0.05:
                            success = False
                            break  # Si la distancia es menor a 0.03, volver a intentar
                else:
                    success = True  
        
        pseudo_rands[2] = np.interp(pseudo_rands[2], [0, 1], [min_alpha, max_alpha]) # Oz
        pseudo_rands[3] = np.random.randint(0, 4) # Color
        
        # Actualizar los valores de los cubos necesarios y disponibles
        cube_color = int(pseudo_rands[3])
        if not self.all_colors_found: 
            if self.needed_cubes[cube_color] == self.available_cubes[cube_color]:
                cube_found = False
                for i in range(4):
                    if self.needed_cubes[i] > self.available_cubes[i]:
                        pseudo_rands[3] = i
                        self.available_cubes[i] += 1
                        cube_found = True
                        break
                if not cube_found:
                    self.available_cubes[cube_color] += 1
                    self.all_colors_found = True
            else:
                self.available_cubes[cube_color] += 1
        else:
            self.available_cubes[cube_color] += 1
                
        if pseudo_rands[0] > 0:
            pseudo_rands[2] -= pi/2
        
        self.pseudo_rands_cubos.append(deepcopy(pseudo_rands))

    def _añadir_cubos_a_planificacion(self, cubos: List[IdCubos]) -> None:
        """
        Añadir cubos a la planificación.
            @param cubos: List[IdCubos], lista de cubos.
        """
        for i, pseudo_rands in enumerate(cubos):
            cubo:IdCubos = IdCubos()
            cubo.pose = Pose(position=Point(x=pseudo_rands[0], y=pseudo_rands[1], z=0.0125), 
                             orientation=Quaternion(*quaternion_from_euler(pi, 0, pseudo_rands[2], 'sxyz')))
            cubo.color = int(pseudo_rands[3])
            cubo.id = i

            self.cubos.append(cubo)

            self.control_robot.add_box_obstacle(f"Cubo_{cubo.id}", cubo.pose, (0.025, 0.025, 0.025))
    
    def _replace_unknow_colors(self) -> None:
        """
        Reeplazar los colores desconocidos.
        """
        for i, color in enumerate(self.figure_order):
            if color == 4:
                for j in range(4):
                    if self.needed_cubes[j] < self.available_cubes[j]:
                        self.figure_order[i]
                        self.needed_cubes[j] += 1
    
    def _empty_workspace(self) -> None:
        """
        Libera el espacio de trabajo del robot.
        """
        x_min = self.workspace_range['x_min']
        y_min = self.workspace_range['y_min']        
        x_max = self.workspace_range['x_max']        
        y_max = self.workspace_range['y_max']   
        
        self.control_robot.move_jointstates(self.j_link_1)     
                
        cube:IdCubos
        for cube in self.cubos: # Se recorren los cubos
            pose:Pose = deepcopy(cube.pose)
            color = cube.color
            if pose.position.x > x_min and pose.position.x < x_max: # Si la posición X está en el rango
                if pose.position.y > y_min and pose.position.y < y_max: # Si la posición Y está en el rango
                    self._discard_cube(cube_id=cube.id, matrix_position=[0, color, self.discarded_cubes[color]])
                    self.discarded_cubes[color] += 1
        

    def _discard_cube(self, cube_id:int = None, matrix_position:list = [0,0,0]) -> None:
        ''' 
        Método para llevar un cubo a la zona de descartes.
            @param cube_id: int, identificador del cubo.
            @param matrix_position: list, posición en la matriz.
        '''

        self.control_robot.scene.remove_world_object(f'Cubo_{cube_id}')

        # Se determina la pose inicial dependiendo si el cubo forma parte de una figura.
        if matrix_position[1] < 2:
                pose: Pose = deepcopy(self.p_discard_origin)  # Pose inicial para desechar el cubo.

        elif matrix_position[1] < 4:
            matrix_position[1] -= 2
            pose: Pose = deepcopy(self.p_discard_origin_2)  # Pose inicial para desechar el cubo.

        # Ajuste en la posición X, Y y Z basados en la matriz de posición.
        pose.position.x += ((self.cube_size + self.cube_separation*2) * matrix_position[0])
        pose.position.y += ((self.cube_size + self.cube_separation*2) * matrix_position[1])
        pose.position.z = ((self.cube_size) * matrix_position[2])+0.0125

        self.control_robot.add_box_obstacle(f"Cubo_{cube_id}", pose, (0.025, 0.025, 0.025))
        self.cubos[cube_id].pose = pose

    def step(self, action: List[int]) -> Tuple[np.ndarray, float, bool, bool, dict]:
        """
        Realiza un paso en el entorno.
            @param action: List[int], acción a realizar.
            @return: Tuple[np.ndarray, float, bool, bool, dict], observación, recompensa, si el episodio ha terminado, si la acción ha sido truncada y la información adicional.
        """

        if self.failed_attempts >= 10:
            self.failed = True
            print(f'\033[31m\nTrial {self.n_trials} - Step {self.n_steps} - \
                          Fallado con Accion {self.orden_cubos}\033[0m')
            self.reward -= (self.n_cubos-self.cubos_recogidos)*-10.0
            return self.observation, self.reward, self.done, self.failed, self.info

        if action >= len(self.cubos): # Si la acción no es válida
            self.n_steps += 1 # Incrementa el número de pasos
            self.reward -= 20.0 # Penaliza la acción con -5
            return self.observation, self.reward, self.done, self.failed, self.info 

        if action in self.taken_actions: # Si la acción ya ha sido tomada
            self.n_steps += 1 # Incrementa el número de pasos
            self.reward -= 20.0 # Penaliza la acción con -5
            return self.observation, self.reward, self.done, self.failed, self.info 
        
        figure_color = self.observation[-1,-1]
        selected_cube:IdCubos = deepcopy(self.cubos[action]) # Selecciona el cubo a recoger

        if figure_color == -1: # Si no hay color de figura
            self.cubos_recogidos += 1
            return self.observation, self.reward, self.done, self.failed, self.info

        if selected_cube.color != figure_color: # Si el color del cubo no coincide con el color de la figura
            self.reward -= 10.0 # Penaliza la acción con -5
            self.n_steps += 1 # Incrementa el número de pasos
            return self.observation, self.reward, self.done, self.failed, self.info 

        cube_pose:Pose = selected_cube.pose # Obtiene la pose del cubo seleccionado
        cube_pose.position.z = 0.237

        prev_pose:Pose = deepcopy(cube_pose) # Copia la pose del cubo seleccionado
        prev_pose.position.z = 0.237 # Ajusta la altura del cubo para recogerlo

        self.control_robot.scene.remove_world_object(f'Cubo_{selected_cube.id}')
        
        self.control_robot.move_group.set_joint_value_target(self.j_link_1) # Se mueve a la posición deseada

        trayectory_tuple = self.control_robot.plan_pose(prev_pose)

        if trayectory_tuple[0] != True:  # Si la trayectoria no es válida
            self.control_robot.add_box_obstacle(f'Cubo_{selected_cube.id}', selected_cube.pose, [0.025]*3)
            if trayectory_tuple[3].val == -1: # Si la colisión es con el robot
                self.failed_attempts += 1
                self.n_steps += 1
                self.reward -= 10.0
                if self.verbose:
                    print('\033[31mColisión con Escena\033[0m')
                return self.observation, self.reward, self.done, self.failed, self.info
            
            else:
                self.failed = True
                self.n_steps += 1
                if self.verbose:
                    print(f'\033[31m\nTrial {self.n_trials} - Step {self.n_steps} - \
                          Fallado con Accion {self.orden_cubos} - Cubo Fallado {selected_cube.id}\033[0m')
                    self.reward -= (self.n_cubos-self.cubos_recogidos)*-10.0
                return self.observation, self.reward, self.done, self.failed, self.info
        
        # Suma del tiempo necesario para alcanzar el cubo
        tiempo = trayectory_tuple[1].joint_trajectory.points[-1].time_from_start
        tiempo_seg = tiempo.to_sec()
        
        self.control_robot.move_group.set_pose_target(prev_pose) # Se mueve a la posición deseada
        trayectory_tuple = self.control_robot.plan_pose(cube_pose)

        if trayectory_tuple[0] != True:  # Si la trayectoria no es válida
            self.control_robot.add_box_obstacle(f'Cubo_{selected_cube.id}', selected_cube.pose, [0.025]*3)
            if trayectory_tuple[3].val == -1:  # Si la colisión es con el robot
                self.failed_attempts += 1
                self.n_steps += 1
                self.reward -= 10.0
                if self.verbose:
                    print('\033[31mColisión con Escena\033[0m')
                return self.observation, self.reward, self.done, self.failed, self.info
            
            else:
                self.failed = True
                self.n_steps += 1
                if self.verbose:
                    print(f'\033[31m\nTrial {self.n_trials} - Step {self.n_steps} - \
                          Fallado con Accion {self.orden_cubos} - Cubo Fallado {selected_cube.id}\033[0m')
                    self.reward -= (self.n_cubos-self.cubos_recogidos)*-10.0
                return self.observation, self.reward, self.done, self.failed, self.info

        # Suma del tiempo necesario para alcanzar el cubo
        tiempo = trayectory_tuple[1].joint_trajectory.points[-1].time_from_start
        tiempo_seg += tiempo.to_sec()

        if tiempo_seg > 5:  # Penaliza si el tiempo es excesivo
            self.reward -= 2.0
        else:  # Recompensa por tiempo dentro del rango esperado
            self.reward += 1

        self.total_time += tiempo_seg  # Acumula el tiempo total
        
        self.reward -= tiempo_seg

        self.cubos_recogidos += 1
        self.taken_actions.add(action)
        self.orden_cubos.append(action)

        self.needed_cubes[selected_cube.color] -= 1
        self.available_cubes[selected_cube.color] -= 1
        self.cubos[selected_cube.id].color = -1
        self.cubos_recogidos += 1
        
        self.cubos[selected_cube.id].pose = Pose(Point(0.0, 0.0, 0.0),
                                        Quaternion(0.0, 0.0, 0.0, 0.0))

        if self.cubos_recogidos == self.n_cubos: # Si se han recogido todos los cubos
            self.done=True
            self.n_steps += 1
        
        if self.done: # Si el episodio ha terminado
            avg_time = self.total_time / len(self.taken_actions)
            self.reward += (4 - avg_time) * self.cubos_recogidos  # La recompensa aumenta cuando el tiempo promedio es cercano a 4 segundos

        if self.verbose and self.done: # Si se desea mostrar información en pantalla
            print(f'\033[32m\nTrial {self.n_trials} - Step {self.n_steps} - \
                  Completado con Recompensa {self.reward} - Accion {str(self.orden_cubos)}\033[0m')
        
        return self.observation, self.reward, self.done, self.failed, self.info

    def reset(self, *, seed: Union[int, None] = None, options: Union[Dict[str, Any], None] = None) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Reinicia el entorno.
            @param seed: Union[int, None], semilla para la generación de números pseudo-aleatorios.
            @param options: Union[Dict[str, Any], None], opciones adicionales.
            @return: Tuple[np.ndarray, Dict[str, Any]], observación e información adicional.
        """
        if not seed is None:
            set_random_seed(seed)

        super(ROSEnv, self).reset(seed=seed, options=options)

        self.observation = np.array([[-1]* 8]  * (self.cube_limit+1))
        self.done = False
        self.failed = False
        self.info = {}
        self.reward = 0.0
        self.n_steps = 0
        self.n_trials += 1

        self.pseudo_rands_cubos = []
        self.cubos = []
        self.all_colors_found = False
        self.figure_order = []
        self.cubos_recogidos = 0
        self.discarded_cubes = [0, 0, 0, 0]
        self.orden_cubos = []
        self.total_time = 0
        self.taken_actions = set()
        self.needed_cubes = {i: 0 for i in range(4)}
        self.available_cubes = [0]*4
        self.colors_found = False

        self.control_robot.reset_planning_scene()
        
        self.control_robot.move_group.set_joint_value_target(self.j_home)

        self._sample_new_figure()

        for _ in range(self.n_cubos):
            self._sample_new_cube_value(**self.robot_workspace_values)

        self._replace_unknow_colors()

        self._añadir_cubos_a_planificacion(self.pseudo_rands_cubos)
        self._empty_workspace()

        self._get_obs()
        self._get_info()

        return self.observation, self.info
