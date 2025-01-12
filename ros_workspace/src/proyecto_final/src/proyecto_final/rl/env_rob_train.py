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
    def __init__(self, num_cubos_max: int, seed: int = None, visualization:bool = False, save_data:bool = False, verbose:bool = False):
        """
        Constructor de la clase ROSEnv.
            @param num_cubos_max: int, número máximo de cubos a colocar.
            @param seed: int, semilla para la generación de números aleatorios.
            @param visualization: bool, indica si se desea visualizar el entorno.
            @param save_data: bool, indica si se desea guardar los datos.
            @param verbose: bool, indica si se desea mostrar mensajes en consola.
        """
        super(ROSEnv, self).__init__()
        self.control_robot = ControlRobot("robot", train_env=True)

        self.abs_path = '/'.join(os.path.dirname(os.path.abspath(__file__)).split('/')[:os.path.dirname(os.path.abspath(__file__)).split('/').index('proyecto_final')+1])

        self.cube_limit = 25 # Se pone un limite de 25 cubos, de esta forma un mismo agente puede funcionar con hasta 25 cubos
        self.n_cubos = num_cubos_max
        self.seed = seed
        self.visualization = visualization
        self.verbose = verbose

        self.action_space = gym.spaces.Discrete(self.n_cubos)

        self.observation_space = gym.spaces.Box(
            low=np.array([[-1]* 8]  * (self.cube_limit+1)),
            high=np.array([[-1]* 8] * (self.cube_limit+1)),
            dtype=np.float64
        )

        self.cube_size = 0.025
        self.cube_separation = 0.02 # Separación entre cubos
        self.robot_workspace_values = {"max_x": 0.25, "min_x": -0.25, "max_y": 0.4, "min_y": 0.17, "max_alpha": pi/4, "min_alpha": -pi/4}

        self.j_link_1:JointState = self.control_robot.read_from_yaml(f'{self.abs_path}/data/trayectorias/master_positions', 'J_LINK_1')
        self.j_home:JointState = self.control_robot.read_from_yaml(f'{self.abs_path}/data/trayectorias/master_positions', 'J_HOME')
        self.p_discard_origin:Pose = self.control_robot.read_from_yaml(f'{self.abs_path}/data/trayectorias/master_positions', 'P_DISCARD_ORIGIN')
        self.p_figure_origin:Pose = self.control_robot.read_from_yaml(f'{self.abs_path}/data/trayectorias/master_positions', 'P_MATRIX_ORIGIN')

        self.workspace_range = {'x_max': self.p_figure_origin.position.x+((self.cube_size + self.cube_separation)*5.5), 
                                'x_min': self.p_figure_origin.position.x-((self.cube_size + self.cube_separation)*0.5), 
                                'y_max': self.p_figure_origin.position.y+((self.cube_size + self.cube_separation)*5.5), 
                                'y_min': self.p_figure_origin.position.y-((self.cube_size + self.cube_separation)*0.5)} 

        self.cubos:List[IdCubos] = []
        self.rejected_cubes:List[IdCubos] = []
        self.pseudo_rands_cubos = []
        self.figure_order = []
        self.needed_cubes = {}
        self.cubos_recogidos = 0
        self.discarded_cubes:list = [0, 0, 0, 0] # RGBY
        self.cube_size = 0.025
        self.cube_separation = 0.015
        
        self.flag_save_data = save_data
        self.verbose = verbose
        self.n_steps = 0
        self.action_buffer = set() # Set para las acciones realizadas
        self.orden_cubos = []
        self.total_time = 0  # Total time taken for all actions
        self.colors_found = False

        self.observation = np.array([[-1]* 8]  * (self.n_cubos+1))
        self.reward = 0.0
        self.info = {}
        self.terminated = False
        self.truncated = False

        self.control_robot.reset_planning_scene()
        self.control_robot.move_group.set_joint_value_target(self.j_home)

    def _get_obs(self):
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
        self.info = {}
        if self.terminated:
            self.info = {"orden_cubos": self.orden_cubos}
            return self.info
        self.info = {'orden_cubos': range(self.n_cubos)}
        
    def _sample_new_figure(self):
        """
        Sample a new figure to be built by cubes.
        """
        for _ in range(self.n_cubos+1):
            self.figure_order.append(np.random.randint(0,4))
            if np.random.random() > 0.85:
                self.figure_order[-1] = 4

        # self.figure_order = np.random.randint(0, 4, size=self.num_cubos_max)
        unique, counts = np.unique(self.figure_order, return_counts=True)
        # Crear un diccionario con las claves del 0 al 3 e inicializarlas a 0
        self.needed_cubes = {i: 0 for i in range(4)}
        
        # Actualizar el diccionario con los valores presentes en figure_order
        self.needed_cubes.update(dict(zip(unique, counts)))

    def __sample_new_cube_value(self, max_x: float, min_x: float, 
                            max_y: float, min_y: float, 
                            max_alpha: float, min_alpha: float) -> np.ndarray:
        """
        Sample a new cube value for x, y and alpha.
        Input:
            max_x: float, maximum value for x.
            min_x: float, minimum value for x.
            max_y: float, maximum value for y.
            min_y: float, minimum value for y.
            max_alpha: float, maximum value for alpha.
            min_alpha: float, minimum value for alpha.
        Output:
            np.ndarray, new cube value for x, y and alpha.
        """
        
        if max_x < min_x or max_y < min_y or max_alpha < min_alpha:
            raise ValueError("max values must be greater than min values")
        
        success = False
        
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
        
        pseudo_rands[2] = np.interp(pseudo_rands[2], [0, 1], [min_alpha, max_alpha])
        pseudo_rands[3] = np.random.randint(0, 4)

        
        if self.needed_cubes[int(pseudo_rands[3])] <= 0 and not self.colors_found:
            checked_values = set()
            pseudo_rands[3] = np.random.randint(0, 4)
            while self.needed_cubes[int(pseudo_rands[3])] <= 0:
                checked_values.add(pseudo_rands[3])    
                if len(checked_values) == 4:
                    self.colors_found = True
                    break
                pseudo_rands[3] = np.random.randint(0, 4)
        
        self.needed_cubes[int(pseudo_rands[3])] -= 1

        if self.colors_found:
            for i, figure_cube in enumerate(self.figure_order):
                if figure_cube == 4:
                    self.figure_order[i] = int(pseudo_rands[3])
                    break

        if pseudo_rands[0] > 0:
            pseudo_rands[2] -= pi/2
        
        self.pseudo_rands_cubos.append(deepcopy(pseudo_rands))

        return pseudo_rands

    def _añadir_cubos_a_planificacion(self, cubos: List[IdCubos]) -> None:
        for i, pseudo_rands in enumerate(cubos):
            cubo:IdCubos = IdCubos()
            cubo.pose = Pose(position=Point(x=pseudo_rands[0], y=pseudo_rands[1], z=0.0125), 
                             orientation=Quaternion(*quaternion_from_euler(pi, 0, pseudo_rands[2], 'sxyz')))
            cubo.color = int(pseudo_rands[3])
            cubo.id = i

            self.cubos.append(cubo)

            if self.visualization == True:
                self.control_robot.add_box_obstacle(f"Cubo_{cubo.id}", cubo.pose, (0.025, 0.025, 0.025))
    
    def _empty_workspace(self):
        """
        Empty the workspace of the robot.
        """
        ''' 
        Libera el espacio de trabajo del robot, moviendo y descartando los cubos encontrados.
            @return None
        '''
        x_min = self.workspace_range['x_min']
        y_min = self.workspace_range['y_min']        
        x_max = self.workspace_range['x_max']        
        y_max = self.workspace_range['y_max']   
        
        self.control_robot.move_jointstates(self.j_link_1)     
                
        cube:IdCubos
        for cube in self.cubos:
            pose:Pose = deepcopy(cube.pose)
            color = cube.color
            if pose.position.x > x_min and pose.position.x < x_max:
                if pose.position.y > y_min and pose.position.y < y_max:
                    self._discard_cube(cube_id=cube.id, matrix_position=[0, color, self.discarded_cubes[color]])
                    self.discarded_cubes[color] += 1
        

    def _discard_cube(self, cube_id:int = None, matrix_position:list = [0,0,0]) -> bool:
        ''' 
        Suelta el cubo en la posición deseada.
            @param make_figure (bool) - Determina si el cubo forma parte de una figura o es un descarte.
            @param cube_id (int) - Identificador único del cubo.
            @param matrix_position (list) - Posición 3D para colocar el cubo en una matriz.
            @return success (bool) - Indica si el movimiento fue exitoso.
        '''
        self.control_robot.scene.remove_world_object(f'Cubo_{cube_id}')


        # Se determina la pose inicial dependiendo si el cubo forma parte de una figura.
        pose: Pose = deepcopy(self.p_discard_origin)  # Pose inicial para desechar el cubo.

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

        if action in self.taken_actions: # Si la acción ya ha sido tomada
            self.n_steps += 1 # Incrementa el número de pasos
            self.reward -= 5.0 # Penaliza la acción con -5
            return self.observation, self.reward, self.terminated, self.truncated, self.info 
        
        figure_color = self.observation[-1,-1]
        selected_cube:IdCubos = deepcopy(self.cubos[action]) # Selecciona el cubo a recoger
        if selected_cube.color != figure_color: # Si el color del cubo no coincide con el color de la figura
            self.reward -= 5.0 # Penaliza la acción con -5
            self.n_steps += 1 # Incrementa el número de pasos
            return self.observation, self.reward, self.terminated, self.truncated, self.info 

        cube_pose:Pose = selected_cube.pose # Obtiene la pose del cubo seleccionado
        cube_pose.position.z = 0.237

        prev_pose:Pose = deepcopy(cube_pose) # Copia la pose del cubo seleccionado
        prev_pose.position.z = 0.237 # Ajusta la altura del cubo para recogerlo

        if self.visualization:
            self.control_robot.scene.remove_world_object(f'Cubo_{selected_cube.id}')

        trayectory_tuple = self.control_robot.plan_pose(prev_pose)

        if trayectory_tuple[0] != True: 
            if self.visualization:
                self.control_robot.add_box_obstacle(f'Cubo_{selected_cube.id}', selected_cube.pose, [0.025]*3)
            if trayectory_tuple[3].val == -1:
                self.n_steps += 1
                self.reward -= 10.0
                if self.verbose:
                    print('Colisión con Escena')
                return self.observation, self.reward, self.terminated, self.truncated, self.info
            
            else:
                self.truncated = True
                self.n_steps += 1
                if self.verbose:
                    print(f'\nStep {self.n_steps} - Fallado con Accion {self.orden_cubos} - Cubo Fallado {selected_cube.id}')
                    self.reward -= (self.n_cubos-self.cubos_recogidos)*-10.0
                return self.observation, self.reward, self.terminated, self.truncated, self.info
        
        # Suma del tiempo necesario para alcanzar el cubo
        tiempo = trayectory_tuple[1].joint_trajectory.points[-1].time_from_start
        tiempo_seg = tiempo.to_sec()
        
        self.control_robot.move_group.set_pose_target(prev_pose) # Se mueve a la posición deseada
        trayectory_tuple = self.control_robot.plan_pose(cube_pose)

        if trayectory_tuple[0] != True: 
            if self.visualization:
                self.control_robot.add_box_obstacle(f'Cubo_{selected_cube.id}', selected_cube.pose, [0.025]*3)
            if trayectory_tuple[3].value == -1:
                self.n_steps += 1
                self.reward -= 10.0
                if self.verbose:
                    print('Colisión con Escena')
                return self.observation, self.reward, self.terminated, self.truncated, self.info
            
            else:
                self.truncated = True
                self.n_steps += 1
                if self.verbose:
                    print(f'\nStep {self.n_steps} - Fallado con Accion {self.orden_cubos} - Cubo Fallado {selected_cube.id}')
                    self.reward -= (self.n_cubos-self.cubos_recogidos)*-10.0
                return self.observation, self.reward, self.terminated, self.truncated, self.info

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

        self.cubos[selected_cube.id].color = -1
        self.cubos_recogidos += 1
        
        self.cubos[selected_cube.id].pose = Pose(Point(0.0, 0.0, 0.0),
                                        Quaternion(0.0, 0.0, 0.0, 0.0))

        if self.cubos_recogidos == self.n_cubos:
            self.terminated=True
            self.n_steps += 1
        
        if self.terminated:
            avg_time = self.total_time / len(self.taken_actions)
            self.reward += (4 - avg_time) * self.cubos_recogidos  # La recompensa aumenta cuando el tiempo promedio es cercano a 4 segundos

        if self.verbose and self.terminated:
            print(f'\nStep {self.n_steps} - Completado con Recompensa {self.reward} - Accion {str(self.orden_cubos)}')
        
        return self.observation, self.reward, self.terminated, self.truncated, self.info

    def reset(self, *, seed: Union[int, None] = None, options: Union[Dict[str, Any], None] = None) -> Tuple[np.ndarray, Dict[str, Any]]:
        super(ROSEnv, self).reset(seed=seed, options=options)
        if not seed is None:
            self.seed = seed
        if not self.seed is None:
            set_random_seed(self.seed)

        variables_cubos = []
        self.pose_cubos = []
        self.taken_actions = set()
        self.orden_cubos = []
        self.pseudo_rands_cubos = []
        self.cubos_recogidos = 0
        self.reward = 0.0
        self.total_time = 0  # Total time taken for all actions
        self.terminated = False
        self.truncated = False

        if self.visualization == True:
            self.control_robot.reset_planning_scene()
        
        self.control_robot.move_group.set_joint_value_target(self.j_link_1)

        self._sample_new_figure()

        for _ in range(self.n_cubos):
            variables_cubos.append(self.__sample_new_cube_value(**self.robot_workspace_values))

        self._añadir_cubos_a_planificacion(variables_cubos)
        self._empty_workspace()

        self._get_obs()

        return self.observation, self.info
