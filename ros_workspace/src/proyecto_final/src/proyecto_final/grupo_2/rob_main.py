#!/usr/bin/python3

from control_robot import ControlRobot
import numpy as np
from copy import deepcopy
import rospy
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from sensor_msgs.msg import JointState

from action_client_master import MasterClient
from proyecto_final.msg import IdCubos
from typing import List
from tf.transformations import quaternion_from_euler
from math import pi

class SecuenceCommander:
    def __init__(self, simulation:bool = False) -> None:
        self.robot = ControlRobot('robot')
        self.simulation = simulation

        self.action_client = MasterClient()

        self.figure_origin:Pose = self.robot.get_from_yaml(f'./src/proyecto_final/src/proyecto_final/grupo_2/trayectorias/puntos_dejada', 'Matrix_Ini')
        self.discard_origin:Pose = self.robot.get_from_yaml(f'./src/proyecto_final/src/proyecto_final/grupo_2/trayectorias/puntos_dejada', 'Discard_Ini')
        self.discarded_cubes:list = [0, 0, 0, 0] # RGBY

        self.home:JointState = self.robot.get_from_yaml('src/proyecto_final/src/proyecto_final/grupo_2/trayectorias/master_points', 'home')
        self.link1:JointState = self.robot.get_from_yaml('src/proyecto_final/src/proyecto_final/grupo_2/trayectorias/master_points', 'enlace_1')
        self.joint_fuera_camara:JointState = self.robot.get_from_yaml('src/proyecto_final/src/proyecto_final/grupo_2/trayectorias/master_points', 'fuera_rango_vision')

        self.reset:bool = False
        self.start:bool = False
        self.retry:bool = False

        self.workspace_range = {'x_max': 10, 'x_min': 0, 'y_max': 10, 'y_min': 0}
         
    def moveJoint(self, jointstate:JointState) -> bool:      
        success = self.robot.set_joint_angles(jointstate)
        
        if success == True:
            rospy.loginfo('JointState alcanzada')
        else:
            rospy.logwarn('JointState inalcanzable')
        return success
    
    def movePose(self, pose:Pose):               
        success = self.robot.set_pose(pose)
        
        if success == True:
            rospy.loginfo('Pose alcanzada')
        else:
            rospy.logwarn('Pose inalcanzable')
        return success
    
    def empty_workspace(self, cubos:List[IdCubos], x_max:int, x_min:int, y_max:int, y_min:int):        
        for cube_id, cubo in enumerate(cubos):
            pose = deepcopy(cubo.pose)
            color = cubo.color
            if pose.position.x > x_min and pose.position.x < x_max:
                if pose.position.y > y_min and pose.position.y < y_max:
                    self.pick_cube(pose, cube_id)
                    self.drop_cube(make_figure=False, matrix_position=[0, color, self.discarded_cubes[color]])
                    self.discarded_cubes[color] += 1

    def pick_cube(self, pose:Pose = [], cube_id:int = None):
        if not self.simulation:
            if self.robot.get_pinza_state()[0] > 0.56:
                self.robot.mover_pinza(40.0, 10.0)
        pose_previa = deepcopy(pose)
        if pose.position.z > 0.3:
            pose_previa.position.z += 0.15
        else:
            pose_previa.position.z += 0.3

        pose.position.z +=0.25

        self.movePose(pose_previa)

        if self.robot.set_carthesian_path([pose_previa, pose]):
            if cube_id != None:
                self.robot.scene.attach_object(f'cubo_{cube_id}', 'right_inner_finger')
            if not self.simulation:
                self.robot.mover_pinza(0.0, 10.0)
                rospy.sleep(0.85)
            if self.robot.set_carthesian_path([pose, pose_previa]):
                self.moveJoint(self.home)

    def drop_cube(self, make_figure:bool = True, matrix_position:list = [0,0,0], cube_size:float = 0.02, cube_separation:float = 0.01):
        if not self.simulation:
            if self.robot.get_pinza_state() < 0.2:
                raise ValueError('No hay cubo para dejar')
        if len(matrix_position) != 3:
                raise ValueError('Invalid Input, Matrix position len must be 3')
        if make_figure:
            pose = deepcopy(self.figure_origin)
        else:
            pose = deepcopy(self.discard_origin)
        
        pose_previa = deepcopy(pose)
        if pose.position.z > 0.3:
            pose_previa.position.z += 0.15
        else:
            pose_previa.position.z += 0.3

        pose.position.z +=0.25
            
        pose.position.x += ((cube_size + cube_separation) * matrix_position[0])
        pose.position.y += ((cube_size + cube_separation) * matrix_position[1])
        pose.position.z += ((cube_size) * matrix_position[2])


        self.movePose(pose_previa)

        if self.robot.set_carthesian_path([pose_previa, pose]):
            if not self.simulation:
                self.robot.mover_pinza(30.0, 10.0)
                rospy.sleep(1)
            if self.robot.set_carthesian_path([pose, pose_previa]):
                self.moveJoint(self.home)
    
    def generate_cubes(self, cubos:List[IdCubos]):
        for i, cubo in enumerate(cubos):
            pose = cubo.pose
            self.robot.set_box_obstacle(f"cubo{i}", pose, (0.02, 0.02, 0.02))

    def free_camera_space(self):
        self.moveJoint(self.joint_fuera_camara)

    def main(self, cubos:List[IdCubos]):
        self.moveJoint(self.home)
        self.generate_cubes(cubos)
        self.empty_workspace(cubos, **self.workspace_range)

if __name__ == '__main__':
    cubos = []
    lista_poses = [Pose(position = Point(x=0.26, y=0.4, z=0.01),
                        orientation= Quaternion(x=-1, y=0, z=0, w=0)),
                    Pose(position = Point(x=0.2, y=0.26, z=0.01),
                        orientation= Quaternion(x=-1, y=0, z=0, w=0)),
                    Pose(position = Point(x=0.0, y=0.4, z=0.01),
                        orientation= Quaternion(x=-1, y=0, z=0, w=0)),
                    Pose(position = Point(x=0.0, y=0.26, z=0.01),
                        orientation= Quaternion(x=-1, y=0, z=0, w=0))]
    for i in range(4):
        cubo = IdCubos()
        cubo.color = i
        cubo.pose = lista_poses[i]
        cubos.append(cubo)

    robot = SecuenceCommander(simulation=True)
    robot.main(cubos)

    # robot.free_camera_space()