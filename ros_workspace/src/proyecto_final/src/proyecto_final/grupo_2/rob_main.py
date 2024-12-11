#!/usr/bin/python3

import sys
# Importaciones locales (ajustar la ruta de acuerdo a tu proyecto)
sys.path.append("/home/laboratorio/ros_workspace/src/proyecto_final/src/proyecto_final")

from control_robot import ControlRobot
import numpy as np
from cube_tracker import CubeTracker
import cv2
from copy import deepcopy
import rospy
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from sensor_msgs.msg import JointState

from action_client_master import MasterClient
from proyecto_final.msg import IdCubos
from typing import List
from tf.transformations import inverse_matrix, quaternion_matrix, translation_matrix, quaternion_from_euler
from math import pi

class SecuenceCommander:
    def __init__(self, simulation:bool = False) -> None:
        self.robot = ControlRobot('robot')
        self.cube_tracker = CubeTracker(cam_calib_path="/home/laboratorio/ros_workspace/src/proyecto_final/data/camera_data/ost.yaml")
        self.simulation = simulation

        self.action_client = MasterClient()

        self.figure_origin:Pose = self.robot.read_from_yaml(f'./src/proyecto_final/src/proyecto_final/grupo_2/trayectorias/puntos_dejada', 'Matrix_Ini')
        self.discard_origin:Pose = self.robot.read_from_yaml(f'./src/proyecto_final/src/proyecto_final/grupo_2/trayectorias/puntos_dejada', 'Discard_Ini')
        self.discarded_cubes:list = [0, 0, 0, 0] # RGBY

        self.home:JointState = self.robot.read_from_yaml('src/proyecto_final/src/proyecto_final/grupo_2/trayectorias/master_points', 'home')
        self.link1:JointState = self.robot.read_from_yaml('src/proyecto_final/src/proyecto_final/grupo_2/trayectorias/master_points', 'enlace_1')
        self.joint_fuera_camara:JointState = self.robot.read_from_yaml('src/proyecto_final/src/proyecto_final/grupo_2/trayectorias/master_points', 'fuera_rango_vision')

        self.reset:bool = False
        self.start:bool = False
        self.retry:bool = False

        self.workspace_range = {'x_max': 10, 'x_min': 0, 'y_max': 10, 'y_min': 0}
         
    def moveJoint(self, jointstate:JointState) -> bool:      
        success = self.robot.move_jointstates(jointstate)
        
        if success == True:
            rospy.loginfo('JointState alcanzada')
        else:
            rospy.logwarn('JointState inalcanzable')
        return success
    
    def movePose(self, pose:Pose):               
        success = self.robot.move_pose(pose)
        
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
                    self.drop_cube(make_figure=False, cube_id=cube_id, matrix_position=[0, color, self.discarded_cubes[color]])
                    self.discarded_cubes[color] += 1

    def pick_cube(self, pose:Pose = [], cube_id:int = None):
        if not self.simulation:
            gripper, _ = self.robot.get_pinza_state()
            if  gripper < 20.0:
                self.robot.move_gripper(30.0, 10.0, 0.8)
        pose_previa = deepcopy(pose)
        if pose.position.z > 0.3:
            pose_previa.position.z += 0.15
        else:
            pose_previa.position.z += 0.3

        pose.position.z +=0.25

        self.movePose(pose_previa)

        if self.robot.move_carthesian_trayectory([pose_previa, pose]):
            if cube_id != None:
                self.robot.scene.attach_box(link='onrobot_rg2_base_link', name=f'cubo{cube_id}')
            if not self.simulation:
                self.robot.move_gripper(0.0, 10.0, 0.8)
            if self.robot.move_carthesian_trayectory([pose, pose_previa]):
                self.moveJoint(self.link1)

    def drop_cube(self, make_figure:bool = True, cube_id:int = None, matrix_position:list = [0,0,0], cube_size:float = 0.02, cube_separation:float = 0.01):
        if not self.simulation:
            _, effort = self.robot.get_pinza_state()
            if not effort:
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

        pose.position.z +=0.22
            
        pose.position.x += ((cube_size + cube_separation) * matrix_position[0])
        pose.position.y += ((cube_size + cube_separation) * matrix_position[1])
        pose.position.z += ((cube_size) * matrix_position[2])


        self.movePose(pose_previa)

        if self.movePose(pose):
        # if self.robot.set_carthesian_path([pose_previa, pose]):
            self.robot.scene.remove_attached_object(link='onrobot_rg2_base_link', name=f'cubo{cube_id}')
            if not self.simulation:
                self.robot.move_gripper(30.0, 10.0)
            if self.robot.move_carthesian_trayectory([pose, pose_previa]):
                self.moveJoint(self.home)
    
    def generate_cubes(self, cubos:List[IdCubos]):
        for i, cubo in enumerate(cubos):
            pose = cubo.pose
            self.robot.add_box_obstacle(f"cubo{i}", pose, (0.02, 0.02, 0.02))

    def track_cubes(self, cam_id:int, use_cam:bool = True, mostrar:bool = True, debug:bool = False) -> dict:
        self.free_camera_space()

        if use_cam:
            cam = cv2.VideoCapture(cam_id)
            if cam.isOpened():
                _, frame = cam.read()
        else:
            num = 1
            ruta = f'/home/laboratorio/ros_workspace/src/proyecto_final/data/example_img/Cubos_Exparcidos/Cubos_Exparcidos_{num}.png'
            frame = cv2.imread(ruta)
        
        _, resultado = self.cube_tracker.process_image(frame, area_size=1000, mostrar=mostrar, debug=debug)

        return resultado

    def try_pickup(self, dict_cubos, get_aruco_pose:bool=True):
        if get_aruco_pose:
            pose_aruco = self.rob_camera_calibration()
        else:
            pose_aruco = self.robot.read_from_yaml('/home/laboratorio/ros_workspace/src/proyecto_final/src/proyecto_final/grupo_2/trayectorias/pose_aruco', 'pose')

        for dict in dict_cubos:
            position = dict['Position']

            angle = dict['Angle']
            color = dict['Color']

            pose = deepcopy(pose_aruco)
            pose.position.x += position[0]
            pose.position.y += position[1]
            pose.position.z = 0.01

            pose.orientation = Quaternion(*quaternion_from_euler(pi, 0, angle, 'sxyz'))

            self.moveJoint(self.link1)

            self.pick_cube(pose)



    def rob_camera_calibration(self):
        # Obtener datos pose aruco
        print('LLevar robot ')
        input('Robot en Pose Aruco?: ')

        pose = self.robot.get_pose()
        self.robot.save_in_yaml('pose_aruco', 'pose', pose)

        return pose

    def free_camera_space(self):
        self.moveJoint(self.joint_fuera_camara)

    def main(self):
        cubos = self.track_cubes(cam_id = 0, use_cam=True)

        self.moveJoint(self.home)
        self.try_pickup(cubos, True)

if __name__ == '__main__':
    use_cam = True
    
    robot = SecuenceCommander(simulation=False)

    robot.main()




    # cubos = []
    # lista_poses = [Pose(position = Point(x=0.26, y=0.4, z=0.01),
    #                     orientation= Quaternion(x=-1, y=0, z=0, w=0)),
    #                 # Pose(position = Point(x=0.2, y=0.26, z=0.01),
    #                 #     orientation= Quaternion(x=-1, y=0, z=0, w=0)),
    #                 # Pose(position = Point(x=0.0, y=0.4, z=0.01),
    #                 #     orientation= Quaternion(x=-1, y=0, z=0, w=0)),
    #                 # Pose(position = Point(x=0.0, y=0.26, z=0.01),
    #                 #     orientation= Quaternion(x=-1, y=0, z=0, w=0))
    #                 ]
    # for i in range(len(lista_poses)):
    #     cubo = IdCubos()
    #     cubo.color = i
    #     cubo.pose = lista_poses[i]
    #     cubos.append(cubo)

    # robot = SecuenceCommander(simulation=False)
    # # robot.main(cubos)
    # pose_camara_aruco = Pose(position = Point(x=453, y=341, z=0),
    #                     orientation= Quaternion(*quaternion_from_euler(pi, 0, 17.35, 'sxyz')))
    # print(robot.rob_camera_calibration())
    # # robot.free_camera_space()
