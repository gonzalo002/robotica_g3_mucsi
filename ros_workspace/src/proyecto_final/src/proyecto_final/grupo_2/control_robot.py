#!/usr/bin/python3

import sys
from copy import deepcopy
import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandResult
from math import pi, tau, dist, fabs, cosh
from std_msgs.msg import String, Float32
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
import yaml
from time import time
from actionlib import SimpleActionClient

class ControlRobot:
    def __init__(self, group_name:str = 'robot') -> None:
        roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = group_name
        self.move_group = MoveGroupCommander(self.group_name)
        self.gripper_action_client = SimpleActionClient("rg2_action_server", GripperCommandAction)
        self.gripper = rospy.Subscriber("/rg2/joint_states", JointState, self.gripper_callback)
        self.get_gripper_data = False
        self.gripper_states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.clear_planning_scene()

    
    def get_joint_angles(self) -> list:
        return self.move_group.get_current_joint_values()
    
    def get_pose(self) -> Pose:
        return self.move_group.get_current_pose().pose
    
    def set_joint_angles(self, joint_goal: list, wait: bool = True) -> bool:
        return self.move_group.go(joint_goal, wait=wait)
    
    def set_pose(self, pose_goal: Pose, wait: bool = True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)
    
    def plan_pose_target(self, pose_goal:Pose) -> bool:
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.plan()
    
    def set_joint_trajectory(self, trajectory:list = []) -> bool:
        for i in range(len(trajectory)):
            state = self.set_joint_angles(trajectory[i])
            rospy.loginfo(f'Punto {i} alcanzado \n')
            if not state: print('Trayectoria Fallida'); return False
        rospy.loginfo('Trayectoria Finalizada')
        return state
    
    def set_pose_trajectory(self, trajectory:list = []) -> bool:
        for i in range(len(trajectory)):
            state = self.set_pose(trajectory[i])
            rospy.loginfo(f'Punto {i} alcanzado \n')
            if not state: print('Trayectoria Fallida'); return False
        rospy.loginfo('Trayectoria Finalizada')
        return state
    
    def set_carthesian_path(self, waypoints:list = [], eef_step:Float32 = 0.01, avoid_collisions:bool = True ,wait:bool = True) -> bool:
        if eef_step == 0.0:
            eef_step = 0.01
            print('eef_step modificado a valor 0.01 por requisitos de funcionamiento')
            
        waypoints.insert(0, self.get_pose())
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, eef_step = eef_step, avoid_collisions= avoid_collisions)
        
        if fraction != 1.0:
            rospy.logwarn('Trayectoria Inalcanzable')
            rospy.loginfo(f'Porcentaje de la trayectoria alcanzable: {fraction*100:.2f}%')
            return False
        else: 
            rospy.loginfo('Ejecutando Trayectoria')
            return self.move_group.execute(plan, wait=wait)
        
    def set_box_obstacle(self, box_name:String, box_pose:Pose, size:tuple = (.1, .1, .1)) -> None:
        box_pose_stamped = PoseStamped()
        box_pose_stamped.header.frame_id = "base_link"
        box_pose_stamped.pose = box_pose
        self.scene.add_box(box_name, box_pose_stamped, size=size)
        
    def clear_planning_scene(self) -> None:
        # Optionally: clear the octomap (which stores obstacles in the environment)
        self.scene.clear()
        self.__generate_scene()

        rospy.loginfo("Planning scene cleared!")

    def __generate_scene(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z -= .03
        pose_vertical_support = Pose(Point(x=0,y=-0.1,z=0.5),
                                     Quaternion(x=0,y=0,z=0,w=1))
        pose_camera_support = Pose(Point(x=0.0,y=0.0,z=0.85),
                                     Quaternion(x=0,y=0,z=0,w=1))
        self.set_box_obstacle('floor', pose_suelo, (2,2,.05))
        self.set_box_obstacle('vertical_support', pose_vertical_support, (0.05,0.05,1.0))
        self.set_box_obstacle('camera_support', pose_camera_support, (0.1,1.0,.05))

    def create_pose(self, pos_list:list, ori_list) -> Pose:
        if len(pos_list) != 3 or len(ori_list) != 4: return False
        pose = Pose()
        pose.position.x = pos_list[0]
        pose.position.y = pos_list[1]
        pose.position.z = pos_list[2]
        pose.orientation.x = ori_list[0]
        pose.orientation.y = ori_list[1]
        pose.orientation.z = ori_list[2]
        pose.orientation.w = ori_list[3]

        return pose
    
    def save_in_yaml(self, doc_name:str, key_name:str, data:list) -> None:
        diccionario_configuraciones = {key_name:data}
        with open(doc_name, '+a') as f:
            yaml.dump(diccionario_configuraciones, f)
    
    def get_from_yaml(self, doc_name:str, key_name:str) -> list:
        with open(doc_name, '+r') as f:
            configuraciones =  yaml.load(f, yaml.Loader)

        return configuraciones[key_name]
    
    def gripper_callback(self, data:JointState) -> list:
        if self.get_gripper_data:
            self.gripper_states = data.position
            self.get_gripper_data = False
    
    def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
        goal = GripperCommandGoal()
        goal.command.position = anchura_dedos
        goal.command.max_effort = fuerza
        self.gripper_action_client.send_goal(goal)
        self.gripper_action_client.wait_for_result()
        result = self.gripper_action_client.get_result()
        
        return result.reached_goal
    
    def get_pinza_state(self) -> list:
        gripper_state = deepcopy(self.gripper_states)
        self.get_gripper_data = True
        init_time = time()

        while gripper_state == self.gripper_states:
            if (time() - init_time) > 5:
                rospy.logwarn('TIMEOUT: Unable to get gripper state')
                return self.gripper_states
        return self.gripper_states


if __name__ == '__main__':
    control = ControlRobot('robot')
    
    # i = 0
    # nombres = ["Grande", "Mediano", "Pequenio"]
    # while True:
    #     tecla = input("Tecla: ")
    #     if tecla == "m":
    #         punto = control.get_pose()
    #         control.save_in_yaml("alturas_gorka_unai", nombres[i], punto)
    #         print(f"Posición guardada: {punto}")
    #         time.sleep(0.2)  # Evitar múltiples capturas con una sola pulsación
    #         i+=1
    #     elif tecla == "q":
    #         break
    while True:
        input('Gonzalo Pesado !!!')
        print(control.get_pose())

    #trayectoria=[]
    #punto = control.get_pose()
    #print('Punto inicial de la trayectoria')
    #print(punto)
    #trayectoria.append(copy.deepcopy(punto))
    #punto.position.x -= 0.01
    #trayectoria.append(copy.deepcopy(punto))
    #punto.position.y -= 0.1
    #trayectoria.append(copy.deepcopy(punto))
    #print('Punto final de la trayectoria')
    #print(punto)
    ##control.set_pose_trajectory(trayectoria_pose)
    ##control.set_joint_trajectory(trayectoria_joint)
    #if not control.set_carthesian_path(trayectoria, eef_step=0.01):
    #    pass
    #else:
    #    punto = control.get_pose()
    #    print('Posición Alcanzada')
    #    print(punto)
    

# Funcion que cree un array de Poses
# Terminar funcion de trayectoria cartesiana
# Funcion que cree un JointTarget
# ,,.