#!/usr/bin/python3

import sys
import copy
import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cosh
from std_msgs.msg import String, Float32
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
import yaml
import time
class ControlRobot:
    def __init__(self, group_name:str = 'robot') -> None:
        roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = group_name
        self.move_group = MoveGroupCommander(self.group_name)
        self.add_floor()
    
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
        self.add_floor()

        rospy.loginfo("Planning scene cleared!")

    def add_floor(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z -= .03
        self.set_box_obstacle('floor', pose_suelo, (2,2,.05))

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