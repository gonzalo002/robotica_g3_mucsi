#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
from std_msgs.msg import Int32
from typing import List
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from proyecto_final.msg import HandData

class ControlRobot:
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = MoveGroupCommander(self.group_name)
        self.add_floor()

    def get_motor_angles(self) -> List[float]:
        return self.move_group.get_current_state().joint_state.position

    def move_motors(self, joint_goal: List[float], wait: bool=True) -> bool:
        return self.move_group.go(joint_goal, wait=wait)

    def get_pose(self) -> Pose:
        return self.move_group.get_current_pose().pose

    def move_to_pose(self, pose_goal: Pose, wait: bool=True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)

    def move_to_xyz(self, x: float, y: float, z: float, wait: bool=True) -> bool:
        current_pose = self.get_pose()
        current_pose.position.x = x
        current_pose.position.y = y
        current_pose.position.z = z
        return self.move_to_pose(current_pose, wait)

    def add_to_planning_scene(self, pose_box: Pose, name: str, size: tuple = (0.1, 0.1, 0.1)) -> None:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_box
        self.scene.add_box(name, box_pose, size=size)

    def move_trajectory(self, poses: List[Pose], wait: bool=True) -> bool:
        try:
            (plan, fraction) = self.move_group.compute_cartesian_path(
                poses,     # waypoints to follow
                0.01,      # eef_step
                0.0)       # jump_threshold
            
            if fraction < 1.0:
                rospy.logwarn(f"Solo se pudo planificar {fraction*100}% de la trayectoria")
                return False
                
            success = self.move_group.execute(plan, wait=wait)
            return success
            
        except Exception as e:
            rospy.logerr(f"Error al mover en trayectoria: {str(e)}")
            return False

    def add_floor(self) -> None:
        pose_floor = Pose()
        pose_floor.position.z = -0.026
        self.add_to_planning_scene(pose_floor, "floor", (2, 2, 0.05))

control_por_mano_activado = False  # Variable global para controlar el estado

def control_callback(msg, control_robot):
    global control_por_mano_activado
    action_code = msg.data
    if action_code == 1:
        control_robot.add_floor()
        print("Se ha añadido el suelo a la escena de planificación.")
    elif action_code == 2:
        current_pose = control_robot.get_pose()
        print(f"Pose actual del robot: ")
        print(f"Posición: x={current_pose.position.x}, y={current_pose.position.y}, z={current_pose.position.z}")
        print(f"Orientación: qx={current_pose.orientation.x}, qy={current_pose.orientation.y}, qz={current_pose.orientation.z}, qw={current_pose.orientation.w}")

        print("Introduzca los valores para X, Y, Z (separados por espacios):")
        input_str = input()
        values = [float(x) for x in input_str.strip().split()]
        if len(values) != 3:
            print("Entrada inválida. Se esperaban 3 valores.")
            return
        x, y, z = values
        success = control_robot.move_to_xyz(x, y, z)
        if success:
            print(f"El robot se ha movido a la posición (X={x}, Y={y}, Z={z}) manteniendo la orientación actual.")
        else:
            print("No se pudo mover a la posición deseada.")
    elif action_code == 3:
        print("Introduzca los ángulos de las articulaciones (en radianes) separados por espacios:")
        input_str = input()
        joint_goal = [float(x) for x in input_str.strip().split()]
        success = control_robot.move_motors(joint_goal)
        if success:
            print("El robot se ha movido a la configuración objetivo.")
        else:
            print("No se pudo mover a la configuración objetivo.")
    elif action_code == 4:
        print("Introduzca el número de puntos de la trayectoria:")
        num_waypoints = int(input())
        waypoints = []
        for i in range(num_waypoints):
            print(f"Introduzca el punto {i+1} (x y z):")
            input_str = input()
            values = [float(x) for x in input_str.strip().split()]
            if len(values) != 3:
                print("Entrada inválida. Se esperaban 3 valores.")
                return
            pose = Pose()
            pose.position.x = values[0]
            pose.position.y = values[1]
            pose.position.z = values[2]
            current_pose = control_robot.get_pose()
            pose.orientation = current_pose.orientation
            waypoints.append(pose)
        success = control_robot.move_trajectory(waypoints)
        if success:
            print("El robot se ha movido a lo largo de la trayectoria.")
        else:
            print("No se pudo completar la trayectoria.")
    elif action_code == 5:
        control_por_mano_activado = True
        print("Control por detección de manos activado.")
    elif action_code == 6:
        control_por_mano_activado = False
        print("Control por detección de manos desactivado.")
    else:
        print("Código de acción inválido recibido.")

def hand_data_callback(msg, control_robot):
    global control_por_mano_activado
    if not control_por_mano_activado:
        return

    x = msg.x
    y = msg.y
    is_open = msg.is_open

    if is_open:
        limitesx = [0.23207591013811013, -0.19061986164757674]
        limitesy = [0.44488890481150184, 0.23159845770381593]
        rango_x_cam = 640
        rango_y_cam = 480
        rango_x_robot = limitesx[0] - limitesx[1]
        rango_y_robot = limitesy[0] - limitesy[1]

        current_pose = control_robot.get_pose()
        
        # Calcular el desplazamiento proporcional
        delta_x = (x / rango_x_cam) * rango_x_robot * 0.1
        delta_y = (y / rango_y_cam) * rango_y_robot * 0.1

        # Crear nueva pose objetivo
        target_pose = copy.deepcopy(current_pose)
        target_pose.position.x = current_pose.position.x + delta_x
        target_pose.position.y = current_pose.position.y + delta_y

        # Verificar límites
        if target_pose.position.x > limitesx[0] or target_pose.position.x < limitesx[1]:
            rospy.logwarn(f"Posición X fuera de límites: {target_pose.position.x}")
            return
        if target_pose.position.y > limitesy[0] or target_pose.position.y < limitesy[1]:
            rospy.logwarn(f"Posición Y fuera de límites: {target_pose.position.y}")
            return

        try:
            # Planificar y ejecutar el movimiento directamente usando move_to_pose
            success = control_robot.move_to_pose(target_pose, wait=False)
            
            if success:
                rospy.loginfo("Movimiento exitoso")
            else:
                rospy.logwarn("No se pudo ejecutar el movimiento")
                
        except Exception as e:
            rospy.logerr(f"Error en el movimiento: {str(e)}")
    else:
        rospy.loginfo("Mano cerrada; el robot no se mueve.")

def listener():
    rospy.init_node('control_robot_node', anonymous=True)
    control_robot = ControlRobot()
    rospy.Subscriber('/consignas', Int32, control_callback, callback_args=control_robot)
    rospy.Subscriber('/hand_data', HandData, hand_data_callback, callback_args=control_robot)
    rospy.spin()

if __name__ == '__main__':
    #control = ControlRobot()
    #while True:
        #print(control.get_pose())
        #input('Holi')
    listener()