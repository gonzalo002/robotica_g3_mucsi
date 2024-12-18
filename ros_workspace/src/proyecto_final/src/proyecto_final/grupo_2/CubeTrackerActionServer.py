#!/usr/bin/python3
import cv2, os, rospy, actionlib
from time import time
from math import pi
from copy import deepcopy
from threading import Thread
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from proyecto_final.vision.grupo_2.cube_tracker import CubeTracker
from proyecto_final.msg import CubosAction, CubosFeedback, CubosGoal, CubosResult, IdCubos


class CubeTrackerActionServer(object):
    
    def __init__(self):
        # Inicializar nodo
        rospy.init_node('cube_tracker_node')

        # Definición de variables Python
        self.file_path = '/'.join(os.path.dirname(os.path.abspath(__file__)).split('/')[:os.path.dirname(os.path.abspath(__file__)).split('/').index('proyecto_final')+1])
        self.obtain_img = False
        self.cv_img = None
        self.CubeTracker = CubeTracker(cam_calib_path=f"{self.file_path}/data/camera_data/ost.yaml")
        self.bridge = CvBridge()
        
        # Definicion de variables ROS
        self.subs_cam = rospy.Subscriber('/top_cam/image_raw', Image, self.cb_image)
        self._action_server = actionlib.SimpleActionServer('CubeTrackerActionServer', CubosAction, execute_cb=self.execute_cb, auto_start=False)
        self._action_server.start()

        rospy.spin()
    
    def cb_image(self, image:Image)->None:
        ''' 
        Callback del subscriptor de la cámara.
            @param image (Image) - Imagen de la camara
        '''
        if self.obtain_img:
            self.cv_img = deepcopy(self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough'))
            self.cv_img = deepcopy(cv2.cvtColor(self.cv_img, cv2.COLOR_RGB2BGR)) #Convertir imagen a BGR
            self.obtain_img = False

    def execute_cb(self, goal:CubosGoal)->None:
        ''' 
        Callback del action server del CubeTracker
            @param goal (numpy array) - Goal recibido por el cliente
        '''
        if goal.order == 1:
            self.obtain_img = True
            feedback = CubosFeedback()  # Crear un objeto de feedback

            # Crear y ejecutar el hilo de feedback
            feedback_thread = Thread(target=self.send_feedback, args=(feedback,))
            feedback_thread.start()

            # Paso 1: Obtener la imagen con timeout
            timeout = 5  # Tiempo máximo de espera en segundos
            start_time = time()  # Guardar el tiempo de inicio

            imagen = None
            while imagen is None:
                imagen = deepcopy(self.cv_img)

                # Verificar si ha pasado el tiempo de espera
                elapsed_time = time() - start_time
                if elapsed_time > timeout:
                    rospy.logwarn("Timeout al intentar obtener la imagen.")
                    feedback.feedback = -1  # Enviar un valor de feedback para indicar el error
                    self._action_server.publish_feedback(feedback)
                    self._action_server.set_aborted()  # Marcar la acción como fallida
                    return

                rospy.sleep(0.2)
                
            if imagen is None:
                return

            # Paso 2: Procesar la imagen
            resultado_final = CubosResult()
            _, resultado = self.CubeTracker.process_image(imagen)
            cv2.imwrite("imagen.png", imagen)
            resultado_final.cubes_position = self._dict_to_cube(resultado)

            # Paso 3: Finalizar la acción con el resultado procesado
            self._action_server.set_succeeded(resultado_final)

            # Esperar a que el hilo de feedback termine
            feedback_thread.join()
            self.cv_img = None

    def send_feedback(self, feedback:CubosFeedback):
        ''' 
        Función que utiliza el hilo para enviar el feedback al cliente
            @param feedback (CubosActionFeedback) - Feedback
        '''
        while self._action_server.is_active():
            feedback.feedback = 1
            self._action_server.publish_feedback(feedback)
            rospy.sleep(0.1)

    def _dict_to_cube(self, dict_cubos:list) -> list:
        ''' 
        Función que convierte la lista recibida por el procesador de cubos a IDCubos.
            @param dict_cubos (list) - Lista con el diccionario de los diferentes cubos
        '''
        cubos = []
        for dict in dict_cubos:
            # Declaramos IDCubos
            cubo = IdCubos()

            # Ajustamos la posición y orientación
            cubo.pose.position.x = dict['Position'][0]
            cubo.pose.position.y = dict['Position'][1]
            cubo.pose.position.z = 0.0125
            cubo.pose.orientation = Quaternion(*quaternion_from_euler(pi, 0, -dict['Angle'], 'sxyz'))

            # Ajustamos el color del cubo
            cubo.color = dict['Color']
            
            # Guardamos en la lista de cubos
            cubos.append(deepcopy(cubo))
            
        return cubos
            
            
if __name__ == "__main__":
    CubeTrackerActionServer()
