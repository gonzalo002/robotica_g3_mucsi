#!/usr/bin/python3
import rospy
import actionlib
import cv2
from time import time
from copy import deepcopy
from threading import Thread
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from proyecto_final.vision.grupo_2.image_processor_front import ImageProcessor_Front
from proyecto_final.vision.grupo_2.image_processor_top import ImageProcessor_Top
from proyecto_final.vision.grupo_2.generacion_figura import FigureGenerator
from proyecto_final.msg import FigurasAction, FigurasActionFeedback, FigurasActionGoal, FigurasActionResult


class FigureMakerActionServer(object):
    
    def __init__(self):
        # Inicializar nodo
        rospy.init_node('cube_tracker_node')

        # Definición de variables Python
        self.obtain_img_front = False
        self.obtain_img_top = False
        self.cv_img_front = None
        self.cv_img_top = None
        self.ImageProcessorFront = ImageProcessor_Front()
        self.ImageProcessorTop = ImageProcessor_Top()
        self.FigureGenerator = FigureGenerator()
        self.bridge = CvBridge()
        
        # Definicion de variables ROS
        self.subs_cam_front = rospy.Subscriber('/front_cam/image_raw', Image, self.cb_image_front)
        self.subs_cam_top = rospy.Subscriber('/top_cam/image_raw', Image, self.cb_image_top)
        self._action_server = actionlib.SimpleActionServer('FigureMakerActionServer', FigurasAction, execute_cb=self.execute_cb, auto_start=False)
        self._action_server.start()

        rospy.spin()
    
    def cb_image_front(self, image:Image)->None:
        ''' 
        Callback del subscriptor de la cámara.
            @param image (Image) - Imagen de la camara
        '''
        if self.obtain_img_front:
            self.cv_img_front = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
            self.obtain_img_front = False
    
    def cb_image_top(self, image:Image)->None:
        ''' 
        Callback del subscriptor de la cámara.
            @param image (Image) - Imagen de la camara
        '''
        if self.obtain_img_top:
            self.cv_img_top = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
            self.obtain_img_top = False

    def execute_cb(self, goal:FigurasActionGoal)->None:
        ''' 
        Callback del action server del CubeTracker
            @param goal (numpy array) - Goal recibido por el cliente
        '''
        if goal.order == 1:
            self.obtain_img = True
            feedback = FigurasActionFeedback()  # Crear un objeto de feedback

            # Crear y ejecutar el hilo de feedback
            feedback_thread = Thread(target=self.send_feedback, args=(feedback,))
            feedback_thread.start()

            # Paso 1: Obtener la imagen con timeout
            timeout = 5  # Tiempo máximo de espera en segundos
            start_time = time()  # Guardar el tiempo de inicio

            imagen_top = None
            imagen_front = None
            while (imagen_top is None) or (imagen_front is None):
                imagen_front = deepcopy(self.cv_img_front)
                imagen_top = deepcopy(self.cv_img_top)

                # Verificar si ha pasado el tiempo de espera
                elapsed_time = time() - start_time
                if elapsed_time > timeout:
                    rospy.logwarn("Timeout al intentar obtener la imagen.")
                    feedback.feedback = -1  # Enviar un valor de feedback para indicar el error
                    self._action_server.publish_feedback(feedback)
                    self._action_server.set_aborted()  # Marcar la acción como fallida
                    return

                rospy.sleep(0.1)
                
            # Paso 2: Procesar la imagen
            resultado_final = FigurasActionResult()
            matriz_front, _ = self.ImageProcessorFrront.process_image(imagen_front)
            matriz_top, _ = self.ImageProcessorTop.process_image(imagen_top)
            
            
            resultado_final.result.figure_3d = self.FigureGenerator.generate_figure_from_matrix(matriz_top, matriz_front)

            # Paso 3: Finalizar la acción con el resultado procesado
            self._action_server.set_succeeded(resultado_final)

            # Esperar a que el hilo de feedback termine
            feedback_thread.join()

    def send_feedback(self, feedback:FigurasActionFeedback):
        ''' 
        Función que utiliza el hilo para enviar el feedback al cliente
            @param feedback (CubosActionFeedback) - Feedback
        '''
        while self._action_server.is_active():
            feedback.feedback = 1
            self._action_server.publish_feedback(feedback)
            rospy.sleep(0.1)

            
            
if __name__ == "__main__":
    FigureMakerActionServer()
