#!/usr/bin/python3
import rospy
import actionlib
import cv2
import numpy as np
from time import time
from copy import deepcopy
from threading import Thread
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from proyecto_final.vision.grupo_2.image_processor_front import ImageProcessor_Front
from proyecto_final.vision.grupo_2.image_processor_top import ImageProcessor_Top
from proyecto_final.vision.grupo_2.generacion_figura import FigureGenerator
from proyecto_final.msg import FigurasAction, FigurasFeedback, FigurasGoal, FigurasResult
from collections import Counter


class FigureMakerActionServer(object):
    
    def __init__(self):
        # Inicializar nodo
        rospy.init_node('figure_maker_node')

        # --- DEFINICIÓN DE VARIABLES ---
        # Booleanos para activar obtención de imagen
        self.obtain_img_alzado = False
        self.obtain_img_perfil = False
        self.obtain_img_planta = False
        
        # Booleanos para activar obtención de imagen
        self.cv_img_alzado = []
        self.cv_img_perfil = []
        self.cv_img_planta = []
        
        # Procesadores de las imágenes
        self.ImageProcessorAlzado = ImageProcessor_Front()
        self.ImageProcessorPerfil = ImageProcessor_Front()
        self.ImageProcessorPlanta = ImageProcessor_Top()
        
        # Procesador de la figura
        self.FigureGenerator = FigureGenerator()
        
        # Objeto bridge CV
        self.bridge = CvBridge()
        
        # Definicion de variables ROS
        self.subs_cam_perfil = rospy.Subscriber('/alzado_cam/image_raw', Image, self.cb_image_alzado)
        self.subs_cam_perfil = rospy.Subscriber('/perfil_cam/image_raw', Image, self.cb_image_perfil)
        self.subs_cam_planta = rospy.Subscriber('/top_cam/image_raw', Image, self.cb_image_planta)
        
        # Action Server
        self.action_server = actionlib.SimpleActionServer('FigureMakerActionServer', FigurasAction, execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()

        rospy.spin()
    
    def cb_image_alzado(self, image:Image)->None:
        ''' 
        Callback del subscriptor de la cámara.
            @param image (Image) - Imagen de la camara
        '''
        if self.obtain_img_alzado:
            if len(self.cv_img_alzado) < 11:
                img = deepcopy(self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough'))
                img = deepcopy(cv2.cvtColor(img, cv2.COLOR_RGB2BGR)) #Convertir imagen a BGR
                self.cv_img_alzado.append(img)
            else:
                self.obtain_img_alzado = False
        
    
    def cb_image_perfil(self, image2:Image)->None:
        ''' 
        Callback del subscriptor de la cámara.
            @param image (Image) - Imagen de la camara
        '''
        if self.obtain_img_perfil:
            if len(self.cv_img_perfil) < 11:
                img2 = deepcopy(self.bridge.imgmsg_to_cv2(image2, desired_encoding='passthrough'))
                img2 = deepcopy(cv2.cvtColor(img2, cv2.COLOR_RGB2BGR)) #Convertir imagen a BGR
                self.cv_img_perfil.append(img2)
            else:
                self.obtain_img_perfil = False
    
    def cb_image_planta(self, image1:Image)->None:
        ''' 
        Callback del subscriptor de la cámara.
            @param image (Image) - Imagen de la camara
        '''
        if self.obtain_img_planta:
            if len(self.cv_img_planta) < 11:
                img1 = deepcopy(self.bridge.imgmsg_to_cv2(image1, desired_encoding='passthrough'))
                img1 = deepcopy(cv2.cvtColor(img1, cv2.COLOR_RGB2BGR)) #Convertir imagen a BGR
                self.cv_img_planta.append(img1)
            else:
                self.obtain_img_planta = False

    def execute_cb(self, goal:FigurasGoal)->None:
        ''' 
        Callback del action server del CubeTracker
            @param goal (numpy array) - Goal recibido por el cliente
        '''
        if goal.order == 1:
            self.obtain_img_alzado = self.obtain_img_perfil = self.obtain_img_planta = True
            
            # Crear instacia de feedback
            feedback = FigurasFeedback()

            # Crear y ejecutar el hilo de feedback
            feedback_thread = Thread(target=self.send_feedback, args=(feedback,))
            feedback_thread.start()

            # --- PASO 1: Obtener las imagenes con timeout ---
            timeout = 5  # Tiempo máximo de espera en segundos
            start_time = time()  # Guardar el tiempo de inicio
            while (self.obtain_img_alzado) or (self.obtain_img_perfil) or (self.obtain_img_planta): 
                # Verificar si ha pasado el tiempo de espera
                elapsed_time = time() - start_time
                if elapsed_time > timeout:
                    rospy.logwarn("Timeout al intentar obtener la imagen.")
                    feedback.feedback = -1  # Enviar un valor de feedback para indicar el error
                    self.action_server.publish_feedback(feedback)
                    self.action_server.set_aborted()  # Marcar la acción como fallida
                    return

                rospy.sleep(0.1)
            
            L_img_alzado = deepcopy(self.cv_img_alzado)
            L_img_perfil = deepcopy(self.cv_img_perfil)
            L_img_planta = deepcopy(self.cv_img_planta)
            
            # cv2.imshow("Alzado", L_img_alzado[0])
            # cv2.imshow("Alzado", L_img_perfil[0])
            # cv2.imshow("Alzado", L_img_planta[0])

            # --- PASO 2: Procesar cada una de las imagenes ---
            L_matrix_alzado = []
            L_matrix_perfil = []
            L_matrix_planta = []
            for i in range(10):
                matrix, img_alzado = self.ImageProcessorAlzado.process_image(L_img_alzado[i])
                L_matrix_alzado.append(deepcopy(matrix))
                
                matrix, img_perfil = self.ImageProcessorPerfil.process_image(L_img_perfil[i])
                L_matrix_perfil.append(deepcopy(matrix))
                
                matrix, img_planta = self.ImageProcessorPlanta.process_image(L_img_planta[i])
                L_matrix_planta.append(deepcopy(matrix))
            
            cv2.imwrite("alzado.png", img_alzado)
            cv2.imwrite("perfil.png", img_perfil)
            cv2.imwrite("planta.png", img_planta)
            
            
            
            # --- PASO 3: Obtener la matriz más frecuente ---
            matrix_alzado = self._matriz_frecuente(L_matrix_alzado)
            matrix_perfil = self._matriz_frecuente(L_matrix_perfil)
            matrix_planta = self._matriz_frecuente(L_matrix_planta)
            print(f"ALZADO:\n{matrix_alzado}")
            print(f"\n\nPERFIL:\n{matrix_perfil}")
            print(f"\n\nPLANTA:\n{matrix_planta}")
            
            
            
            # --- PASO 4: Obtener la figura 3D ---
            resultado_final = FigurasResult()
            figura_3d = self.FigureGenerator.generate_figure_from_matrix(matrix_planta, matrix_alzado, matrix_perfil)
            figura_3d = figura_3d.astype(np.int8)
            resultado_final.shape_3d = figura_3d.shape
            resultado_final.figure_3d = figura_3d.flatten().tolist()
        
            # --- PASO 5: Enviar el resultado ---
            self.action_server.set_succeeded(resultado_final)

            # --- PASO 6: Esperar union del hilo ---
            feedback_thread.join()
            self.cv_img_alzado  = []
            self.cv_img_perfil = []
            self.cv_img_planta = []
    
    def _matriz_frecuente(self, lista_matrices:list)->np.ndarray:
        matrices_tuple = [tuple(map(tuple, matrix.tolist())) for matrix in lista_matrices]

        # Contamos las ocurrencias de cada matriz
        counter = Counter(matrices_tuple)

        # Obtenemos la matriz más frecuente (la que más se repite)
        most_common_matrix_tuple, frequency = counter.most_common(1)[0]

        # Convertimos la tupla de vuelta a una matriz numpy
        return np.array(most_common_matrix_tuple)
    
    def send_feedback(self, feedback:FigurasFeedback):
        ''' 
        Función que utiliza el hilo para enviar el feedback al cliente
            @param feedback (CubosActionFeedback) - Feedback
        '''
        while self.action_server.is_active():
            feedback.feedback = 1
            self.action_server.publish_feedback(feedback)
            rospy.sleep(0.1)

            
            
if __name__ == "__main__":
    FigureMakerActionServer()
