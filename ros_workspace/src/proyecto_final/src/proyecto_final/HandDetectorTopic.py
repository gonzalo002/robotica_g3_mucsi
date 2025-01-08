#!/usr/bin/env python3

import cv2, os, rospy
from copy import deepcopy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from proyecto_final.msg import HandData
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "1"
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "2"
import mediapipe as mp 

    
class HandDetector():
    def __init__(self):
        rospy.init_node('hand_detector_node')

        self.cv_img_top = self.cv_img_lateral = []
        
        self.bridge = CvBridge()
        self.subs_cam_top = rospy.Subscriber('/top_hand_cam/image_raw', Image, self.cb_image_top)
        self.subs_cam_lateral = rospy.Subscriber('/lateral_hand_cam/image_raw', Image, self.cb_image_lateral)
        self.pub_hand_data = rospy.Publisher('/hand_data', HandData, queue_size=10)

        self.hand_detector()


    def detectar_dino(self,landmarks):
        try:
            # Solo índice extendido, otros dedos cerrados
            indice_extendido = landmarks[8].y < landmarks[6].y
            medio_cerrado = landmarks[12].y > landmarks[10].y
            anular_cerrado = landmarks[16].y > landmarks[14].y
            menique_cerrado = landmarks[20].y > landmarks[18].y
            pulgar_cerrado = landmarks[4].x > landmarks[3].x
            
            return (indice_extendido and medio_cerrado and 
                    anular_cerrado and menique_cerrado and pulgar_cerrado)
        except:
            return False
        
    def detectar_paz(self,landmarks):
        # Verificar posiciones específicas de los dedos usando MediaPipe
        try:
            # Índice y medio extendidos, otros dedos cerrados
            indice_extendido = landmarks[8].y < landmarks[6].y
            medio_extendido = landmarks[12].y < landmarks[10].y
            anular_cerrado = landmarks[16].y > landmarks[14].y
            menique_cerrado = landmarks[20].y > landmarks[18].y
            pulgar_cerrado = landmarks[4].x > landmarks[3].x
            
            return (indice_extendido and medio_extendido and 
                    anular_cerrado and menique_cerrado and pulgar_cerrado)
        except:
            return False

    def detectar_mano_abierta(self,landmarks):
        try:
            # Todos los dedos extendidos
            indice_extendido = landmarks[8].y < landmarks[6].y
            medio_extendido = landmarks[12].y < landmarks[10].y
            anular_extendido = landmarks[16].y < landmarks[14].y
            menique_extendido = landmarks[20].y < landmarks[18].y
            pulgar_extendido = landmarks[4].x < landmarks[3].x  # Para mano derecha
            
            return (indice_extendido and medio_extendido and 
                    anular_extendido and menique_extendido and pulgar_extendido)
        except:
            return False

    def cb_image_top(self, image:Image)->None:
        ''' 
        Callback del subscriptor de la cámara.
            @param image (Image) - Imagen de la camara
        '''

        self.cv_img_top = deepcopy(self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough'))
        self.cv_img_top = deepcopy(cv2.cvtColor(self.cv_img_top, cv2.COLOR_RGB2BGR))

    def cb_image_lateral(self, image:Image)->None:
        ''' 
        Callback del subscriptor de la cámara.
            @param image (Image) - Imagen de la camara
        '''

        self.cv_img_lateral = deepcopy(self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough'))
        self.cv_img_lateral = deepcopy(cv2.cvtColor(self.cv_img_lateral, cv2.COLOR_RGB2BGR))


    def hand_detector(self):
        mp_hands = mp.solutions.hands

        # Configurar MediaPipe con mayor confianza de detección
        hands1 = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )
        hands2 = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )


        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if len(self.cv_img_lateral)!=0 and len(self.cv_img_top)!=0:
                img_top = deepcopy(self.cv_img_top)
                img_lateral = deepcopy(self.cv_img_lateral)

                frame1_flip1 = cv2.flip(img_top, 1)
                frame2_flip2 = cv2.flip(img_lateral, 1)

                # Mejorar la visualización
                frame1 = cv2.resize(frame1_flip1, (640, 480))
                frame2 = cv2.resize(frame2_flip2, (640, 480))

                rgb_frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
                rgb_frame2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2RGB)

                results1 = hands1.process(rgb_frame1)
                results2 = hands2.process(rgb_frame2)

                # Inicializar hand_data
                hand_data = HandData()

                # Procesar cámara 1
                if results1.multi_hand_landmarks:
                    hand_data.hand_detected = True
                    for hand_landmarks in results1.multi_hand_landmarks:

                        landmarks = hand_landmarks.landmark
                        
                        # Actualizar coordenadas
                        palm_center = landmarks[0]
                        hand_data.x = -1 * (palm_center.x * frame1.shape[1] - frame1.shape[1] // 2)
                        hand_data.y = frame1.shape[0] // 2 - palm_center.y * frame1.shape[0]

                        # Detectar gestos
                        hand_data.is_peace = self.detectar_paz(landmarks)
                        hand_data.is_dino = self.detectar_dino(landmarks)
                        hand_data.is_open = self.detectar_mano_abierta(landmarks)



                # Procesar cámara 2
                if results2.multi_hand_landmarks:
                    for hand_landmarks in results2.multi_hand_landmarks:
                        # Calcular coordenada Z
                        palm_center = hand_landmarks.landmark[0]
                        hand_data.z = frame2.shape[0] // 2 - palm_center.y * frame2.shape[0]


                self.pub_hand_data.publish(hand_data)
                self.cv_img_lateral = self.cv_img_top = deepcopy([])
            rate.sleep()


if __name__ == '__main__':
    hand = HandDetector()
