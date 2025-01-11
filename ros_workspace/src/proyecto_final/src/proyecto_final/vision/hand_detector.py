#!/usr/bin/env python3

import mediapipe as mp
import cv2

from copy import deepcopy
from typing import List
from numpy import ndarray

from proyecto_final.msg import HandData

class HandDetector:
    def __init__(self):
        self.landmarks = []
        self.hands1 = None
        self.hands2 = None

        self.hand_data = HandData()

        self.define_hands()
      
    def detect_gesture(self) -> None:
        try:
            indice_extendido = self.landmarks[8].y < self.landmarks[6].y
            medio_extendido = self.landmarks[12].y < self.landmarks[10].y
            anular_extendido = self.landmarks[16].y < self.landmarks[14].y
            menique_extendido = self.landmarks[20].y < self.landmarks[18].y
            pulgar_extendido = self.landmarks[4].x < self.landmarks[3].x  # Para mano derecha
            pulgar_abajo = self.landmarks[4].y > self.landmarks[3].y

        except:
            return None
        
        if indice_extendido and medio_extendido and anular_extendido \
            and menique_extendido and pulgar_extendido:
            self.hand_data.is_open = True
        
        elif indice_extendido and not medio_extendido and not anular_extendido \
            and not menique_extendido and not pulgar_extendido:
            self.hand_data.is_dino = True
        
        elif indice_extendido and medio_extendido and not anular_extendido \
            and not menique_extendido and not pulgar_extendido:
            self.hand_data.is_peace = True
        
        elif not indice_extendido and not medio_extendido and not anular_extendido \
            and not menique_extendido and pulgar_abajo:
            self.hand_data.is_dislike = True
        
        
    def define_hands(self) -> None:
        mp_hands = mp.solutions.hands

        # Configurar MediaPipe con mayor confianza de detección
        self.hands1 = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )
        self.hands2 = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

    def detect_hand(self, img_top:ndarray, img_lateral:ndarray) -> None:   
            img_top = deepcopy(img_top)
            img_lateral = deepcopy(img_lateral)

            frame1_flip1 = cv2.flip(img_top, 1)
            frame2_flip2 = cv2.flip(img_lateral, 1)

            # Mejorar la visualización
            frame1 = cv2.resize(frame1_flip1, (640, 480))
            frame2 = cv2.resize(frame2_flip2, (640, 480))

            rgb_frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
            rgb_frame2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2RGB)

            results1 = self.hands1.process(rgb_frame1)
            results2 = self.hands2.process(rgb_frame2)

            # Inicializar hand_data
            self.hand_data = HandData()

            # Procesar cámara 1
            if results1.multi_hand_landmarks:
                self.hand_data.hand_detected = True
                for hand_landmarks in results1.multi_hand_landmarks:

                    landmarks = hand_landmarks.landmark
                    
                    # Actualizar coordenadas
                    palm_center = landmarks[0]
                    self.hand_data.x = -1 * (palm_center.x * frame1.shape[1] - frame1.shape[1] // 2)
                    self.hand_data.y = frame1.shape[0] // 2 - palm_center.y * frame1.shape[0]

            # Procesar cámara 2
            if results2.multi_hand_landmarks:
                for hand_landmarks in results2.multi_hand_landmarks:
                    # Calcular coordenada Z
                    palm_center = hand_landmarks.landmark[0]
                    self.hand_data.z = frame2.shape[0] // 2 - palm_center.y * frame2.shape[0]