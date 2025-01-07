#!/usr/bin/env python3

import rospy
from proyecto_final.msg import HandData
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "1"
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "2"
import cv2
import mediapipe as mp 
import numpy as np

def detectar_dislike(landmarks):
    try:
        # Pulgar hacia abajo, otros dedos cerrados
        pulgar_abajo = landmarks[4].y > landmarks[3].y
        indice_cerrado = landmarks[8].y > landmarks[6].y
        medio_cerrado = landmarks[12].y > landmarks[10].y
        anular_cerrado = landmarks[16].y > landmarks[14].y
        menique_cerrado = landmarks[20].y > landmarks[18].y
        
        return (pulgar_abajo and indice_cerrado and 
                medio_cerrado and anular_cerrado and menique_cerrado)
    except:
        return False

def detectar_dino(landmarks):
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
    
def detectar_paz(landmarks):
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

def detectar_mano_abierta(landmarks):
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

def hand_detector():
    rospy.init_node('hand_detector_node', anonymous=True)
    pub = rospy.Publisher('/hand_data', HandData, queue_size=10)

    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles

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

    cap1 = cv2.VideoCapture(3)
    cap2 = cv2.VideoCapture(7)

    rate = rospy.Rate(10)
    
    window_name1 = "Camara 1"
    window_name2 = "Camara 2"

    cv2.namedWindow(window_name1)
    cv2.namedWindow(window_name2)

    while not rospy.is_shutdown():
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()

        if not ret1 or not ret2:
            return True

        frame1_flip1 = cv2.flip(frame1, 1)
        frame2_flip2 = cv2.flip(frame2, 1)

        # Mejorar la visualización
        frame1 = cv2.resize(frame1_flip1, (640, 480))
        frame2 = cv2.resize(frame2_flip2, (640, 480))

        rgb_frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
        rgb_frame2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2RGB)

        results1 = hands1.process(rgb_frame1)
        results2 = hands2.process(rgb_frame2)

        # Inicializar hand_data
        hand_data = HandData()
        hand_data.hand_detected = False
        hand_data.x = 0.0
        hand_data.y = 0.0
        hand_data.z = 0.0
        hand_data.is_open = False
        hand_data.is_peace = False
        hand_data.is_dino = False
        hand_data.is_dislike = False

        # Procesar cámara 1
        if results1.multi_hand_landmarks:
            hand_data.hand_detected = True
            for hand_landmarks in results1.multi_hand_landmarks:
                # Dibujar landmarks y conexiones
                mp_draw.draw_landmarks(
                    frame1,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style()
                )

                landmarks = hand_landmarks.landmark
                
                # Actualizar coordenadas
                palm_center = landmarks[0]
                hand_data.x = -1 * (palm_center.x * frame1.shape[1] - frame1.shape[1] // 2)
                hand_data.y = frame1.shape[0] // 2 - palm_center.y * frame1.shape[0]

                # Detectar gestos
                hand_data.is_peace = detectar_paz(landmarks)
                hand_data.is_dino = detectar_dino(landmarks)
                hand_data.is_dislike = detectar_dislike(landmarks)
                hand_data.is_open = detectar_mano_abierta(landmarks)

                # Mostrar coordenadas de los dedos
                for id, lm in enumerate(landmarks):
                    h, w, c = frame1.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    # Mostrar puntos clave de los dedos (puntas y articulaciones base)
                    if id in [4, 8, 12, 16, 20]:  # Puntas de los dedos
                        cv2.circle(frame1, (cx, cy), 8, (255, 0, 0), cv2.FILLED)
                    elif id in [3, 7, 11, 15, 19]:  # Articulaciones medias
                        cv2.circle(frame1, (cx, cy), 5, (0, 255, 0), cv2.FILLED)

                # Mostrar información en pantalla
                info_text = [
                    f"Mano: {'Abierta' if hand_data.is_open else 'Cerrada'}",
                    f"Paz: {'Si' if hand_data.is_peace else 'No'}",
                    f"Dino: {'Si' if hand_data.is_dino else 'No'}",
                    f"Dislike: {'Si' if hand_data.is_dislike else 'No'}",
                    f"X: {hand_data.x:.2f}",
                    f"Y: {hand_data.y:.2f}"
                ]

                for i, text in enumerate(info_text):
                    cv2.putText(frame1, text, (10, 30 + i*30),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                              (0, 255, 0), 2)

        # Procesar cámara 2
        if results2.multi_hand_landmarks:
            for hand_landmarks in results2.multi_hand_landmarks:
                # Dibujar landmarks y conexiones
                mp_draw.draw_landmarks(
                    frame2,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style()
                )

                # Calcular coordenada Z
                palm_center = hand_landmarks.landmark[0]
                hand_data.z = frame2.shape[0] // 2 - palm_center.y * frame2.shape[0]

                # Mostrar coordenada Z
                cv2.putText(frame2, f"Z: {hand_data.z:.2f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Mostrar frames
        cv2.imshow("Camara 1 (XY)", frame1)
        cv2.imshow("Camara 2 (Z)", frame2)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        pub.publish(hand_data)
        rate.sleep()

    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        hand_detector()
    except rospy.ROSInterruptException:
        pass
