#!/usr/bin/env python3

import rospy
from proyecto_final.msg import HandData
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "1"
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "2"
import cv2
import mediapipe as mp 
import numpy as np

def detectar_gesto(contorno, area, z):
    # Definir AREA_UMBRAL por defecto
    AREA_UMBRAL = 59000  # valor por defecto

    # Ajustar AREA_UMBRAL según la posición z
    if -50.0 < z <= 0.0:
        AREA_UMBRAL = 59000
    elif 0.0 <= z < 192.0:
        AREA_UMBRAL = 90000
    elif -182.0 < z < -50.0:
        AREA_UMBRAL = 39000

    return area > AREA_UMBRAL

def detectar_dislike(contorno):
    try:
        hull = cv2.convexHull(contorno)
        hull_area = cv2.contourArea(hull)
        contour_area = cv2.contourArea(contorno)
        
        if hull_area == 0:
            return False

        solidity = float(contour_area) / hull_area
        
        # Ajustamos los parámetros para una mejor detección del gesto de paz
        if 0.5 <= solidity <= 0.7:
            epsilon = 0.02 * cv2.arcLength(contorno, True)
            approx = cv2.approxPolyDP(contorno, epsilon, True)
            x, y, w, h = cv2.boundingRect(contorno)
            aspect_ratio = float(w) / h
            if 6 <= len(approx) <= 7 and 0.70 <= aspect_ratio <= 0.90:
                return True
        return False
    except:
        return False

def detectar_dino(contorno):
    try:
        hull = cv2.convexHull(contorno)
        hull_area = cv2.contourArea(hull)
        contour_area = cv2.contourArea(contorno)
        
        if hull_area == 0:
            return False

        solidity = float(contour_area) / hull_area
        
        # Ajustamos los parámetros para una mejor detección del gesto de paz
        if 0.6 <= solidity <= 0.73:
            epsilon = 0.02 * cv2.arcLength(contorno, True)
            approx = cv2.approxPolyDP(contorno, epsilon, True)
            x, y, w, h = cv2.boundingRect(contorno)
            aspect_ratio = float(w) / h
            
            if 8 <= len(approx) <= 9 and 1.0 <= aspect_ratio <= 1.46:
                return True
        return False
    except:
        return False
    
def detectar_paz(contorno):
    try:
        hull = cv2.convexHull(contorno)
        hull_area = cv2.contourArea(hull)
        contour_area = cv2.contourArea(contorno)
        
        if hull_area == 0:
            return False

        solidity = float(contour_area) / hull_area
        
        # Ajustamos los parámetros para una mejor detección del gesto de paz
        if 0.67 <= solidity <= 0.75:
            epsilon = 0.02 * cv2.arcLength(contorno, True)
            approx = cv2.approxPolyDP(contorno, epsilon, True)
            x, y, w, h = cv2.boundingRect(contorno)
            aspect_ratio = float(w) / h
            if 7 <= len(approx) <= 8 and 0.40 <= aspect_ratio <= 60:
                return True
        return False
    except:
        return False

def hand_detector():
    rospy.init_node('hand_detector_node', anonymous=True)
    pub = rospy.Publisher('/hand_data', HandData, queue_size=10)

    mp_hands = mp.solutions.hands
    hands1 = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)
    hands2 = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)

    cap1 = cv2.VideoCapture(7)
    cap2 = cv2.VideoCapture(3)

    rate = rospy.Rate(10)
    
    window_name1 = "Camara 1"
    window_name2 = "Camara 2"

    cv2.namedWindow(window_name1)
    cv2.namedWindow(window_name2)

    while not rospy.is_shutdown():
        
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()

        if not ret1:
                return True
        if not ret2:
                return True

        frame1_flip1 = cv2.flip(frame1, 1)
        frame2_flip2 = cv2.flip(frame2, 1)

        rgb_frame1 = cv2.cvtColor(frame1_flip1, cv2.COLOR_BGR2RGB)
        rgb_frame2 = cv2.cvtColor(frame2_flip2, cv2.COLOR_BGR2RGB)

        results1 = hands1.process(rgb_frame1)
        results2 = hands2.process(rgb_frame2)

        # Inicializar hand_data con valores por defecto y mano no detectada
        hand_data = HandData()        
        hand_data.x = 0.0
        hand_data.y = 0.0
        hand_data.z = 0.0
        hand_data.is_open = True  # Por defecto abierta
        hand_data.is_peace = False
        hand_data.is_dino = False
        hand_data.is_dislike = False
        area = 0.0

        if results1.multi_hand_landmarks:
            hand_data.hand_detected = True
            # Procesar cámara 1 para x, y
            for hand_landmarks in results1.multi_hand_landmarks:
                h, w, _ = frame1.shape
                landmarks = [(int(lm.x * w), int(lm.y * h)) for lm in hand_landmarks.landmark]

                x_min = max(0, min(landmarks, key=lambda p: p[0])[0] - 30)
                y_min = max(0, min(landmarks, key=lambda p: p[1])[1] - 30)
                x_max = min(w, max(landmarks, key=lambda p: p[0])[0] + 30)
                y_max = min(h, max(landmarks, key=lambda p: p[1])[1] + 30)

                centro_x = (x_min + x_max) // 2
                centro_y = (y_min + y_max) // 2
                hand_data.x = -1*(float(centro_x - w // 2))
                hand_data.y = float(h // 2 - centro_y)

                # Detección de gestos
                roi = frame1[y_min:y_max, x_min:x_max]
                if roi.size > 0:
                    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                    _, thresh = cv2.threshold(gray_roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    if contours:
                        cnt = max(contours, key=cv2.contourArea)
                        area = cv2.contourArea(cnt)
                        
                        # Detección de gestos usando el área de cámara 1 y z de cámara 2
                        hand_data.is_open = detectar_gesto(cnt, area, hand_data.z)
                        
                        hand_data.is_peace = detectar_paz(cnt)
                        hand_data.is_dino = detectar_dino(cnt)
                        hand_data.is_dislike = detectar_dislike(cnt)
                # Dibujar un círculo verde cuando se detecta la mano
                cv2.circle(frame1, (centro_x, centro_y), 10, (0, 255, 0), -1)
                cv2.putText(frame1, "Mano detectada", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Añadir visualización del estado de la pinza
                estado_texto = "Gesto de paz detectado" if hand_data.is_peace else "Gesto normal"
                cv2.putText(frame1, estado_texto, (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, 
                           (0, 255, 0) if hand_data.is_peace else (0, 0, 255), 2)
                
                # Añadir visualización del estado de la pinza
                estado_texto = "Gesto de dino detectado" if hand_data.is_dino else "Gesto normal"   
                cv2.putText(frame1, estado_texto, (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, 
                           (0, 255, 0) if hand_data.is_dino else (0, 0, 255), 2)
                # Añadir visualización del estado de la pinza
                estado_texto = "Gesto de dislike" if hand_data.is_dislike else "Gesto normal"   
                cv2.putText(frame1, estado_texto, (10, 120), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, 
                           (0, 255, 0) if hand_data.is_dislike else (0, 0, 255), 2)
        
        if results2.multi_hand_landmarks:
            hand_data.hand_detected = True
            # Procesar cámara 2 para z
            for hand_landmarks in results2.multi_hand_landmarks:
                h, w, _ = frame2.shape
                landmarks = [(int(lm.x * w), int(lm.y * h)) for lm in hand_landmarks.landmark]

                x_min = max(0, min(landmarks, key=lambda p: p[0])[0] - 30)
                y_min = max(0, min(landmarks, key=lambda p: p[1])[1] - 30)
                x_max = min(w, max(landmarks, key=lambda p: p[0])[0] + 30)
                y_max = min(h, max(landmarks, key=lambda p: p[1])[1] + 30)
                centro_x = (x_min + x_max) // 2
                centro_y = (y_min + y_max) // 2
                
                # Actualizar z en hand_data
                hand_data.z = float(h // 2 - centro_y)

                # Detección de gestos
                roi = frame2[y_min:y_max, x_min:x_max]
                if roi.size > 0:
                    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                    _, thresh = cv2.threshold(gray_roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    if contours:
                        cnt = max(contours, key=cv2.contourArea)
                        area = cv2.contourArea(cnt)
                        # Actualizar is_open basado en el gesto detectado
                        hand_data.is_open = detectar_gesto(cnt, area, hand_data.z)

                # Dibujar un círculo verde cuando se detecta la mano
                cv2.circle(frame2, (centro_x, centro_y), 10, (0, 255, 0), -1)
                cv2.putText(frame2, "Mano detectada", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame2, f"Z: {hand_data.z:.2f}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        else:
            # Mostrar texto en rojo cuando no se detecta la mano
            cv2.putText(frame1, "No se detecta mano", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow(window_name1, frame1)
        cv2.imshow(window_name2, frame2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        pub.publish(hand_data)
        rate.sleep()
    
    hand_data.is_open = detectar_gesto(cnt, area, hand_data.z)
    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        hand_detector()
    except rospy.ROSInterruptException:
        pass
