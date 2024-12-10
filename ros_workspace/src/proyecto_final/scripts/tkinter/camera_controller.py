#!/usr/bin/python3

<<<<<<< HEAD
import os
# Configurar la variable de entorno para que no aparezcan mensajes de error de index de camara
os.environ["OPENCV_LOG_LEVEL"] = "FATAL"

import cv2
=======
import sys, os, cv2
#export OPENCV_LOG_LEVEL=FATAL
#export OPENCV_LOG_LEVEL=WARNING
colores = {
    "rojo": "\033[31m",      # Rojo
    "verde": "\033[32m",     # Verde
    "amarillo": "\033[33m",  # Amarillo
    "azul": "\033[34m",      # Azul
    "magenta": "\033[35m",   # Magenta
    "cian": "\033[36m",      # Cian
    "blanco": "\033[37m",    # Blanco
    "reset": "\033[0m"       # Restablecer al color predeterminado
}

>>>>>>> 02145a90b86fc69c9843b05f653d880b56d5cde1

colores = {
    "rojo": "\033[31m",      # Rojo
    "verde": "\033[32m",     # Verde
    "amarillo": "\033[33m",  # Amarillo
    "azul": "\033[34m",      # Azul
    "magenta": "\033[35m",   # Magenta
    "cian": "\033[36m",      # Cian
    "blanco": "\033[37m",    # Blanco
    "reset": "\033[0m"       # Restablecer al color predeterminado
}


class CameraController:
    def __init__(self, max_cameras:int):
        self.cameras = []
        self.cameras_index = []
        self.camera_names = []
<<<<<<< HEAD
        self._start(max_cameras)

        #Para silenciar los mensajes de OpenCV


    def _start(self, max_cameras:int):
=======
        self.start(max_cameras)

    def start(self, max_cameras:int):
>>>>>>> 02145a90b86fc69c9843b05f653d880b56d5cde1
        for index in range(max_cameras):
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                self.cameras.append(cap)
                self.cameras_index.append(index)
                self.camera_names.append(f"Cámara {index}")
                print(f"{colores['verde']} --- CAMARA {index} DISPONIBLE ---{colores['reset']}")
            else:
                cap.release()
        
        if self.cameras is None:
            print(f"{colores['rojo']} --- NINGUNA CÁMARA DISPONIBLE ---{colores['reset']}")
            
    def stop(self):
        for cap in self.cameras:
            cap.release()
        
<<<<<<< HEAD
        self.camera = []
=======
        self.cameras = []
        self.cameras_index = []
        self.camera_names = []
>>>>>>> 02145a90b86fc69c9843b05f653d880b56d5cde1

    def get_frame(self, camera_index):
        
        if 0 <= camera_index < len(self.cameras):
            ret, frame = self.cameras[camera_index].read()
            if ret:
                return frame
            else:
                print(f"{colores['rojo']}[ERROR] Failed to read camera{colores['reset']}")
                return None
        else:
            print(f"{colores['rojo']}[ERROR] Camera index out of range{colores['reset']}")
            return None


if __name__ in "__main__":
    controlador = CameraController(10)
<<<<<<< HEAD
    #controlador.get_frame(0)
=======
    
    controlador.get_frame(0)
    sys.stderr = sys.__stderr__
>>>>>>> 02145a90b86fc69c9843b05f653d880b56d5cde1
