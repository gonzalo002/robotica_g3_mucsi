#!/usr/bin/python3

import os
# Configurar la variable de entorno para que no aparezcan mensajes de error de index de camara
os.environ["OPENCV_LOG_LEVEL"] = "FATAL"

import ttkbootstrap as ttk
from ttkbootstrap.constants import *
from tkinter.font import Font
from tkinter import filedialog, IntVar
from PIL import Image, ImageTk, Image, ImageDraw, ImageFont
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
from PIL.ImageTk import PhotoImage
import cv2

from image_processor_top import ImageProcessor_Top
from image_processor_front import ImageProcessor_Front
from cube_tracker import CubeTracker
from camera_controller import CameraController
from geometry import FigureGenerator


class RoboticaTab:
    def __init__(self, tab) -> None:
        self.tab = tab
        pass
    
    def robot_tab(self):
        # Crear un Frame para organizar los elementos dentro de la pestaña "Procesado de Figura"
        self.frame_rob = ttk.Frame(self.tab, borderwidth=0)
        self.frame_rob.pack(fill="both", expand=True)

        #Configuración grid
        self.frame_vision.grid_columnconfigure(0, weight=2)
        self.frame_vision.grid_columnconfigure(1, weight=1)
        self.frame_vision.grid_columnconfigure(2, weight=4)
        
        # --- CHECK BUTTON ---
        self.camera_check_var = ttk.IntVar()  # Variable para el estado del Checkbutton
        self.camera_check = ttk.Checkbutton(
            self.frame_vision,
            text="Usar Cámara",
            variable=self.camera_check_var,
            
            command=self.toggle_mode,  # Función para manejar el cambio
        )
        self.camera_check.grid(row=0, column=0, sticky="w", padx=10, pady=10)
