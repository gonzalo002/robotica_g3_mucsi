import cv2
import numpy as np
from copy import deepcopy

class ImageProcessor_Frontal:
    def __init__(self): 
        self.matrix_size = 5
        self.matrix = np.full((self.matrix_size, self.matrix_size), -1)
        self.frame = None
        self.contour_img = None
    
    def preprocess_image(self):
        """
        Convierte la imagen a escala de grises y aplica filtros para detectar bordes.
        """
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        
        # Aplicar el filtro Sobel para detección de bordes en los ejes X y Y
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        
        # Combinación de los bordes detectados
        sobel_combined = cv2.magnitude(sobelx, sobely)
        sobel_combined = np.uint8(sobel_combined)
        
        # Aplicar el filtro Canny para una detección de bordes más refinada
        edges = cv2.Canny(sobel_combined, 50, 200)
        cv2.imshow('Canny Filter', sobel_combined)
        
        # Operaciones morfológicas para limpiar el ruido
        kernel = np.ones((5, 5), np.uint8)
        morph_clean = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        return morph_clean

    def find_external_contours(self, morph_clean):
        """
        Encuentra los contornos externos y su jerarquía.
        """
        contours, hierarchy = cv2.findContours(morph_clean, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filtrar los contornos externos que tienen un padre
        filtered_contours = []
        for i, (_, _, child, parent) in enumerate(hierarchy[0]):
            if child == -1 and parent != -1:  # Contorno exterior con padre
                filtered_contours.append(contours[i])
        
        return filtered_contours, contours

    def filter_contours_by_size(self, filtered_contours):
        """
        Filtra los contornos por tamaño, eliminando los más pequeños.
        """
        large_contours = []
        for contour in filtered_contours:
            area = cv2.contourArea(contour)
            if area > 6000:  # Filtrar contornos muy pequeños
                large_contours.append(contour)
        
        return large_contours

    def get_dominant_color(self, contour):
        """
        Obtiene el color predominante (rojo, verde, azul, amarillo) dentro del contorno,
        usando el espacio de color HSV y los rangos definidos.
        """
        # Definir los rangos de colores en el espacio HSV
        color_ranges = {
            "Red": [(0, 100, 100), (15, 255, 255)],
            "Green": [(40, 50, 50), (80, 255, 255)],
            "Blue": [(100, 100, 100), (130, 255, 255)],
            "Yellow": [(20, 100, 100), (30, 255, 255)]
        }
        
        # Convertir la imagen BGR a HSV
        hsv_image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        
        # Crear una máscara para cada color en el diccionario
        color_counts = {color: 0 for color in color_ranges}
        
        # Crear una máscara para la región definida por el contorno
        mask = np.zeros(self.frame.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, (255), thickness=cv2.FILLED)
        
        # Recorrer cada rango de color y contar los píxeles que están dentro del rango
        for color, (lower, upper) in color_ranges.items():
            lower_bound = np.array(lower)
            upper_bound = np.array(upper)
            
            # Crear la máscara para el color
            color_mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
            
            # Aplicar la máscara al área del contorno
            color_mask = cv2.bitwise_and(color_mask, color_mask, mask=mask)
            
            # Contar la cantidad de píxeles del color
            color_counts[color] = cv2.countNonZero(color_mask)
        
        # Determinar el color predominante
        dominant_color = max(color_counts, key=color_counts.get)
        
        # Asignar un valor numérico a cada color
        color_map = {"Red": 0, "Green": 1, "Blue": 2, "Yellow": 3}
        
        return color_map[dominant_color]

    def map_to_matrix(self, centers, colors):
        """
        Mapea las coordenadas del centro a una matriz 5x5, en base a los límites de los cubos.
        """
        # Ordenar los centros por coordenada Y (fila) de abajo hacia arriba
        centers_with_colors = sorted(zip(centers, colors), key=lambda x: (-x[0][1], x[0][0]))

        # Definir
        row = 4  # Empezar desde la última fila (inferior)
        col = 0
        current_row_y = centers_with_colors[0][0][1]

        for center, color in centers_with_colors:
            
            if abs(center[1] - current_row_y) > 10:
                row -= 1
                col = 0  
                current_row_y = center[1] 

            
            self.matrix[row][col] = color
            col += 1  # Pasar a la siguiente columna

    def draw_contours(self, img, contour, color=(0, 255, 0), thickness=2):
        """
        Dibuja los contornos sobre la imagen.
        """
        boundRect = cv2.boundingRect(contour)
        cv2.rectangle(img, 
                        (int(boundRect[0]), int(boundRect[1])), 
                        (int(boundRect[0] + boundRect[2]), int(boundRect[1] + boundRect[3])), 
                        color, thickness)

    def process_image(self, frame):
        """
        Ejecuta el procesamiento completo de la imagen.
        """
        self.frame = deepcopy(frame)
        self.contour_img = deepcopy(frame)

        # Preprocesamiento de la imagen
        morph_clean = self.preprocess_image()
        
        # Encontrar contornos externos
        filtered_contours, _ = self.find_external_contours(morph_clean)
        
        # Filtrar los contornos por tamaño
        large_contours = self.filter_contours_by_size(filtered_contours)

        # Lista de los centros de los cubos
        centers = []
        colors = []
        colores = {0: (0,0,255), 1: (0,255,0), 2: (255,0,0), 3: (0, 255, 255)}
        
        # Recorrer los contornos filtrados y calcular el color y la posición en la matriz para cada uno
        for contour in large_contours:
            # Obtener el centro del contorno y el color predominante
            rect = cv2.minAreaRect(contour)
            center = (int(rect[0][0]), int(rect[0][1]))
            centers.append(center)
            color = self.get_dominant_color(contour)
            colors.append(color)

            self.draw_contours(self.contour_img, contour, color=colores.get(color))

            # Visualizar el contorno y el color en la imagen
            cv2.circle(self.contour_img, center, 5, (0, 0, 0), -1)
            cv2.putText(self.contour_img, str(color), (center[0], center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        

        self.map_to_matrix(centers, colors)

        # Mostrar imágenes
        cv2.imshow('Morphological Clean', morph_clean)
        cv2.imshow('Contornos Filtrados', self.contour_img)
        cv2.imshow('Frame original', self.frame)

        print("Matriz de colores:")
        # print(self.matrix)
        # Mostrar la matriz de colores
        for row in self.matrix:
            print(row)

        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return self.matrix


# Ejecutar el programa
if __name__ == "__main__":
    # Crear instancia de ImageProcessor con la ruta de la imagen
    processor = ImageProcessor_Frontal()
    frame = cv2.imread('src/proyecto_final/scripts/vision/Imagenes/Imagen_75.png')
    processor.process_image(frame)
