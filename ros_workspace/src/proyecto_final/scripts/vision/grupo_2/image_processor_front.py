import cv2
import numpy as np
from copy import deepcopy

class ImageProcessor_Front:
    ''' 
    Clase que procesa imágenes para detectar y clasificar cubos en función de su color, forma y tamaño, 
    usando técnicas de procesamiento de imágenes como la detección de bordes, la segmentación de contornos 
    y el análisis del color dominante.

    Esta clase realiza los siguientes pasos:
        - Preprocesa la imagen convirtiéndola a escala de grises y aplicando filtros de detección de bordes.
        - Encuentra contornos externos y filtra los contornos según su tamaño.
        - Extrae el color dominante (rojo, verde, azul o amarillo) dentro de los contornos usando el espacio de color HSV.
        - Alinea los puntos detectados en una cuadrícula equidistante de 5x5.
        - Mapea los centros de los cubos a una matriz de 5x5 para representarlos en un formato estructurado.
        - Dibuja los contornos de los cubos sobre la imagen original, con su color asignado.

    Métodos:
        - __init__() - Inicializa el objeto y configura la matriz 5x5 y las variables necesarias.
        - _preprocess_image() - Convierte la imagen a escala de grises y aplica filtros de detección de bordes.
        - _find_external_contours() - Encuentra los contornos externos en la imagen.
        - _filter_contours_by_size() - Filtra los contornos según su tamaño (eliminando los más pequeños).
        - _get_dominant_color() - Obtiene el color dominante dentro de un contorno utilizando el espacio HSV.
        - _align_equidistant() - Alinea los puntos detectados en una cuadrícula equidistante de 5x5.
        - _map_to_matrix() - Mapea las coordenadas de los centros de los cubos a una matriz 5x5.
        - _draw_contours() - Dibuja los contornos de los cubos sobre la imagen.
        - process_image() - Procesa la imagen completa, encuentra contornos y cubos, y los mapea a la matriz.

    Atributos:
        - matrix_size (int) - Tamaño de la matriz 5x5.
        - matrix (numpy array) - Matriz que almacena la ubicación de los cubos detectados.
        - contour_img (numpy array) - Imagen con los contornos de los cubos dibujados.
        - frame (numpy array) - Imagen de entrada que se va a procesar.
    '''
        
    def __init__(self, matrix_size:int=5) -> None:
        ''' 
        Inicializa los atributos de la clase.
        - matrix_size: Tamaño de la matriz cuadrada que representa la cuadrícula de colores (por defecto, 5x5).
        - matrix: Matriz inicializada con valores -1, que se usará para almacenar los colores detectados.
        - contour_img: Imagen utilizada para dibujar contornos detectados.
        - frame: Imagen original que se procesará.  
        @param matrix_size (int) - Tamaño de la matriz cuadrada. Por defecto, 5.
    '''
        self.matrix_size = matrix_size
        self.matrix = np.full((self.matrix_size, self.matrix_size), -1)
        self.contour_img = None
        self.frame = None

        
    
    def _preprocess_image(self) -> np.ndarray:
        """ 
        Convierte la imagen a escala de grises y aplica filtros para mejorar la detección de bordes.
            - Aplica el filtro Sobel para detectar bordes en las direcciones X e Y.
            - Combina las detecciones en una imagen de magnitud.
            - Utiliza el filtro Canny para refinar los bordes detectados.
            - Aplica operaciones morfológicas para limpiar ruido.
            @return morph_clean (numpy array) - Imagen binaria con los bordes detectados y limpiados.
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
        
        # Operaciones morfológicas para limpiar el ruido
        kernel = np.ones((5, 5), np.uint8)
        morph_clean = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        return morph_clean

    def _find_external_contours(self, morph_clean:np.ndarray) -> tuple[list, np.ndarray]:
        '''
        Encuentra los contornos externos en la imagen procesada. Filtra los contornos que tienen jerarquía específica (externos con un padre).
            @param morph_clean (numpy array) - Imagen binaria con bordes detectados.
            @return filtered_contours (list) - Lista de contornos externos filtrados.
            @return contours (list) - Lista completa de contornos detectados.
        '''
        contours, hierarchy = cv2.findContours(morph_clean, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filtrar los contornos externos que tienen un padre
        filtered_contours = []
        for i, (_, _, child, parent) in enumerate(hierarchy[0]):
            if child == -1 and parent != -1:  # Contorno exterior con padre
                filtered_contours.append(contours[i])
        
        return filtered_contours, contours

    def _filter_contours_by_size(self, filtered_contours:list, area_size:int = 2000) -> list:
        ''' 
        Filtra los contornos según su tamaño, eliminando aquellos con un área menor a un umbral.
            @param filtered_contours (list) - Lista de contornos externos filtrados.
            @param area_size (int) - Umbral mínimo de área para considerar un contorno como válido. Por defecto, 2000.
            @return large_contours (list) - Lista de contornos con área mayor al umbral.
        '''
        large_contours = []
        for contour in filtered_contours:
            area = cv2.contourArea(contour)
            if area > area_size:  # Filtrar contornos muy pequeños
                large_contours.append(contour)
        
        return large_contours

    def _get_dominant_color(self, contour:np.ndarray) -> list:
        ''' 
        Determina el color (rojo, verde azul y amarillo) predominante dentro de un contorno basado en el espacio de color HSV.
            @param contour (numpy array) - Contorno del que se quiere extraer el color predominante.
            @return dominant_color (int) - Índice numérico que representa el color predominante (0=Rojo, 1=Verde, 2=Azul, 3=Amarillo).
        '''
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

    def _align_equidistant(self, points:list, side_length:float) -> list:
        ''' 
        Alinea los puntos detectados en una cuadrícula equidistante.
            @param points (list) - Lista de coordenadas de los puntos detectados.
            @param side_length (float) - Longitud de los lados para definir la cuadrícula.
            @return aligned_points_indices (list) - Índices de los puntos alineados en la cuadrícula.
        '''

        # Encontrar los valores mínimo de X y máximo de Y
        min_x = min(points, key=lambda p: p[0])[0]
        max_y = max(points, key=lambda p: p[1])[1]

        # Definir el número de elementos de la cuadrícula
        num_elements = 5

        # Crear las listas de valores equidistantes
        lista_resultado_x = [min_x + side_length * i for i in range(num_elements)]
        lista_resultado_y = [max_y - side_length * i for i in range(num_elements)]

        # Alinear los puntos a la cuadrícula más cercana
        aligned_points_indices = []  # Lista para almacenar los índices de los puntos alineados
        for x, y in points:
            # Obtener los índices del valor más cercano en lista_resultado_x e lista_resultado_y
            aligned_x_index = min(range(len(lista_resultado_x)), key=lambda i: abs(lista_resultado_x[i] - x))
            aligned_y_index = min(range(len(lista_resultado_y)), key=lambda i: abs(lista_resultado_y[i] - y))
            
            aligned_points_indices.append((aligned_x_index, aligned_y_index))
        
        return aligned_points_indices

    def _map_to_matrix(self, centers:list, colors:list, areas:list) -> np.ndarray:
        ''' 
        Mapea los centros detectados y sus colores a una matriz 5x5.
            @param centers (list) - Lista de centros detectados.
            @param colors (list) - Lista de colores correspondientes a los centros.
            @param areas (list) - Lista de áreas de los contornos detectados.
            @return matrix (numpy array) - Matriz 5x5 representando los colores en sus posiciones.
        '''
        # Calcular el área promedio para el umbral
        side_length = round(np.sqrt(sum(areas) / len(areas)))
        
        # Alinear los centros detectados
        aligned_centers = self._align_equidistant(centers, side_length)
        
        # Empaquetar centros con colores
        centers_with_colors_areas = zip(aligned_centers, colors)
        
        
        # Definir la matriz 5x5 vacía
        matrix = np.full((5,5), -1)
    
        # Mapear cada centro a la celda correspondiente
        for center, color in centers_with_colors_areas:
            # Determinar la fila y columna basándose en el tamaño promedio del cubo
            row = 4 - center[1] # Invertir el eje Y
            col = center[0]
            matrix[row][col] = color
        
        return matrix

        

    def _draw_contours(self, img:np.ndarray, contour:np.ndarray, color:tuple=(0, 255, 0), thickness:int=2):
        ''' 
        Dibuja los contornos detectados sobre la imagen original.

            - Calcula un rectángulo delimitador alrededor del contorno.
            - Dibuja el rectángulo sobre la imagen especificada.

            @param img (numpy array) - Imagen sobre la que se dibujarán los contornos.
            @param contour (numpy array) - Contorno detectado a dibujar.
            @param color (tuple) - Color del rectángulo en formato BGR. Por defecto, verde (0, 255, 0).
            @param thickness (int) - Grosor de las líneas del rectángulo. Por defecto, 2.
        '''
        boundRect = cv2.boundingRect(contour)
        cv2.rectangle(img, 
                        (int(boundRect[0]), int(boundRect[1])), 
                        (int(boundRect[0] + boundRect[2]), int(boundRect[1] + boundRect[3])), 
                        color, thickness)
        

    def process_image(self, frame:np.ndarray, area_size:int = 2000, mostrar:bool=False )->tuple:
        ''' 
        Ejecuta el flujo completo de procesamiento de una imagen para detectar colores y posiciones de cubos.
            - Almacena la imagen original y una copia para dibujar contornos.
            - Realiza el preprocesamiento para limpiar la imagen y detectar bordes.
            - Encuentra y filtra los contornos relevantes basándose en su tamaño.
            - Identifica los colores predominantes dentro de cada contorno y calcula las posiciones en una matriz.
            - Dibuja contornos y anotaciones sobre la imagen de salida.
            @param frame (numpy array) - Imagen de entrada en formato BGR.
            @param area_size (int) - Umbral mínimo de área para considerar un contorno como válido. Por defecto, 2000.
            @param mostrar (bool) - Si es True, muestra la imagen procesada.
            @return (tuple) - Una tupla con:
                - matrix (numpy array) - Matriz 5x5 que representa la cuadrícula con los colores detectados.
                - contour_img (numpy array) - Imagen con los contornos y anotaciones dibujados.
        '''
        self.frame = deepcopy(frame)
        self.contour_img = deepcopy(self.frame)

        # Preprocesamiento de la imagen
        morph_clean = self._preprocess_image()
        
        # Encontrar contornos externos
        filtered_contours, _ = self._find_external_contours(morph_clean)
        
        # Filtrar los contornos por tamaño
        large_contours = self._filter_contours_by_size(filtered_contours, area_size)

        # Lista de los centros de los cubos
        centers = []
        colors = []
        areas = []
        colores = {0: (0,0,255), 1: (0,255,0), 2: (255,0,0), 3: (0, 255, 255)}
        
        # Recorrer los contornos filtrados y calcular el color y la posición en la matriz para cada uno
        for contour in large_contours:
            # Obtener el centro del contorno y el color predominante
            rect = cv2.minAreaRect(contour)
            center = (int(rect[0][0]), int(rect[0][1]))
            centers.append(center)
            color = self._get_dominant_color(contour)
            colors.append(color)
            areas.append(cv2.contourArea(contour))

            self._draw_contours(self.contour_img, contour, color=colores.get(color))

            # Visualizar el contorno y el color en la imagen
            cv2.circle(self.contour_img, center, 5, (0, 0, 0), -1)
            cv2.putText(self.contour_img, str(color), (center[0], center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        
        if len(large_contours) > 0:
            self.matrix = self._map_to_matrix(centers, colors, areas)

        if mostrar:
            cv2.imshow('Contoured Image', self.contour_img)

            if cv2.waitKey(0) & 0xFF == ord('q'):
                cv2.destroyAllWindows()

        return self.matrix, self.contour_img


# Ejecutar el programa
if __name__ == "__main__":
    # Crear instancia de ImageProcessor con la ruta de la imagen
    ruta = 'ProyectoFinal\Figuras_Lateral\Figura4_L.png'
    processor = ImageProcessor_Front()
    frame = cv2.imread(ruta)
    processor.process_image(frame, mostrar = True)
