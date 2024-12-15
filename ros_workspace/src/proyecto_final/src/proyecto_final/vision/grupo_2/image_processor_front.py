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
        - _get_cubes_location() - Recorta la imagen respecto a las máscaras de contraste obtenidas.
        - _get_contrast_image() - Obtiene la localización de los cubos utilizando máscaras de color HSV.
        - _filter_contours() - Filtra los contornos según su tamaño (eliminando los más pequeños).
        - _get_color() - Obtiene el color de un contorno en base a las máscaras obtenidas previamente.
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
        self.debug = False

        self.filtered_colors = []

        
    
    def _preprocess_image(self) -> np.ndarray:
        """ 
        Convierte la imagen a escala de grises y aplica filtros para mejorar la detección de bordes.
            - Aplica el filtro Sobel para detectar bordes en las direcciones X e Y.
            - Combina las detecciones en una imagen de magnitud.
            - Utiliza el filtro Canny para refinar los bordes detectados.
            - Aplica operaciones morfológicas para limpiar ruido.
                @return morph_clean (numpy array) - Imagen binaria con los bordes detectados y limpiados.
        """

        self.filtered_colors = self._get_contrast_img(self.frame)

        cropper_frame = self._get_cubes_location(self.filtered_colors)
        
        gray = cv2.cvtColor(cropper_frame, cv2.COLOR_BGR2GRAY)

        # Aplicar el filtro Sobel para detección de bordes en los ejes X y Y
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=5)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=5)

        # Combinación de los bordes detectados
        sobel_combined = cv2.magnitude(sobelx, sobely)
        sobel_combined = np.uint8(sobel_combined)

        _, sobel_umbralized = cv2.threshold(sobel_combined, 150, 255, cv2.THRESH_BINARY)
        
        # Operaciones morfológicas para limpiar el ruido
        kernel = np.ones((7, 7), np.uint8)
        sobel_umbralized = cv2.morphologyEx(sobel_umbralized, cv2.MORPH_CLOSE, kernel)

        new_gray = cv2.bitwise_or(gray, gray, mask=cv2.bitwise_not(sobel_umbralized))

        _, new_gray = cv2.threshold(new_gray, 2, 255, cv2.THRESH_BINARY)
        
        kernel = np.ones((5, 5), np.uint8)
        new_gray = cv2.morphologyEx(new_gray, cv2.MORPH_CLOSE, kernel)

        # Aplicar el filtro Canny para una detección de bordes más refinada
        edges = cv2.Canny(new_gray, 20, 200)

        if self.debug:
            cv2.imshow('Sobel Combined', sobel_combined)
            cv2.imshow('Sobel Umbralized', sobel_umbralized)
            cv2.imshow('New Gray', new_gray)
            cv2.imshow('Canny', edges)
        
        return new_gray

    def _get_cubes_location(self, constrast_images:list) -> np.ndarray:
        """ 
        Obtiene la ubicación de los cubos en la imagen utilizando las imágenes de contraste.
            - Combina las máscaras de colores para identificar las áreas donde se encuentran los cubos.
            @return result (numpy array) - Imagen con los cubos localizados.
        """
        mask = np.zeros_like(self.frame[:,:,0]) 

        mask_combined = cv2.bitwise_or(mask, constrast_images[0])  
        mask_combined = cv2.bitwise_or(mask_combined, constrast_images[1])  
        mask_combined = cv2.bitwise_or(mask_combined, constrast_images[2])  
        mask_combined = cv2.bitwise_or(mask_combined, constrast_images[3])  

        result = cv2.bitwise_and(self.frame, self.frame, mask=mask_combined)

        if self.debug:
            cv2.imshow('Cube Location Mask', mask_combined)
            cv2.imshow('Cube Location', result)

        return result

    def _get_contrast_img(self, frame:np.ndarray) -> list:
        ''' 
        Obtiene las imágenes de contraste para cada color (rojo, verde, azul, amarillo) en el espacio HSV.
            @param frame (numpy array) - Imagen de entrada para el análisis de colores.
            @return filtered_images (list) - Lista de imágenes filtradas por color.
        '''
        # Convertir la imagen a HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   
        height, width = hsv.shape[:2]
        # Definir los rangos de colores en HSV

        # Verde: Hue entre 35 y 85, Saturación y Valor altos
        lower_green = np.array([30, 150, 0])   # Mínimo verde
        upper_green = np.array([110, 255, 255]) # Máximo verde

        # Rojo: Hue entre 0 y 10, o entre 170 y 180, Saturación y Valor altos
        lower_red1 = np.array([0, 70, 70], np.uint8)    # Rojo (primer rango) (0, 100, 100), (15, 255, 255)
        upper_red1 = np.array([20, 255, 255], np.uint8)  # Rojo (primer rango)

        # Rango superior para el rojo (170-180 grados)
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        # Azul: Hue entre 100 y 140, Saturación y Valor altos
        lower_blue = np.array([100, 135, 115], np.uint8)   # Mínimo azul
        upper_blue = np.array([130, 255, 255], np.uint8) # Máximo azul

        # Amarillo: Hue entre 20 y 40, Saturación y Valor altos
        lower_yellow = np.array([20, 70, 30], np.uint8)  # Mínimo amarillo
        upper_yellow = np.array([60, 255, 255], np.uint8)# Máximo amarillo

        lower_values = [lower_red1, lower_green, lower_blue, lower_yellow, lower_red2]
        upper_values = [upper_red1, upper_green, upper_blue, upper_yellow, upper_red2]
        filtered_images:list = []
        
        for i in range(len(lower_values)):
            # Aplicar un umbral de valor mínimo para eliminar el fondo negro (valor bajo)
            mask = cv2.inRange(hsv, lower_values[i], upper_values[i]) 
            value_threshold = 70  # Umbral mínimo de valor (a partir de este valor consideramos los colores)
            value_mask = hsv[:, :, 2] > value_threshold  # Solo seleccionamos los píxeles con valor mayor que el umbral
            value_mask = np.uint8(value_mask) * 255
            mask = cv2.bitwise_and(mask, mask, mask=value_mask)

            filtered_image = (cv2.bitwise_and(frame, frame, mask=mask))
            filtered_images.append(filtered_image)
        
        filtered_images[0] = cv2.bitwise_or(filtered_images[0], filtered_images[4])
        filtered_images.pop(4)

        contrast_images = []
        kernel = np.ones((5, 5), np.uint8)
        for i in range(len(filtered_images)):
            alpha = 1.3
            beta = 1

            contrast_img = cv2.convertScaleAbs(filtered_images[i], alpha=alpha, beta=beta) 
            contrast_img = cv2.morphologyEx(contrast_img, cv2.MORPH_CLOSE, kernel)
            contrast_images.append(contrast_img)
        
        gray_r = cv2.cvtColor(contrast_images[0], cv2.COLOR_BGR2GRAY)
        _, r_inv = cv2.threshold(gray_r, 50, 255, cv2.THRESH_BINARY)

         # Invertir la imagen amarilla
        gray_g = cv2.cvtColor(contrast_images[1], cv2.COLOR_BGR2GRAY)
        _, g_inv = cv2.threshold(gray_g, 50, 255, cv2.THRESH_BINARY)


        # Invertir la imagen azul
        gray_b = cv2.cvtColor(contrast_images[2], cv2.COLOR_BGR2GRAY)
        _, b_inv = cv2.threshold(gray_b, 30, 255, cv2.THRESH_BINARY)

        # Invertir la imagen amarilla
        gray_y = cv2.cvtColor(contrast_images[3], cv2.COLOR_BGR2GRAY)
        _, y_inv = cv2.threshold(gray_y, 50, 255, cv2.THRESH_BINARY)                    
             
        resultado= [r_inv, g_inv, b_inv, y_inv]
        
        kernel = np.ones((5, 5), np.uint8)

        for i, img in enumerate(resultado):
            resultado[i] = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = np.zeros_like(resultado[i])

            contours, _ = cv2.findContours(resultado[i], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                # Obtener el rectángulo delimitador del contorno (x, y, w, h)
                x, y, w, h = cv2.boundingRect(contour)
                
                # Hacer el rectángulo ligeramente más pequeño
                reduction = 10  # Cuánto reducir el tamaño
                x_small = x + reduction
                y_small = y + reduction
                w_small = w - 2 * reduction
                h_small = h - 2 * reduction
                
                try:
                    # Dibujar el cuadrado más pequeño dentro del rectángulo delimitador
                    cv2.rectangle(mask, (x_small, y_small), (x_small + w_small, y_small + h_small), 255, -1)
                except:
                    pass
            resultado[i] = mask

            
        if self.debug:
             cv2.imshow('Red Only', resultado[0])
             cv2.imshow('Green Only', resultado[1])
             cv2.imshow('Blue Only', resultado[2])
             cv2.imshow('Yellow Only', resultado[3])
             
        return resultado


    def _filter_contours(self, contours:list) -> list:
        ''' 
        Filtra los contornos para eliminar aquellos que son demasiado pequeños.
            @param contours (list) - Lista de contornos detectados en la imagen.
            @return filtered_contours (list) - Lista de contornos que pasan el filtro.
        '''
        correct_contour = []
        area_size = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                correct_contour.append(cnt)
                area_size.append(area)

        mode_cubes = np.median(area_size)
        large_contours = []
        for contour in correct_contour:
                
            large_contours.append(contour) # Sin filtrar por cuadraticidad

        if self.debug:
            print(f'Areas : {area_size}')
            print(f'Media Areas: {mode_cubes}')
        
        return correct_contour

    def _get_color(self, center:list) -> int:
        ''' 
        Obtiene el color dominante de un contorno, basado en las máscaras de color preprocesadas.
            @param contour (list) - Un contorno detectado en la imagen.
            @return color (int) - El color detectado como un valor de índice (0: Rojo, 1: Verde, 2: Azul, 3: Amarillo).
        '''
        for i, color in enumerate(self.filtered_colors):
            if color[center[1],center[0]] > 20:
                return i
        return 4

    def _align_equidistant(self, points:list, side_length:float) -> list:
        ''' 
        Alinea los puntos detectados a una cuadrícula equidistante.
            @param points (numpy array) - Lista de puntos detectados en la imagen.
            @return aligned_points (numpy array) - Puntos alineados en una cuadrícula equidistante.
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
        Mapea los puntos detectados a una matriz de tamaño 5x5.
            @param points (numpy array) - Lista de puntos detectados.
            @return matrix (numpy array) - Matriz de 5x5 con los índices de color correspondientes.
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

        

    def _draw_contours(self, contour:np.ndarray, color:tuple=(0, 255, 0), thickness:int=2):
        ''' 
        Dibuja los contornos de los cubos sobre la imagen original.
            @param contours (list) - Lista de contornos detectados.
            @param frame (numpy array) - Imagen sobre la cual se dibujarán los contornos.
            @return contour_img (numpy array) - Imagen con los contornos dibujados.
        '''
        boundRect = cv2.boundingRect(contour)
        cv2.rectangle(self.contour_img, 
                        (int(boundRect[0]), int(boundRect[1])), 
                        (int(boundRect[0] + boundRect[2]), int(boundRect[1] + boundRect[3])), 
                        color, thickness)
        

    def process_image(self, frame:np.ndarray, mostrar:bool=False, debug:bool=False)-> tuple:
        ''' 
        Procesa la imagen, encuentra los contornos y cubos, y los mapea a la matriz.
            @param frame (numpy array) - Imagen que se va a procesar.
            @return matrix (numpy array) - Matriz con la información de los cubos detectados.
        '''
        self.frame = deepcopy(frame)
        self.contour_img = deepcopy(self.frame)
        self.debug = debug

        # Preprocesamiento de la imagen
        morph_clean = self._preprocess_image()
        
        # Encontrar contornos externos
        contours, _ = cv2.findContours(morph_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filtrar los contornos por tamaño
        large_contours = self._filter_contours(contours)

        if self.debug:
            img = deepcopy(self.frame)
            img_filtered = deepcopy(self.frame)
            cv2.drawContours(img, contourIdx=-1, contours=contours, color=(0, 255, 0))
            cv2.drawContours(img_filtered, contourIdx=-1, contours=large_contours, color=(0, 255, 0))

            cv2.imshow('All Contours', img)
            cv2.imshow('Filtered Contours', img_filtered)


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
            color = self._get_color(center=center)
            colors.append(color)
            if color == 4:
                raise TypeError('Color not Found')
            areas.append(cv2.contourArea(contour))

            self._draw_contours(contour=contour, color=colores[color])

            # Visualizar el contorno y el color en la imagen
            cv2.circle(self.contour_img, center, 5, (0, 0, 0), -1)
            # cv2.putText(self.contour_img, str(color), (center[0], center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        
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
    use_cam = False
    if use_cam:
        cam = cv2.VideoCapture(4)
        if cam.isOpened():
            _, frame = cam.read()
    else:
        for i in range(8, 25, 1):
            num = i
            ruta = f'/home/laboratorio/ros_workspace/src/proyecto_final/data/example_img/Figuras_Lateral/Figura_{num}_L.png'
            frame = cv2.imread(ruta)

            processor = ImageProcessor_Front()
            # frame = cv2.imread(ruta)
            processor.process_image(frame, mostrar = True, debug=True)
            # print(np.array2string(matriz, separator=', ', formatter={'all': lambda x: f'{int(x)}'}))
            # print(np.array(matriz))
