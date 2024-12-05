import cv2
import numpy as np
from copy import deepcopy
import yaml

class CubeTracker:
    ''' 
    Clase que procesa imágenes para detectar cubos y asignarles un color en función de su forma y tonalidad en el espacio de color HSV.
    
    Esta clase implementa varios métodos para:
        - Convertir imágenes a escala de grises o al espacio de color HSV.
        - Aplicar umbrales de Otsu para binarización.
        - Detectar bordes utilizando el filtro de Canny.
        - Encontrar contornos en las imágenes procesadas.
        - Identificar cubos en las imágenes y asignarles un color basado en su tonalidad.
        - Mostrar la imagen procesada y cerrar las ventanas de visualización.
    
    Métodos:
        - __init__() - Inicializa el objeto y crea un espacio para almacenar la imagen.
        - _aplicar_filtro_grises() - Convierte la imagen a escala de grises.
        - _aplicar_filtro_hsv() - Convierte la imagen a espacio de color HSV.
        - _procesar_umbral_otsu() - Aplica un umbral de Otsu para binarizar la imagen.
        - _detectar_bordes() - Detecta los bordes en una imagen utilizando Canny.
        - _encontrar_contornos() - Encuentra los contornos de una imagen binarizada.
        - _identificar_cubo_y_color() - Identifica cubos y les asigna un color basado en el espacio HSV.
        - analizar_imagen() - Procesa la imagen, detecta cubos y muestra los resultados si es necesario.
    
    Atributos:
        - frame (numpy array) - Almacena la imagen de entrada que se va a procesar.
    '''
    def __init__(self, path:str, aruco_dict=cv2.aruco.DICT_4X4_50, marker_length=0.05) -> None:
        '''
        Inicializa la clase y configura los parámetros de Aruco.
            @param aruco_dict (int) - Diccionario de Aruco (por defecto, 4x4 con 50 marcadores).
            @param marker_length (float) - Longitud del lado del marcador en metros.
            @param camera_matrix (numpy array) - Matriz intrínseca de la cámara.
            @param dist_coeffs (numpy array) - Coeficientes de distorsión de la cámara.
        '''
        self.frame = None
        self.frame = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self._get_camara_calibration(path)

        self.marker_length = marker_length
        self.marker_position = None

    def _get_camara_calibration(self, path:str)->None:
        with open(path, '+r') as f:
            fichero =  yaml.load(f, yaml.Loader)

        self.camera_matrix = np.array(fichero["camera_matrix"]["data"]).reshape(fichero["camera_matrix"]["rows"], fichero["camera_matrix"]["cols"])
        self.dist_coeffs = np.array(fichero["distortion_coefficients"]["data"])
        

    def _aplicar_filtro_grises(self, frame:np.ndarray) -> np.ndarray:
        ''' 
        Convierte una imagen de color a escala de grises.
            @param frame (numpy array) - Imagen en formato BGR.
            @return gray_frame (numpy array) - Imagen en escala de grises.
        '''
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    def _aplicar_filtro_hsv(self, frame:np.ndarray) -> np.ndarray:
        ''' 
        Convierte una imagen de color a espacio de color HSV.
            @param frame (numpy array) - Imagen en formato BGR.
            @return hsv_frame (numpy array) - Imagen en espacio de color HSV.
        '''
        return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    def _procesar_umbral_otsu(self, gray:np.ndarray) -> np.ndarray:
        ''' 
        Aplica un umbral de Otsu para binarizar la imagen.
            @param gray (numpy array) - Imagen en escala de grises.
            @return otsu_thresh (numpy array) - Imagen binarizada tras el umbral de Otsu.
        '''
        _, otsu_thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        cv2.imwrite("otsu.png", otsu_thresh)
        kernel = np.ones((3, 3), np.uint8)
        return cv2.morphologyEx(otsu_thresh, cv2.MORPH_CLOSE, kernel, iterations=1)
    
    def _detectar_bordes(self, imagen:np.ndarray) -> np.ndarray:
        ''' 
        Detecta los bordes de una imagen utilizando el filtro de Canny.
            @param imagen (numpy array) - Imagen de entrada (en escala de grises o color).
            @return edges (numpy array) - Imagen con bordes detectados.
        '''
        desenfocada = cv2.GaussianBlur(imagen, (5, 5), 0)
        return cv2.Canny(desenfocada, 0, 255)
    
    def _encontrar_contornos(self, edges:np.ndarray) -> list:
        ''' 
        Encuentra los contornos en una imagen binarizada.
            @param edges (numpy array) - Imagen binarizada con bordes detectados.
            @return contours (list) - Lista de contornos encontrados.
        '''
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours
    
    def _identificar_cubo_y_color(self, contours:list, frame:np.ndarray, hsv_frame:np.ndarray, gray:np.ndarray, area_size:int=1000) -> np.ndarray:
        ''' 
        Identifica cubos en la imagen y les asigna un color basado en el espacio HSV.
            @param contours (list) - Lista de contornos encontrados en la imagen.
            @param frame (numpy array) - Imagen original en formato BGR.
            @param hsv_frame (numpy array) - Imagen convertida a espacio de color HSV.
            @param gray (numpy array) - Imagen en escala de grises.
            @param area_size (int) - Umbral mínimo de área para considerar un contorno como válido. Por defecto, 2000.
            @return resultado (numpy array) - Imagen con los cubos identificados y etiquetados por color.
        '''
        color_ranges = {
            "Red": [(0, 100, 100), (15, 255, 255)],
            "Green": [(40, 50, 50), (80, 255, 255)],
            "Blue": [(100, 50, 50), (130, 255, 255)],
            "Yellow": [(20, 100, 100), (30, 255, 255)]
        }
        color_map = {"Red": 0, "Green": 1, "Blue": 2, "Yellow": 3, "Unknown":-1}
        resultado = frame.copy()
        area_size = [cv2.contourArea(cnt) for cnt in contours]
        mode_cubes = np.median(area_size)

        dicionario_resultado = []
        for i,cnt in enumerate(contours):
            if (area_size[i] > mode_cubes-300) and (area_size[i] < mode_cubes+300):
                approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
                
                if len(approx) == 4:
                    rect = cv2.minAreaRect(cnt)
                    box = cv2.boxPoints(rect)
                    box = np.intp(box)
                    cv2.drawContours(resultado, [box], 0, (0, 255, 0), 2)

                    center = (int(rect[0][0]), int(rect[0][1]))
                    angle = rect[2]

                    cv2.circle(resultado, center, 3, (255, 0, 0), -1)
                    
                    cv2.putText(resultado, f"Position: {center}", 
                            (center[0] + 20, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
                    cv2.putText(resultado, f"Angle: {angle:.2f}", 
                            (center[0] + 20, center[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

                    mask = np.zeros_like(gray)
                    cv2.drawContours(mask, [cnt], -1, 255, -1)
                    mean_val = cv2.mean(hsv_frame, mask=mask)[:3]

                    color = "Unknown"
                    for key, (lower, upper) in color_ranges.items():
                        if all(lower[i] <= mean_val[i] <= upper[i] for i in range(3)):
                            color = key
                            break

                    # Creamos diccionario con las características de la pieza
                    dicionario_resultado.append({"Posicion": center, "Angulo": angle, "Color": color_map[color]})
                    
                    cv2.putText(resultado, color, (center[0] + 20, center[1] + 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
                    
        self.imagen_analizada = resultado
        
        return dicionario_resultado
    
    def _detectar_aruco(self, frame: np.ndarray) -> None:
        '''
        Detecta un marcador Aruco en la imagen y calcula su posición y orientación.
            @param frame (numpy array) - Imagen en formato escala de grises.
        '''
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Estimar pose del marcador
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            # Guardar la posición del primer marcador detectado
            self.marker_position = tvecs[0]
            frame = cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[0], tvecs[0], 0.1)


    def analizar_imagen(self, frame:np.ndarray, area_size:int=1000, mostrar:bool = False) -> list:
        ''' 
        Procesa una imagen para detectar cubos y sus colores, y muestra los resultados.
            @param frame (numpy array) - Imagen de entrada en formato BGR.
            @param area_size (int) - Umbral mínimo de área para considerar un contorno como válido. Por defecto, 2000.
            @param mostrar (bool) - Si es True, muestra la imagen procesada. Por defecto, False.
            @return resultado (list) - Imagen procesada con cubos identificados y etiquetados.
        '''
        # Realizar copia del frame
        self.frame = deepcopy(frame)

        # Detectar Aruco
        self._detectar_aruco(frame)
            
        gray = self._aplicar_filtro_grises(frame)
        hsv_frame = self._aplicar_filtro_hsv(frame)
        morph_clean = self._procesar_umbral_otsu(gray)
        edges = self._detectar_bordes(morph_clean)
        contours = self._encontrar_contornos(edges)
        
        resultado = self._identificar_cubo_y_color(contours, frame, hsv_frame, gray, area_size)

        if mostrar:
            cv2.imshow('Contoured Image', self.imagen_analizada)

            if cv2.waitKey(0) & 0xFF == ord('q'):
                cv2.destroyAllWindows()

        return resultado

if __name__ == "__main__":
    cam = cv2.VideoCapture(0)
    camara = CubeTracker(path="src/proyecto_final/data/camera_data/ost.yaml")
    if cam.isOpened():
        _, frame = cam.read()
    # frame = cv2.imread('ProyectoFinal/Cubos_Exparcidos/Aruco_1.png')
        resultado = camara.analizar_imagen(frame,area_size=800,mostrar=True)
        cam.release()
    else:
        print("ERROR")