import cv2
import os

# Función principal que captura el video y aplica los procesamientos
def main():
    cam = cv2.VideoCapture(0)
    
    if not cam.isOpened():
        print("Error: No se pudo abrir el vídeo.")
        return
    
    if os.listdir('src/proyecto_final/scripts/vision/Imagenes') == []:
        i = 0
    else:
        i = int(os.listdir('src/proyecto_final/scripts/vision/Imagenes')[-1][-5])
        
    
    while True:
        success, frame = cam.read()
        
        cv2.imshow('MyWindow', frame)
        
        if cv2.waitKey(1) == ord('s'):
            cv2.imwrite(f'src/proyecto_final/scripts/vision/Imagenes/Imagen_{i}.png', frame)
            i += 1 
            
        if cv2.waitKey(1) == ord('q') or not success:
            break

    cam.release()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()