import cv2
import os

# Función principal que captura el video y aplica los procesamientos
def main():
    cam_1 = cv2.VideoCapture(0)
    cam_2 = cv2.VideoCapture(4)
    
    if not cam_1.isOpened():
        print("Error: No se pudo abrir el vídeo.")
        return
    
    if os.listdir('src/proyecto_final/scripts/vision/grupo_2/Imagenes') == []:
        i = 0
    else:
        i = int(os.listdir('src/proyecto_final/scripts/vision/grupo_2/Imagenes')[-1][-5])
        
    
    while True:
        success_1, frame_1 = cam_1.read()
        success_2, frame_2 = cam_2.read()
        
        cv2.imshow('cam_1', frame_1)
        cv2.imshow('cam_2', frame_2)
        
        if cv2.waitKey(1) == ord('s'):
            cv2.imwrite(f'src/proyecto_final/scripts/vision/grupo_2/Imagenes/cam_1_Imagen_{i}.png', frame_1)
            cv2.imwrite(f'src/proyecto_final/scripts/vision/grupo_2/Imagenes/cam_2_Imagen_{i}.png', frame_2)
            i += 1 
            
        if cv2.waitKey(1) == ord('q') or not success_1 or not success_2:
            break

    cam_1.release()
    cam_2.release()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()