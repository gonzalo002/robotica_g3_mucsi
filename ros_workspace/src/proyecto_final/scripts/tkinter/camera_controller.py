import cv2

class CameraController:
    def __init__(self):
        self.cap_1 = None
        self.cap_2 = None
        self.cap_3 = None

    def start(self):
        self.cap_1 = cv2.VideoCapture(0)
        self.cap_2 = cv2.VideoCapture(1)
        self.cap_3 = cv2.VideoCapture(1)

    def stop(self):
        if self.cap_1:
            self.cap_1.release()

        if self.cap_2:
            self.cap_2.release()
        
        if self.cap_3:
            self.cap_2.release()

    def get_frame(self, camera_index):
        if camera_index == 1:
            return self.cap_1.read()
        elif camera_index == 2:
            return self.cap_2.read()
        else:
            return self.cap_3.read()