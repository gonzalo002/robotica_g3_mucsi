#!/usr/bin/env python3

import cv2, os, rospy
from copy import deepcopy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from proyecto_final.msg import HandData
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "1"
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "2"

from proyecto_final.vision.hand_detector import HandDetector

    
class HandTopic():
    def __init__(self):
        rospy.init_node('hand_detector_node')

        self.cv_img_top = self.cv_img_lateral = []
        
        self.bridge = CvBridge()
        self.subs_cam_top = rospy.Subscriber('/top_hand_cam/image_raw', Image, self.cb_image_top)
        self.subs_cam_lateral = rospy.Subscriber('/lateral_hand_cam/image_raw', Image, self.cb_image_lateral)
        self.pub_hand_data = rospy.Publisher('/hand_data', HandData, queue_size=10)

        self.hand_processor = HandDetector()

        self.hand_processor.define_hands()
        self.hand_detector()

    def cb_image_top(self, image:Image)->None:
        ''' 
        Callback del subscriptor de la cámara.
            @param image (Image) - Imagen de la camara
        '''

        self.cv_img_top = deepcopy(self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough'))
        self.cv_img_top = deepcopy(cv2.cvtColor(self.cv_img_top, cv2.COLOR_RGB2BGR))

    def cb_image_lateral(self, image:Image)->None:
        ''' 
        Callback del subscriptor de la cámara.
            @param image (Image) - Imagen de la camara
        '''

        self.cv_img_lateral = deepcopy(self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough'))
        self.cv_img_lateral = deepcopy(cv2.cvtColor(self.cv_img_lateral, cv2.COLOR_RGB2BGR))


    def hand_detector(self) -> None:
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if len(self.cv_img_lateral)!=0 and len(self.cv_img_top)!=0:
                img_top = deepcopy(self.cv_img_top)
                img_lateral = deepcopy(self.cv_img_lateral)

                self.hand_processor.detect_hand(img_top=img_top, img_lateral=img_lateral)

                self.pub_hand_data.publish(self.hand_processor.hand_data)
                self.cv_img_lateral = self.cv_img_top = deepcopy([])
            
            rate.sleep()

if __name__ == '__main__':
    hand = HandTopic()
