;; Auto-generated. Do not edit!


(when (boundp 'proyecto_final::HandData)
  (if (not (find-package "PROYECTO_FINAL"))
    (make-package "PROYECTO_FINAL"))
  (shadow 'HandData (find-package "PROYECTO_FINAL")))
(unless (find-package "PROYECTO_FINAL::HANDDATA")
  (make-package "PROYECTO_FINAL::HANDDATA"))

(in-package "ROS")
;;//! \htmlinclude HandData.msg.html


(defclass proyecto_final::HandData
  :super ros::object
  :slots (_x _y _z _is_open _is_peace _hand_detected _is_dino _is_dislike ))

(defmethod proyecto_final::HandData
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:is_open __is_open) nil)
    ((:is_peace __is_peace) nil)
    ((:hand_detected __hand_detected) nil)
    ((:is_dino __is_dino) nil)
    ((:is_dislike __is_dislike) nil)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _is_open __is_open)
   (setq _is_peace __is_peace)
   (setq _hand_detected __hand_detected)
   (setq _is_dino __is_dino)
   (setq _is_dislike __is_dislike)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:is_open
   (&optional (__is_open :null))
   (if (not (eq __is_open :null)) (setq _is_open __is_open)) _is_open)
  (:is_peace
   (&optional (__is_peace :null))
   (if (not (eq __is_peace :null)) (setq _is_peace __is_peace)) _is_peace)
  (:hand_detected
   (&optional (__hand_detected :null))
   (if (not (eq __hand_detected :null)) (setq _hand_detected __hand_detected)) _hand_detected)
  (:is_dino
   (&optional (__is_dino :null))
   (if (not (eq __is_dino :null)) (setq _is_dino __is_dino)) _is_dino)
  (:is_dislike
   (&optional (__is_dislike :null))
   (if (not (eq __is_dislike :null)) (setq _is_dislike __is_dislike)) _is_dislike)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ;; bool _is_open
    1
    ;; bool _is_peace
    1
    ;; bool _hand_detected
    1
    ;; bool _is_dino
    1
    ;; bool _is_dislike
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _is_open
       (if _is_open (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_peace
       (if _is_peace (write-byte -1 s) (write-byte 0 s))
     ;; bool _hand_detected
       (if _hand_detected (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_dino
       (if _is_dino (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_dislike
       (if _is_dislike (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _is_open
     (setq _is_open (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_peace
     (setq _is_peace (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _hand_detected
     (setq _hand_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_dino
     (setq _is_dino (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_dislike
     (setq _is_dislike (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get proyecto_final::HandData :md5sum-) "9b0493b2ed6710620a749baf2ddc5457")
(setf (get proyecto_final::HandData :datatype-) "proyecto_final/HandData")
(setf (get proyecto_final::HandData :definition-)
      "float32 x
float32 y
float32 z
bool is_open
bool is_peace
bool hand_detected
bool is_dino
bool is_dislike

")



(provide :proyecto_final/HandData "9b0493b2ed6710620a749baf2ddc5457")


