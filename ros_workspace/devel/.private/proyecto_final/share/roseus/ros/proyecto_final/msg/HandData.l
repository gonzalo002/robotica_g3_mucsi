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
  :slots (_x _y _is_open ))

(defmethod proyecto_final::HandData
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:is_open __is_open) nil)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _is_open __is_open)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:is_open
   (&optional (__is_open :null))
   (if (not (eq __is_open :null)) (setq _is_open __is_open)) _is_open)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; bool _is_open
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
     ;; bool _is_open
       (if _is_open (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _is_open
     (setq _is_open (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get proyecto_final::HandData :md5sum-) "84735364a2a5c5b5d5c40ee10fa1ddee")
(setf (get proyecto_final::HandData :datatype-) "proyecto_final/HandData")
(setf (get proyecto_final::HandData :definition-)
      "float32 x
float32 y
bool is_open
")



(provide :proyecto_final/HandData "84735364a2a5c5b5d5c40ee10fa1ddee")


