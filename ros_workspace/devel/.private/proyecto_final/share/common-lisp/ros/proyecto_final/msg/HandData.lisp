; Auto-generated. Do not edit!


(cl:in-package proyecto_final-msg)


;//! \htmlinclude HandData.msg.html

(cl:defclass <HandData> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (is_open
    :reader is_open
    :initarg :is_open
    :type cl:boolean
    :initform cl:nil)
   (is_peace
    :reader is_peace
    :initarg :is_peace
    :type cl:boolean
    :initform cl:nil)
   (hand_detected
    :reader hand_detected
    :initarg :hand_detected
    :type cl:boolean
    :initform cl:nil)
   (is_dino
    :reader is_dino
    :initarg :is_dino
    :type cl:boolean
    :initform cl:nil)
   (is_dislike
    :reader is_dislike
    :initarg :is_dislike
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass HandData (<HandData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name proyecto_final-msg:<HandData> is deprecated: use proyecto_final-msg:HandData instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <HandData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proyecto_final-msg:x-val is deprecated.  Use proyecto_final-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <HandData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proyecto_final-msg:y-val is deprecated.  Use proyecto_final-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <HandData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proyecto_final-msg:z-val is deprecated.  Use proyecto_final-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'is_open-val :lambda-list '(m))
(cl:defmethod is_open-val ((m <HandData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proyecto_final-msg:is_open-val is deprecated.  Use proyecto_final-msg:is_open instead.")
  (is_open m))

(cl:ensure-generic-function 'is_peace-val :lambda-list '(m))
(cl:defmethod is_peace-val ((m <HandData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proyecto_final-msg:is_peace-val is deprecated.  Use proyecto_final-msg:is_peace instead.")
  (is_peace m))

(cl:ensure-generic-function 'hand_detected-val :lambda-list '(m))
(cl:defmethod hand_detected-val ((m <HandData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proyecto_final-msg:hand_detected-val is deprecated.  Use proyecto_final-msg:hand_detected instead.")
  (hand_detected m))

(cl:ensure-generic-function 'is_dino-val :lambda-list '(m))
(cl:defmethod is_dino-val ((m <HandData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proyecto_final-msg:is_dino-val is deprecated.  Use proyecto_final-msg:is_dino instead.")
  (is_dino m))

(cl:ensure-generic-function 'is_dislike-val :lambda-list '(m))
(cl:defmethod is_dislike-val ((m <HandData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proyecto_final-msg:is_dislike-val is deprecated.  Use proyecto_final-msg:is_dislike instead.")
  (is_dislike m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandData>) ostream)
  "Serializes a message object of type '<HandData>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_open) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_peace) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'hand_detected) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_dino) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_dislike) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandData>) istream)
  "Deserializes a message object of type '<HandData>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'is_open) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_peace) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'hand_detected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_dino) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_dislike) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandData>)))
  "Returns string type for a message object of type '<HandData>"
  "proyecto_final/HandData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandData)))
  "Returns string type for a message object of type 'HandData"
  "proyecto_final/HandData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandData>)))
  "Returns md5sum for a message object of type '<HandData>"
  "9b0493b2ed6710620a749baf2ddc5457")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandData)))
  "Returns md5sum for a message object of type 'HandData"
  "9b0493b2ed6710620a749baf2ddc5457")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandData>)))
  "Returns full string definition for message of type '<HandData>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%bool is_open~%bool is_peace~%bool hand_detected~%bool is_dino~%bool is_dislike~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandData)))
  "Returns full string definition for message of type 'HandData"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%bool is_open~%bool is_peace~%bool hand_detected~%bool is_dino~%bool is_dislike~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandData>))
  (cl:+ 0
     4
     4
     4
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandData>))
  "Converts a ROS message object to a list"
  (cl:list 'HandData
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':is_open (is_open msg))
    (cl:cons ':is_peace (is_peace msg))
    (cl:cons ':hand_detected (hand_detected msg))
    (cl:cons ':is_dino (is_dino msg))
    (cl:cons ':is_dislike (is_dislike msg))
))
