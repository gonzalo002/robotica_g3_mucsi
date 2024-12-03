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
   (is_open
    :reader is_open
    :initarg :is_open
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

(cl:ensure-generic-function 'is_open-val :lambda-list '(m))
(cl:defmethod is_open-val ((m <HandData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proyecto_final-msg:is_open-val is deprecated.  Use proyecto_final-msg:is_open instead.")
  (is_open m))
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_open) 1 0)) ostream)
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
    (cl:setf (cl:slot-value msg 'is_open) (cl:not (cl:zerop (cl:read-byte istream))))
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
  "84735364a2a5c5b5d5c40ee10fa1ddee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandData)))
  "Returns md5sum for a message object of type 'HandData"
  "84735364a2a5c5b5d5c40ee10fa1ddee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandData>)))
  "Returns full string definition for message of type '<HandData>"
  (cl:format cl:nil "float32 x~%float32 y~%bool is_open~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandData)))
  "Returns full string definition for message of type 'HandData"
  (cl:format cl:nil "float32 x~%float32 y~%bool is_open~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandData>))
  (cl:+ 0
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandData>))
  "Converts a ROS message object to a list"
  (cl:list 'HandData
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':is_open (is_open msg))
))
