; Auto-generated. Do not edit!


(cl:in-package proyecto_final-msg)


;//! \htmlinclude FigurasResult.msg.html

(cl:defclass <FigurasResult> (roslisp-msg-protocol:ros-message)
  ((up
    :reader up
    :initarg :up
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (front
    :reader front
    :initarg :front
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass FigurasResult (<FigurasResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FigurasResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FigurasResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name proyecto_final-msg:<FigurasResult> is deprecated: use proyecto_final-msg:FigurasResult instead.")))

(cl:ensure-generic-function 'up-val :lambda-list '(m))
(cl:defmethod up-val ((m <FigurasResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proyecto_final-msg:up-val is deprecated.  Use proyecto_final-msg:up instead.")
  (up m))

(cl:ensure-generic-function 'front-val :lambda-list '(m))
(cl:defmethod front-val ((m <FigurasResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proyecto_final-msg:front-val is deprecated.  Use proyecto_final-msg:front instead.")
  (front m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FigurasResult>) ostream)
  "Serializes a message object of type '<FigurasResult>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'up))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'up))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'front))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'front))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FigurasResult>) istream)
  "Deserializes a message object of type '<FigurasResult>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'up) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'up)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'front) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'front)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FigurasResult>)))
  "Returns string type for a message object of type '<FigurasResult>"
  "proyecto_final/FigurasResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FigurasResult)))
  "Returns string type for a message object of type 'FigurasResult"
  "proyecto_final/FigurasResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FigurasResult>)))
  "Returns md5sum for a message object of type '<FigurasResult>"
  "d57cf924ec719c04c0d0db866abe5ad7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FigurasResult)))
  "Returns md5sum for a message object of type 'FigurasResult"
  "d57cf924ec719c04c0d0db866abe5ad7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FigurasResult>)))
  "Returns full string definition for message of type '<FigurasResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%int8[] up~%int8[] front~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FigurasResult)))
  "Returns full string definition for message of type 'FigurasResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%int8[] up~%int8[] front~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FigurasResult>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'up) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'front) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FigurasResult>))
  "Converts a ROS message object to a list"
  (cl:list 'FigurasResult
    (cl:cons ':up (up msg))
    (cl:cons ':front (front msg))
))
