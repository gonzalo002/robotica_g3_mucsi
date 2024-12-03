; Auto-generated. Do not edit!


(cl:in-package proyecto_final-msg)


;//! \htmlinclude IdCubos.msg.html

(cl:defclass <IdCubos> (roslisp-msg-protocol:ros-message)
  ((color
    :reader color
    :initarg :color
    :type cl:fixnum
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass IdCubos (<IdCubos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IdCubos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IdCubos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name proyecto_final-msg:<IdCubos> is deprecated: use proyecto_final-msg:IdCubos instead.")))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <IdCubos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proyecto_final-msg:color-val is deprecated.  Use proyecto_final-msg:color instead.")
  (color m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <IdCubos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proyecto_final-msg:pose-val is deprecated.  Use proyecto_final-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IdCubos>) ostream)
  "Serializes a message object of type '<IdCubos>"
  (cl:let* ((signed (cl:slot-value msg 'color)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IdCubos>) istream)
  "Deserializes a message object of type '<IdCubos>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'color) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IdCubos>)))
  "Returns string type for a message object of type '<IdCubos>"
  "proyecto_final/IdCubos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IdCubos)))
  "Returns string type for a message object of type 'IdCubos"
  "proyecto_final/IdCubos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IdCubos>)))
  "Returns md5sum for a message object of type '<IdCubos>"
  "7d40436a98b00a5347eed8cfb3ef00bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IdCubos)))
  "Returns md5sum for a message object of type 'IdCubos"
  "7d40436a98b00a5347eed8cfb3ef00bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IdCubos>)))
  "Returns full string definition for message of type '<IdCubos>"
  (cl:format cl:nil "int8 color~%geometry_msgs/Pose pose~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IdCubos)))
  "Returns full string definition for message of type 'IdCubos"
  (cl:format cl:nil "int8 color~%geometry_msgs/Pose pose~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IdCubos>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IdCubos>))
  "Converts a ROS message object to a list"
  (cl:list 'IdCubos
    (cl:cons ':color (color msg))
    (cl:cons ':pose (pose msg))
))
