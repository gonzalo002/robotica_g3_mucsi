; Auto-generated. Do not edit!


(cl:in-package proyecto_final-msg)


;//! \htmlinclude CubosGoal.msg.html

(cl:defclass <CubosGoal> (roslisp-msg-protocol:ros-message)
  ((order
    :reader order
    :initarg :order
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CubosGoal (<CubosGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CubosGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CubosGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name proyecto_final-msg:<CubosGoal> is deprecated: use proyecto_final-msg:CubosGoal instead.")))

(cl:ensure-generic-function 'order-val :lambda-list '(m))
(cl:defmethod order-val ((m <CubosGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proyecto_final-msg:order-val is deprecated.  Use proyecto_final-msg:order instead.")
  (order m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CubosGoal>) ostream)
  "Serializes a message object of type '<CubosGoal>"
  (cl:let* ((signed (cl:slot-value msg 'order)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CubosGoal>) istream)
  "Deserializes a message object of type '<CubosGoal>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'order) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CubosGoal>)))
  "Returns string type for a message object of type '<CubosGoal>"
  "proyecto_final/CubosGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CubosGoal)))
  "Returns string type for a message object of type 'CubosGoal"
  "proyecto_final/CubosGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CubosGoal>)))
  "Returns md5sum for a message object of type '<CubosGoal>"
  "0bb344a14dad212e50d218aec04eba29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CubosGoal)))
  "Returns md5sum for a message object of type 'CubosGoal"
  "0bb344a14dad212e50d218aec04eba29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CubosGoal>)))
  "Returns full string definition for message of type '<CubosGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%int8 order~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CubosGoal)))
  "Returns full string definition for message of type 'CubosGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%int8 order~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CubosGoal>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CubosGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'CubosGoal
    (cl:cons ':order (order msg))
))