; Auto-generated. Do not edit!


(cl:in-package reflex_one_msgs-msg)


;//! \htmlinclude Command.msg.html

(cl:defclass <Command> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type reflex_one_msgs-msg:PoseCommand
    :initform (cl:make-instance 'reflex_one_msgs-msg:PoseCommand))
   (velocity
    :reader velocity
    :initarg :velocity
    :type reflex_one_msgs-msg:VelocityCommand
    :initform (cl:make-instance 'reflex_one_msgs-msg:VelocityCommand)))
)

(cl:defclass Command (<Command>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Command>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Command)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reflex_one_msgs-msg:<Command> is deprecated: use reflex_one_msgs-msg:Command instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_one_msgs-msg:pose-val is deprecated.  Use reflex_one_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_one_msgs-msg:velocity-val is deprecated.  Use reflex_one_msgs-msg:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Command>) ostream)
  "Serializes a message object of type '<Command>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Command>) istream)
  "Deserializes a message object of type '<Command>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Command>)))
  "Returns string type for a message object of type '<Command>"
  "reflex_one_msgs/Command")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Command)))
  "Returns string type for a message object of type 'Command"
  "reflex_one_msgs/Command")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Command>)))
  "Returns md5sum for a message object of type '<Command>"
  "cdbebc0876ac872616d6c63603a84a2e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Command)))
  "Returns md5sum for a message object of type 'Command"
  "cdbebc0876ac872616d6c63603a84a2e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Command>)))
  "Returns full string definition for message of type '<Command>"
  (cl:format cl:nil "PoseCommand pose~%VelocityCommand velocity~%~%================================================================================~%MSG: reflex_one_msgs/PoseCommand~%# Position in radians of various motors~%float64 f1~%float64 f2~%float64 f3~%float64 preshape1~%float64 preshape2~%~%================================================================================~%MSG: reflex_one_msgs/VelocityCommand~%# Velocity in radians/second of various motors~%float64 f1~%float64 f2~%float64 f3~%float64 preshape1~%float64 preshape2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Command)))
  "Returns full string definition for message of type 'Command"
  (cl:format cl:nil "PoseCommand pose~%VelocityCommand velocity~%~%================================================================================~%MSG: reflex_one_msgs/PoseCommand~%# Position in radians of various motors~%float64 f1~%float64 f2~%float64 f3~%float64 preshape1~%float64 preshape2~%~%================================================================================~%MSG: reflex_one_msgs/VelocityCommand~%# Velocity in radians/second of various motors~%float64 f1~%float64 f2~%float64 f3~%float64 preshape1~%float64 preshape2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Command>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Command>))
  "Converts a ROS message object to a list"
  (cl:list 'Command
    (cl:cons ':pose (pose msg))
    (cl:cons ':velocity (velocity msg))
))
