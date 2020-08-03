; Auto-generated. Do not edit!


(cl:in-package reflex_msgs2-msg)


;//! \htmlinclude RadianServoCommands.msg.html

(cl:defclass <RadianServoCommands> (roslisp-msg-protocol:ros-message)
  ((radian_commands
    :reader radian_commands
    :initarg :radian_commands
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass RadianServoCommands (<RadianServoCommands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RadianServoCommands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RadianServoCommands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reflex_msgs2-msg:<RadianServoCommands> is deprecated: use reflex_msgs2-msg:RadianServoCommands instead.")))

(cl:ensure-generic-function 'radian_commands-val :lambda-list '(m))
(cl:defmethod radian_commands-val ((m <RadianServoCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:radian_commands-val is deprecated.  Use reflex_msgs2-msg:radian_commands instead.")
  (radian_commands m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RadianServoCommands>) ostream)
  "Serializes a message object of type '<RadianServoCommands>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'radian_commands))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RadianServoCommands>) istream)
  "Deserializes a message object of type '<RadianServoCommands>"
  (cl:setf (cl:slot-value msg 'radian_commands) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'radian_commands)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RadianServoCommands>)))
  "Returns string type for a message object of type '<RadianServoCommands>"
  "reflex_msgs2/RadianServoCommands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RadianServoCommands)))
  "Returns string type for a message object of type 'RadianServoCommands"
  "reflex_msgs2/RadianServoCommands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RadianServoCommands>)))
  "Returns md5sum for a message object of type '<RadianServoCommands>"
  "ba1e88d9da1745cdc1900895d8c434b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RadianServoCommands)))
  "Returns md5sum for a message object of type 'RadianServoCommands"
  "ba1e88d9da1745cdc1900895d8c434b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RadianServoCommands>)))
  "Returns full string definition for message of type '<RadianServoCommands>"
  (cl:format cl:nil "# Sets either radian position or radian/s velocity, depending on control mode~%float32[4] radian_commands~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RadianServoCommands)))
  "Returns full string definition for message of type 'RadianServoCommands"
  (cl:format cl:nil "# Sets either radian position or radian/s velocity, depending on control mode~%float32[4] radian_commands~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RadianServoCommands>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'radian_commands) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RadianServoCommands>))
  "Converts a ROS message object to a list"
  (cl:list 'RadianServoCommands
    (cl:cons ':radian_commands (radian_commands msg))
))
