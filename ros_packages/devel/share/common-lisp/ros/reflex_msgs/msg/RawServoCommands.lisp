; Auto-generated. Do not edit!


(cl:in-package reflex_msgs-msg)


;//! \htmlinclude RawServoCommands.msg.html

(cl:defclass <RawServoCommands> (roslisp-msg-protocol:ros-message)
  ((raw_positions
    :reader raw_positions
    :initarg :raw_positions
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass RawServoCommands (<RawServoCommands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RawServoCommands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RawServoCommands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reflex_msgs-msg:<RawServoCommands> is deprecated: use reflex_msgs-msg:RawServoCommands instead.")))

(cl:ensure-generic-function 'raw_positions-val :lambda-list '(m))
(cl:defmethod raw_positions-val ((m <RawServoCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs-msg:raw_positions-val is deprecated.  Use reflex_msgs-msg:raw_positions instead.")
  (raw_positions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RawServoCommands>) ostream)
  "Serializes a message object of type '<RawServoCommands>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'raw_positions))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RawServoCommands>) istream)
  "Deserializes a message object of type '<RawServoCommands>"
  (cl:setf (cl:slot-value msg 'raw_positions) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'raw_positions)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RawServoCommands>)))
  "Returns string type for a message object of type '<RawServoCommands>"
  "reflex_msgs/RawServoCommands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RawServoCommands)))
  "Returns string type for a message object of type 'RawServoCommands"
  "reflex_msgs/RawServoCommands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RawServoCommands>)))
  "Returns md5sum for a message object of type '<RawServoCommands>"
  "aea318ab5663e05395d3ab895fd67e1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RawServoCommands)))
  "Returns md5sum for a message object of type 'RawServoCommands"
  "aea318ab5663e05395d3ab895fd67e1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RawServoCommands>)))
  "Returns full string definition for message of type '<RawServoCommands>"
  (cl:format cl:nil "uint16[4] raw_positions~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RawServoCommands)))
  "Returns full string definition for message of type 'RawServoCommands"
  (cl:format cl:nil "uint16[4] raw_positions~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RawServoCommands>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'raw_positions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RawServoCommands>))
  "Converts a ROS message object to a list"
  (cl:list 'RawServoCommands
    (cl:cons ':raw_positions (raw_positions msg))
))
