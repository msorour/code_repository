; Auto-generated. Do not edit!


(cl:in-package ros_gazebo_plugin_comm-msg)


;//! \htmlinclude Vector7d.msg.html

(cl:defclass <Vector7d> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 7 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Vector7d (<Vector7d>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Vector7d>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Vector7d)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_gazebo_plugin_comm-msg:<Vector7d> is deprecated: use ros_gazebo_plugin_comm-msg:Vector7d instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Vector7d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_gazebo_plugin_comm-msg:data-val is deprecated.  Use ros_gazebo_plugin_comm-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Vector7d>) ostream)
  "Serializes a message object of type '<Vector7d>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Vector7d>) istream)
  "Deserializes a message object of type '<Vector7d>"
  (cl:setf (cl:slot-value msg 'data) (cl:make-array 7))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i 7)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Vector7d>)))
  "Returns string type for a message object of type '<Vector7d>"
  "ros_gazebo_plugin_comm/Vector7d")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Vector7d)))
  "Returns string type for a message object of type 'Vector7d"
  "ros_gazebo_plugin_comm/Vector7d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Vector7d>)))
  "Returns md5sum for a message object of type '<Vector7d>"
  "e4844fe6885794a5295df1d81cbae79f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Vector7d)))
  "Returns md5sum for a message object of type 'Vector7d"
  "e4844fe6885794a5295df1d81cbae79f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Vector7d>)))
  "Returns full string definition for message of type '<Vector7d>"
  (cl:format cl:nil "float32[7] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Vector7d)))
  "Returns full string definition for message of type 'Vector7d"
  (cl:format cl:nil "float32[7] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Vector7d>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Vector7d>))
  "Converts a ROS message object to a list"
  (cl:list 'Vector7d
    (cl:cons ':data (data msg))
))
