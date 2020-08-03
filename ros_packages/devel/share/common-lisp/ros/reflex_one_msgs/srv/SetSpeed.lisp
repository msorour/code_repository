; Auto-generated. Do not edit!


(cl:in-package reflex_one_msgs-srv)


;//! \htmlinclude SetSpeed-request.msg.html

(cl:defclass <SetSpeed-request> (roslisp-msg-protocol:ros-message)
  ((motor
    :reader motor
    :initarg :motor
    :type (cl:vector cl:float)
   :initform (cl:make-array 5 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SetSpeed-request (<SetSpeed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSpeed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSpeed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reflex_one_msgs-srv:<SetSpeed-request> is deprecated: use reflex_one_msgs-srv:SetSpeed-request instead.")))

(cl:ensure-generic-function 'motor-val :lambda-list '(m))
(cl:defmethod motor-val ((m <SetSpeed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_one_msgs-srv:motor-val is deprecated.  Use reflex_one_msgs-srv:motor instead.")
  (motor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSpeed-request>) ostream)
  "Serializes a message object of type '<SetSpeed-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'motor))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSpeed-request>) istream)
  "Deserializes a message object of type '<SetSpeed-request>"
  (cl:setf (cl:slot-value msg 'motor) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'motor)))
    (cl:dotimes (i 5)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSpeed-request>)))
  "Returns string type for a service object of type '<SetSpeed-request>"
  "reflex_one_msgs/SetSpeedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSpeed-request)))
  "Returns string type for a service object of type 'SetSpeed-request"
  "reflex_one_msgs/SetSpeedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSpeed-request>)))
  "Returns md5sum for a message object of type '<SetSpeed-request>"
  "04171ebc3d171407826cbe90dea5b330")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSpeed-request)))
  "Returns md5sum for a message object of type 'SetSpeed-request"
  "04171ebc3d171407826cbe90dea5b330")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSpeed-request>)))
  "Returns full string definition for message of type '<SetSpeed-request>"
  (cl:format cl:nil "# message for calling hand commands~%float64[5] motor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSpeed-request)))
  "Returns full string definition for message of type 'SetSpeed-request"
  (cl:format cl:nil "# message for calling hand commands~%float64[5] motor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSpeed-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'motor) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSpeed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSpeed-request
    (cl:cons ':motor (motor msg))
))
;//! \htmlinclude SetSpeed-response.msg.html

(cl:defclass <SetSpeed-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetSpeed-response (<SetSpeed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSpeed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSpeed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reflex_one_msgs-srv:<SetSpeed-response> is deprecated: use reflex_one_msgs-srv:SetSpeed-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSpeed-response>) ostream)
  "Serializes a message object of type '<SetSpeed-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSpeed-response>) istream)
  "Deserializes a message object of type '<SetSpeed-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSpeed-response>)))
  "Returns string type for a service object of type '<SetSpeed-response>"
  "reflex_one_msgs/SetSpeedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSpeed-response)))
  "Returns string type for a service object of type 'SetSpeed-response"
  "reflex_one_msgs/SetSpeedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSpeed-response>)))
  "Returns md5sum for a message object of type '<SetSpeed-response>"
  "04171ebc3d171407826cbe90dea5b330")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSpeed-response)))
  "Returns md5sum for a message object of type 'SetSpeed-response"
  "04171ebc3d171407826cbe90dea5b330")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSpeed-response>)))
  "Returns full string definition for message of type '<SetSpeed-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSpeed-response)))
  "Returns full string definition for message of type 'SetSpeed-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSpeed-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSpeed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSpeed-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetSpeed)))
  'SetSpeed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetSpeed)))
  'SetSpeed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSpeed)))
  "Returns string type for a service object of type '<SetSpeed>"
  "reflex_one_msgs/SetSpeed")