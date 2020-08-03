; Auto-generated. Do not edit!


(cl:in-package reflex_msgs-srv)


;//! \htmlinclude SetTactileThreshold-request.msg.html

(cl:defclass <SetTactileThreshold-request> (roslisp-msg-protocol:ros-message)
  ((finger
    :reader finger
    :initarg :finger
    :type (cl:vector reflex_msgs-msg:FingerPressure)
   :initform (cl:make-array 3 :element-type 'reflex_msgs-msg:FingerPressure :initial-element (cl:make-instance 'reflex_msgs-msg:FingerPressure))))
)

(cl:defclass SetTactileThreshold-request (<SetTactileThreshold-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTactileThreshold-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTactileThreshold-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reflex_msgs-srv:<SetTactileThreshold-request> is deprecated: use reflex_msgs-srv:SetTactileThreshold-request instead.")))

(cl:ensure-generic-function 'finger-val :lambda-list '(m))
(cl:defmethod finger-val ((m <SetTactileThreshold-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs-srv:finger-val is deprecated.  Use reflex_msgs-srv:finger instead.")
  (finger m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTactileThreshold-request>) ostream)
  "Serializes a message object of type '<SetTactileThreshold-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'finger))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTactileThreshold-request>) istream)
  "Deserializes a message object of type '<SetTactileThreshold-request>"
  (cl:setf (cl:slot-value msg 'finger) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'finger)))
    (cl:dotimes (i 3)
    (cl:setf (cl:aref vals i) (cl:make-instance 'reflex_msgs-msg:FingerPressure))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTactileThreshold-request>)))
  "Returns string type for a service object of type '<SetTactileThreshold-request>"
  "reflex_msgs/SetTactileThresholdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTactileThreshold-request)))
  "Returns string type for a service object of type 'SetTactileThreshold-request"
  "reflex_msgs/SetTactileThresholdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTactileThreshold-request>)))
  "Returns md5sum for a message object of type '<SetTactileThreshold-request>"
  "01cec83f9d223083364c730460331524")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTactileThreshold-request)))
  "Returns md5sum for a message object of type 'SetTactileThreshold-request"
  "01cec83f9d223083364c730460331524")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTactileThreshold-request>)))
  "Returns full string definition for message of type '<SetTactileThreshold-request>"
  (cl:format cl:nil "# message for calling setting pressure thresholds on various fingers~%FingerPressure[3] finger~%~%================================================================================~%MSG: reflex_msgs/FingerPressure~%# message for pressure on a single finger~%uint16[9] sensor    	# The sensors enumerate from the base of the finger to the tip~%						# There are 5 on the proximal link, 4 on the distal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTactileThreshold-request)))
  "Returns full string definition for message of type 'SetTactileThreshold-request"
  (cl:format cl:nil "# message for calling setting pressure thresholds on various fingers~%FingerPressure[3] finger~%~%================================================================================~%MSG: reflex_msgs/FingerPressure~%# message for pressure on a single finger~%uint16[9] sensor    	# The sensors enumerate from the base of the finger to the tip~%						# There are 5 on the proximal link, 4 on the distal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTactileThreshold-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'finger) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTactileThreshold-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTactileThreshold-request
    (cl:cons ':finger (finger msg))
))
;//! \htmlinclude SetTactileThreshold-response.msg.html

(cl:defclass <SetTactileThreshold-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetTactileThreshold-response (<SetTactileThreshold-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTactileThreshold-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTactileThreshold-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reflex_msgs-srv:<SetTactileThreshold-response> is deprecated: use reflex_msgs-srv:SetTactileThreshold-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTactileThreshold-response>) ostream)
  "Serializes a message object of type '<SetTactileThreshold-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTactileThreshold-response>) istream)
  "Deserializes a message object of type '<SetTactileThreshold-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTactileThreshold-response>)))
  "Returns string type for a service object of type '<SetTactileThreshold-response>"
  "reflex_msgs/SetTactileThresholdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTactileThreshold-response)))
  "Returns string type for a service object of type 'SetTactileThreshold-response"
  "reflex_msgs/SetTactileThresholdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTactileThreshold-response>)))
  "Returns md5sum for a message object of type '<SetTactileThreshold-response>"
  "01cec83f9d223083364c730460331524")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTactileThreshold-response)))
  "Returns md5sum for a message object of type 'SetTactileThreshold-response"
  "01cec83f9d223083364c730460331524")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTactileThreshold-response>)))
  "Returns full string definition for message of type '<SetTactileThreshold-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTactileThreshold-response)))
  "Returns full string definition for message of type 'SetTactileThreshold-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTactileThreshold-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTactileThreshold-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTactileThreshold-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetTactileThreshold)))
  'SetTactileThreshold-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetTactileThreshold)))
  'SetTactileThreshold-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTactileThreshold)))
  "Returns string type for a service object of type '<SetTactileThreshold>"
  "reflex_msgs/SetTactileThreshold")