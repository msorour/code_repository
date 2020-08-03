; Auto-generated. Do not edit!


(cl:in-package reflex_msgs2-msg)


;//! \htmlinclude FingerPressure.msg.html

(cl:defclass <FingerPressure> (roslisp-msg-protocol:ros-message)
  ((sensor
    :reader sensor
    :initarg :sensor
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 14 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass FingerPressure (<FingerPressure>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FingerPressure>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FingerPressure)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reflex_msgs2-msg:<FingerPressure> is deprecated: use reflex_msgs2-msg:FingerPressure instead.")))

(cl:ensure-generic-function 'sensor-val :lambda-list '(m))
(cl:defmethod sensor-val ((m <FingerPressure>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:sensor-val is deprecated.  Use reflex_msgs2-msg:sensor instead.")
  (sensor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FingerPressure>) ostream)
  "Serializes a message object of type '<FingerPressure>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'sensor))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FingerPressure>) istream)
  "Deserializes a message object of type '<FingerPressure>"
  (cl:setf (cl:slot-value msg 'sensor) (cl:make-array 14))
  (cl:let ((vals (cl:slot-value msg 'sensor)))
    (cl:dotimes (i 14)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FingerPressure>)))
  "Returns string type for a message object of type '<FingerPressure>"
  "reflex_msgs2/FingerPressure")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FingerPressure)))
  "Returns string type for a message object of type 'FingerPressure"
  "reflex_msgs2/FingerPressure")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FingerPressure>)))
  "Returns md5sum for a message object of type '<FingerPressure>"
  "2fecdd6322f86c468a82244a96936129")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FingerPressure)))
  "Returns md5sum for a message object of type 'FingerPressure"
  "2fecdd6322f86c468a82244a96936129")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FingerPressure>)))
  "Returns full string definition for message of type '<FingerPressure>"
  (cl:format cl:nil "# message for pressure on a single finger~%uint16[14] sensor    	# The sensors enumerate from the base of the finger to the tip~%						# There are 5 on the proximal link, 4 on the distal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FingerPressure)))
  "Returns full string definition for message of type 'FingerPressure"
  (cl:format cl:nil "# message for pressure on a single finger~%uint16[14] sensor    	# The sensors enumerate from the base of the finger to the tip~%						# There are 5 on the proximal link, 4 on the distal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FingerPressure>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'sensor) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FingerPressure>))
  "Converts a ROS message object to a list"
  (cl:list 'FingerPressure
    (cl:cons ':sensor (sensor msg))
))
