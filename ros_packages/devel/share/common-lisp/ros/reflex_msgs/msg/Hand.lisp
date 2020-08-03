; Auto-generated. Do not edit!


(cl:in-package reflex_msgs-msg)


;//! \htmlinclude Hand.msg.html

(cl:defclass <Hand> (roslisp-msg-protocol:ros-message)
  ((finger
    :reader finger
    :initarg :finger
    :type (cl:vector reflex_msgs-msg:Finger)
   :initform (cl:make-array 3 :element-type 'reflex_msgs-msg:Finger :initial-element (cl:make-instance 'reflex_msgs-msg:Finger)))
   (motor
    :reader motor
    :initarg :motor
    :type (cl:vector reflex_msgs-msg:Motor)
   :initform (cl:make-array 4 :element-type 'reflex_msgs-msg:Motor :initial-element (cl:make-instance 'reflex_msgs-msg:Motor))))
)

(cl:defclass Hand (<Hand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Hand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Hand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reflex_msgs-msg:<Hand> is deprecated: use reflex_msgs-msg:Hand instead.")))

(cl:ensure-generic-function 'finger-val :lambda-list '(m))
(cl:defmethod finger-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs-msg:finger-val is deprecated.  Use reflex_msgs-msg:finger instead.")
  (finger m))

(cl:ensure-generic-function 'motor-val :lambda-list '(m))
(cl:defmethod motor-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs-msg:motor-val is deprecated.  Use reflex_msgs-msg:motor instead.")
  (motor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hand>) ostream)
  "Serializes a message object of type '<Hand>"
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'finger))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'motor))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hand>) istream)
  "Deserializes a message object of type '<Hand>"
  (cl:setf (cl:slot-value msg 'finger) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'finger)))
    (cl:dotimes (i 3)
    (cl:setf (cl:aref vals i) (cl:make-instance 'reflex_msgs-msg:Finger))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'motor) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'motor)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:make-instance 'reflex_msgs-msg:Motor))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Hand>)))
  "Returns string type for a message object of type '<Hand>"
  "reflex_msgs/Hand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hand)))
  "Returns string type for a message object of type 'Hand"
  "reflex_msgs/Hand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Hand>)))
  "Returns md5sum for a message object of type '<Hand>"
  "5328a4ca4ff4a136db2555c3178a3e1b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hand)))
  "Returns md5sum for a message object of type 'Hand"
  "5328a4ca4ff4a136db2555c3178a3e1b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hand>)))
  "Returns full string definition for message of type '<Hand>"
  (cl:format cl:nil "# message for ReFlex Hand~%Finger[3] finger       # Hold out your right hand palm up, with pointer finger, middle finger and thumb extended~%                       # Pointer = finger[0], Middle = finger[1], Thumb = finger[2]~%Motor[4] motor         # Finger 1, Finger 2, Finger 3, and Preshape~%~%================================================================================~%MSG: reflex_msgs/Finger~%# message for ReFlex Fingers~%float32 proximal		# radians, measured from all open = 0, to pi = closed~%float32 distal_approx   # radians, measured from all open = 0, to roughly pi = against proximal pad, relative to prox link~%bool[9] contact			# binary, 0 = proximal, 8 = fingertip~%float32[9] pressure		# scalar, dimensionless units, 0 = proximal, 8 = fingertip (can go negative)~%~%================================================================================~%MSG: reflex_msgs/Motor~%float64 joint_angle~%float64 raw_angle~%float64 velocity~%float64 load~%float64 voltage~%int32 temperature~%string error_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hand)))
  "Returns full string definition for message of type 'Hand"
  (cl:format cl:nil "# message for ReFlex Hand~%Finger[3] finger       # Hold out your right hand palm up, with pointer finger, middle finger and thumb extended~%                       # Pointer = finger[0], Middle = finger[1], Thumb = finger[2]~%Motor[4] motor         # Finger 1, Finger 2, Finger 3, and Preshape~%~%================================================================================~%MSG: reflex_msgs/Finger~%# message for ReFlex Fingers~%float32 proximal		# radians, measured from all open = 0, to pi = closed~%float32 distal_approx   # radians, measured from all open = 0, to roughly pi = against proximal pad, relative to prox link~%bool[9] contact			# binary, 0 = proximal, 8 = fingertip~%float32[9] pressure		# scalar, dimensionless units, 0 = proximal, 8 = fingertip (can go negative)~%~%================================================================================~%MSG: reflex_msgs/Motor~%float64 joint_angle~%float64 raw_angle~%float64 velocity~%float64 load~%float64 voltage~%int32 temperature~%string error_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hand>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'finger) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'motor) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hand>))
  "Converts a ROS message object to a list"
  (cl:list 'Hand
    (cl:cons ':finger (finger msg))
    (cl:cons ':motor (motor msg))
))
