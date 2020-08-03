; Auto-generated. Do not edit!


(cl:in-package reflex_msgs2-srv)


;//! \htmlinclude DistalRotation-request.msg.html

(cl:defclass <DistalRotation-request> (roslisp-msg-protocol:ros-message)
  ((palm_imu_quat
    :reader palm_imu_quat
    :initarg :palm_imu_quat
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (joint_angle
    :reader joint_angle
    :initarg :joint_angle
    :type cl:float
    :initform 0.0)
   (proximal
    :reader proximal
    :initarg :proximal
    :type cl:float
    :initform 0.0)
   (finger_imu_quat
    :reader finger_imu_quat
    :initarg :finger_imu_quat
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass DistalRotation-request (<DistalRotation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DistalRotation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DistalRotation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reflex_msgs2-srv:<DistalRotation-request> is deprecated: use reflex_msgs2-srv:DistalRotation-request instead.")))

(cl:ensure-generic-function 'palm_imu_quat-val :lambda-list '(m))
(cl:defmethod palm_imu_quat-val ((m <DistalRotation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-srv:palm_imu_quat-val is deprecated.  Use reflex_msgs2-srv:palm_imu_quat instead.")
  (palm_imu_quat m))

(cl:ensure-generic-function 'joint_angle-val :lambda-list '(m))
(cl:defmethod joint_angle-val ((m <DistalRotation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-srv:joint_angle-val is deprecated.  Use reflex_msgs2-srv:joint_angle instead.")
  (joint_angle m))

(cl:ensure-generic-function 'proximal-val :lambda-list '(m))
(cl:defmethod proximal-val ((m <DistalRotation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-srv:proximal-val is deprecated.  Use reflex_msgs2-srv:proximal instead.")
  (proximal m))

(cl:ensure-generic-function 'finger_imu_quat-val :lambda-list '(m))
(cl:defmethod finger_imu_quat-val ((m <DistalRotation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-srv:finger_imu_quat-val is deprecated.  Use reflex_msgs2-srv:finger_imu_quat instead.")
  (finger_imu_quat m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DistalRotation-request>) ostream)
  "Serializes a message object of type '<DistalRotation-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'palm_imu_quat))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'joint_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'proximal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'finger_imu_quat))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DistalRotation-request>) istream)
  "Deserializes a message object of type '<DistalRotation-request>"
  (cl:setf (cl:slot-value msg 'palm_imu_quat) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'palm_imu_quat)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'joint_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'proximal) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'finger_imu_quat) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'finger_imu_quat)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DistalRotation-request>)))
  "Returns string type for a service object of type '<DistalRotation-request>"
  "reflex_msgs2/DistalRotationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DistalRotation-request)))
  "Returns string type for a service object of type 'DistalRotation-request"
  "reflex_msgs2/DistalRotationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DistalRotation-request>)))
  "Returns md5sum for a message object of type '<DistalRotation-request>"
  "96436694e12dc909600e6760ed1174cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DistalRotation-request)))
  "Returns md5sum for a message object of type 'DistalRotation-request"
  "96436694e12dc909600e6760ed1174cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DistalRotation-request>)))
  "Returns full string definition for message of type '<DistalRotation-request>"
  (cl:format cl:nil "# message for reading IMU rotations~%float32[4] palm_imu_quat~%float32 joint_angle~%float32 proximal~%float32[4] finger_imu_quat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DistalRotation-request)))
  "Returns full string definition for message of type 'DistalRotation-request"
  (cl:format cl:nil "# message for reading IMU rotations~%float32[4] palm_imu_quat~%float32 joint_angle~%float32 proximal~%float32[4] finger_imu_quat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DistalRotation-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'palm_imu_quat) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'finger_imu_quat) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DistalRotation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DistalRotation-request
    (cl:cons ':palm_imu_quat (palm_imu_quat msg))
    (cl:cons ':joint_angle (joint_angle msg))
    (cl:cons ':proximal (proximal msg))
    (cl:cons ':finger_imu_quat (finger_imu_quat msg))
))
;//! \htmlinclude DistalRotation-response.msg.html

(cl:defclass <DistalRotation-response> (roslisp-msg-protocol:ros-message)
  ((rotation
    :reader rotation
    :initarg :rotation
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass DistalRotation-response (<DistalRotation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DistalRotation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DistalRotation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reflex_msgs2-srv:<DistalRotation-response> is deprecated: use reflex_msgs2-srv:DistalRotation-response instead.")))

(cl:ensure-generic-function 'rotation-val :lambda-list '(m))
(cl:defmethod rotation-val ((m <DistalRotation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-srv:rotation-val is deprecated.  Use reflex_msgs2-srv:rotation instead.")
  (rotation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DistalRotation-response>) ostream)
  "Serializes a message object of type '<DistalRotation-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'rotation))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DistalRotation-response>) istream)
  "Deserializes a message object of type '<DistalRotation-response>"
  (cl:setf (cl:slot-value msg 'rotation) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'rotation)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DistalRotation-response>)))
  "Returns string type for a service object of type '<DistalRotation-response>"
  "reflex_msgs2/DistalRotationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DistalRotation-response)))
  "Returns string type for a service object of type 'DistalRotation-response"
  "reflex_msgs2/DistalRotationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DistalRotation-response>)))
  "Returns md5sum for a message object of type '<DistalRotation-response>"
  "96436694e12dc909600e6760ed1174cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DistalRotation-response)))
  "Returns md5sum for a message object of type 'DistalRotation-response"
  "96436694e12dc909600e6760ed1174cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DistalRotation-response>)))
  "Returns full string definition for message of type '<DistalRotation-response>"
  (cl:format cl:nil "float32[3] rotation~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DistalRotation-response)))
  "Returns full string definition for message of type 'DistalRotation-response"
  (cl:format cl:nil "float32[3] rotation~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DistalRotation-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'rotation) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DistalRotation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DistalRotation-response
    (cl:cons ':rotation (rotation msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DistalRotation)))
  'DistalRotation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DistalRotation)))
  'DistalRotation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DistalRotation)))
  "Returns string type for a service object of type '<DistalRotation>"
  "reflex_msgs2/DistalRotation")