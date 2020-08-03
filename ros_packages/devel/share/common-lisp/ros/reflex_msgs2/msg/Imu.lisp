; Auto-generated. Do not edit!


(cl:in-package reflex_msgs2-msg)


;//! \htmlinclude Imu.msg.html

(cl:defclass <Imu> (roslisp-msg-protocol:ros-message)
  ((quat
    :reader quat
    :initarg :quat
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (euler_angles
    :reader euler_angles
    :initarg :euler_angles
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (calibration_status
    :reader calibration_status
    :initarg :calibration_status
    :type cl:fixnum
    :initform 0)
   (calibration_data
    :reader calibration_data
    :initarg :calibration_data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 11 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Imu (<Imu>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Imu>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Imu)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reflex_msgs2-msg:<Imu> is deprecated: use reflex_msgs2-msg:Imu instead.")))

(cl:ensure-generic-function 'quat-val :lambda-list '(m))
(cl:defmethod quat-val ((m <Imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:quat-val is deprecated.  Use reflex_msgs2-msg:quat instead.")
  (quat m))

(cl:ensure-generic-function 'euler_angles-val :lambda-list '(m))
(cl:defmethod euler_angles-val ((m <Imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:euler_angles-val is deprecated.  Use reflex_msgs2-msg:euler_angles instead.")
  (euler_angles m))

(cl:ensure-generic-function 'calibration_status-val :lambda-list '(m))
(cl:defmethod calibration_status-val ((m <Imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:calibration_status-val is deprecated.  Use reflex_msgs2-msg:calibration_status instead.")
  (calibration_status m))

(cl:ensure-generic-function 'calibration_data-val :lambda-list '(m))
(cl:defmethod calibration_data-val ((m <Imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:calibration_data-val is deprecated.  Use reflex_msgs2-msg:calibration_data instead.")
  (calibration_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Imu>) ostream)
  "Serializes a message object of type '<Imu>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'quat))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'euler_angles))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'calibration_status)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'calibration_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Imu>) istream)
  "Deserializes a message object of type '<Imu>"
  (cl:setf (cl:slot-value msg 'quat) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'quat)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'euler_angles) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'euler_angles)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'calibration_status)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'calibration_data) (cl:make-array 11))
  (cl:let ((vals (cl:slot-value msg 'calibration_data)))
    (cl:dotimes (i 11)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Imu>)))
  "Returns string type for a message object of type '<Imu>"
  "reflex_msgs2/Imu")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Imu)))
  "Returns string type for a message object of type 'Imu"
  "reflex_msgs2/Imu")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Imu>)))
  "Returns md5sum for a message object of type '<Imu>"
  "dea5c53d0c934f48b1ee6e5a6eed4389")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Imu)))
  "Returns md5sum for a message object of type 'Imu"
  "dea5c53d0c934f48b1ee6e5a6eed4389")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Imu>)))
  "Returns full string definition for message of type '<Imu>"
  (cl:format cl:nil "# quaternion reading from IMU (w, x, y, z)~%float32[4] quat	~%float32[3] euler_angles ~%~%# these are defined in reflex_hand.h driver~%uint8 calibration_status ~%uint16[11] calibration_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Imu)))
  "Returns full string definition for message of type 'Imu"
  (cl:format cl:nil "# quaternion reading from IMU (w, x, y, z)~%float32[4] quat	~%float32[3] euler_angles ~%~%# these are defined in reflex_hand.h driver~%uint8 calibration_status ~%uint16[11] calibration_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Imu>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'quat) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'euler_angles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'calibration_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Imu>))
  "Converts a ROS message object to a list"
  (cl:list 'Imu
    (cl:cons ':quat (quat msg))
    (cl:cons ':euler_angles (euler_angles msg))
    (cl:cons ':calibration_status (calibration_status msg))
    (cl:cons ':calibration_data (calibration_data msg))
))
