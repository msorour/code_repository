; Auto-generated. Do not edit!


(cl:in-package reflex_msgs2-msg)


;//! \htmlinclude ImuCalibrationData.msg.html

(cl:defclass <ImuCalibrationData> (roslisp-msg-protocol:ros-message)
  ((acc_offset_f1
    :reader acc_offset_f1
    :initarg :acc_offset_f1
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (acc_offset_f2
    :reader acc_offset_f2
    :initarg :acc_offset_f2
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (acc_offset_f3
    :reader acc_offset_f3
    :initarg :acc_offset_f3
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (acc_offset_palm
    :reader acc_offset_palm
    :initarg :acc_offset_palm
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (mag_offset_f1
    :reader mag_offset_f1
    :initarg :mag_offset_f1
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (mag_offset_f2
    :reader mag_offset_f2
    :initarg :mag_offset_f2
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (mag_offset_f3
    :reader mag_offset_f3
    :initarg :mag_offset_f3
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (mag_offset_palm
    :reader mag_offset_palm
    :initarg :mag_offset_palm
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (gyr_offset_f1
    :reader gyr_offset_f1
    :initarg :gyr_offset_f1
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (gyr_offset_f2
    :reader gyr_offset_f2
    :initarg :gyr_offset_f2
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (gyr_offset_f3
    :reader gyr_offset_f3
    :initarg :gyr_offset_f3
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (gyr_offset_palm
    :reader gyr_offset_palm
    :initarg :gyr_offset_palm
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (acc_radius
    :reader acc_radius
    :initarg :acc_radius
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (gyr_radius
    :reader gyr_radius
    :initarg :gyr_radius
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass ImuCalibrationData (<ImuCalibrationData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImuCalibrationData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImuCalibrationData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reflex_msgs2-msg:<ImuCalibrationData> is deprecated: use reflex_msgs2-msg:ImuCalibrationData instead.")))

(cl:ensure-generic-function 'acc_offset_f1-val :lambda-list '(m))
(cl:defmethod acc_offset_f1-val ((m <ImuCalibrationData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:acc_offset_f1-val is deprecated.  Use reflex_msgs2-msg:acc_offset_f1 instead.")
  (acc_offset_f1 m))

(cl:ensure-generic-function 'acc_offset_f2-val :lambda-list '(m))
(cl:defmethod acc_offset_f2-val ((m <ImuCalibrationData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:acc_offset_f2-val is deprecated.  Use reflex_msgs2-msg:acc_offset_f2 instead.")
  (acc_offset_f2 m))

(cl:ensure-generic-function 'acc_offset_f3-val :lambda-list '(m))
(cl:defmethod acc_offset_f3-val ((m <ImuCalibrationData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:acc_offset_f3-val is deprecated.  Use reflex_msgs2-msg:acc_offset_f3 instead.")
  (acc_offset_f3 m))

(cl:ensure-generic-function 'acc_offset_palm-val :lambda-list '(m))
(cl:defmethod acc_offset_palm-val ((m <ImuCalibrationData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:acc_offset_palm-val is deprecated.  Use reflex_msgs2-msg:acc_offset_palm instead.")
  (acc_offset_palm m))

(cl:ensure-generic-function 'mag_offset_f1-val :lambda-list '(m))
(cl:defmethod mag_offset_f1-val ((m <ImuCalibrationData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:mag_offset_f1-val is deprecated.  Use reflex_msgs2-msg:mag_offset_f1 instead.")
  (mag_offset_f1 m))

(cl:ensure-generic-function 'mag_offset_f2-val :lambda-list '(m))
(cl:defmethod mag_offset_f2-val ((m <ImuCalibrationData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:mag_offset_f2-val is deprecated.  Use reflex_msgs2-msg:mag_offset_f2 instead.")
  (mag_offset_f2 m))

(cl:ensure-generic-function 'mag_offset_f3-val :lambda-list '(m))
(cl:defmethod mag_offset_f3-val ((m <ImuCalibrationData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:mag_offset_f3-val is deprecated.  Use reflex_msgs2-msg:mag_offset_f3 instead.")
  (mag_offset_f3 m))

(cl:ensure-generic-function 'mag_offset_palm-val :lambda-list '(m))
(cl:defmethod mag_offset_palm-val ((m <ImuCalibrationData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:mag_offset_palm-val is deprecated.  Use reflex_msgs2-msg:mag_offset_palm instead.")
  (mag_offset_palm m))

(cl:ensure-generic-function 'gyr_offset_f1-val :lambda-list '(m))
(cl:defmethod gyr_offset_f1-val ((m <ImuCalibrationData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:gyr_offset_f1-val is deprecated.  Use reflex_msgs2-msg:gyr_offset_f1 instead.")
  (gyr_offset_f1 m))

(cl:ensure-generic-function 'gyr_offset_f2-val :lambda-list '(m))
(cl:defmethod gyr_offset_f2-val ((m <ImuCalibrationData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:gyr_offset_f2-val is deprecated.  Use reflex_msgs2-msg:gyr_offset_f2 instead.")
  (gyr_offset_f2 m))

(cl:ensure-generic-function 'gyr_offset_f3-val :lambda-list '(m))
(cl:defmethod gyr_offset_f3-val ((m <ImuCalibrationData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:gyr_offset_f3-val is deprecated.  Use reflex_msgs2-msg:gyr_offset_f3 instead.")
  (gyr_offset_f3 m))

(cl:ensure-generic-function 'gyr_offset_palm-val :lambda-list '(m))
(cl:defmethod gyr_offset_palm-val ((m <ImuCalibrationData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:gyr_offset_palm-val is deprecated.  Use reflex_msgs2-msg:gyr_offset_palm instead.")
  (gyr_offset_palm m))

(cl:ensure-generic-function 'acc_radius-val :lambda-list '(m))
(cl:defmethod acc_radius-val ((m <ImuCalibrationData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:acc_radius-val is deprecated.  Use reflex_msgs2-msg:acc_radius instead.")
  (acc_radius m))

(cl:ensure-generic-function 'gyr_radius-val :lambda-list '(m))
(cl:defmethod gyr_radius-val ((m <ImuCalibrationData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:gyr_radius-val is deprecated.  Use reflex_msgs2-msg:gyr_radius instead.")
  (gyr_radius m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImuCalibrationData>) ostream)
  "Serializes a message object of type '<ImuCalibrationData>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'acc_offset_f1))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'acc_offset_f2))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'acc_offset_f3))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'acc_offset_palm))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'mag_offset_f1))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'mag_offset_f2))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'mag_offset_f3))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'mag_offset_palm))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'gyr_offset_f1))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'gyr_offset_f2))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'gyr_offset_f3))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'gyr_offset_palm))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'acc_radius))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'gyr_radius))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImuCalibrationData>) istream)
  "Deserializes a message object of type '<ImuCalibrationData>"
  (cl:setf (cl:slot-value msg 'acc_offset_f1) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'acc_offset_f1)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'acc_offset_f2) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'acc_offset_f2)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'acc_offset_f3) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'acc_offset_f3)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'acc_offset_palm) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'acc_offset_palm)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'mag_offset_f1) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'mag_offset_f1)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'mag_offset_f2) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'mag_offset_f2)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'mag_offset_f3) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'mag_offset_f3)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'mag_offset_palm) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'mag_offset_palm)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'gyr_offset_f1) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'gyr_offset_f1)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'gyr_offset_f2) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'gyr_offset_f2)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'gyr_offset_f3) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'gyr_offset_f3)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'gyr_offset_palm) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'gyr_offset_palm)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'acc_radius) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'acc_radius)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'gyr_radius) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'gyr_radius)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImuCalibrationData>)))
  "Returns string type for a message object of type '<ImuCalibrationData>"
  "reflex_msgs2/ImuCalibrationData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImuCalibrationData)))
  "Returns string type for a message object of type 'ImuCalibrationData"
  "reflex_msgs2/ImuCalibrationData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImuCalibrationData>)))
  "Returns md5sum for a message object of type '<ImuCalibrationData>"
  "1ef3e1b102a68813a645fa51b970838b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImuCalibrationData)))
  "Returns md5sum for a message object of type 'ImuCalibrationData"
  "1ef3e1b102a68813a645fa51b970838b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImuCalibrationData>)))
  "Returns full string definition for message of type '<ImuCalibrationData>"
  (cl:format cl:nil "# Acceleration offsets for each dimension in the format [x,y,z]~%uint16[3] acc_offset_f1~%uint16[3] acc_offset_f2~%uint16[3] acc_offset_f3~%uint16[3] acc_offset_palm~%~%# Magnetometer offsets for each dimension in the format [x,y,z]~%uint16[3] mag_offset_f1~%uint16[3] mag_offset_f2~%uint16[3] mag_offset_f3~%uint16[3] mag_offset_palm~%~%# Gyroscope offsets for each dimension in the format [x,y,z]~%uint16[3] gyr_offset_f1~%uint16[3] gyr_offset_f2~%uint16[3] gyr_offset_f3~%uint16[3] gyr_offset_palm~%~%# Accelerometer and Gyroscope radius in the format [f1,f2,f3,palm]~%uint16[4] acc_radius~%uint16[4] gyr_radius~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImuCalibrationData)))
  "Returns full string definition for message of type 'ImuCalibrationData"
  (cl:format cl:nil "# Acceleration offsets for each dimension in the format [x,y,z]~%uint16[3] acc_offset_f1~%uint16[3] acc_offset_f2~%uint16[3] acc_offset_f3~%uint16[3] acc_offset_palm~%~%# Magnetometer offsets for each dimension in the format [x,y,z]~%uint16[3] mag_offset_f1~%uint16[3] mag_offset_f2~%uint16[3] mag_offset_f3~%uint16[3] mag_offset_palm~%~%# Gyroscope offsets for each dimension in the format [x,y,z]~%uint16[3] gyr_offset_f1~%uint16[3] gyr_offset_f2~%uint16[3] gyr_offset_f3~%uint16[3] gyr_offset_palm~%~%# Accelerometer and Gyroscope radius in the format [f1,f2,f3,palm]~%uint16[4] acc_radius~%uint16[4] gyr_radius~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImuCalibrationData>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'acc_offset_f1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'acc_offset_f2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'acc_offset_f3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'acc_offset_palm) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mag_offset_f1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mag_offset_f2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mag_offset_f3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mag_offset_palm) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gyr_offset_f1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gyr_offset_f2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gyr_offset_f3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gyr_offset_palm) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'acc_radius) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gyr_radius) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImuCalibrationData>))
  "Converts a ROS message object to a list"
  (cl:list 'ImuCalibrationData
    (cl:cons ':acc_offset_f1 (acc_offset_f1 msg))
    (cl:cons ':acc_offset_f2 (acc_offset_f2 msg))
    (cl:cons ':acc_offset_f3 (acc_offset_f3 msg))
    (cl:cons ':acc_offset_palm (acc_offset_palm msg))
    (cl:cons ':mag_offset_f1 (mag_offset_f1 msg))
    (cl:cons ':mag_offset_f2 (mag_offset_f2 msg))
    (cl:cons ':mag_offset_f3 (mag_offset_f3 msg))
    (cl:cons ':mag_offset_palm (mag_offset_palm msg))
    (cl:cons ':gyr_offset_f1 (gyr_offset_f1 msg))
    (cl:cons ':gyr_offset_f2 (gyr_offset_f2 msg))
    (cl:cons ':gyr_offset_f3 (gyr_offset_f3 msg))
    (cl:cons ':gyr_offset_palm (gyr_offset_palm msg))
    (cl:cons ':acc_radius (acc_radius msg))
    (cl:cons ':gyr_radius (gyr_radius msg))
))
