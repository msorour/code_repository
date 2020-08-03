; Auto-generated. Do not edit!


(cl:in-package reflex_one_msgs-msg)


;//! \htmlinclude Finger.msg.html

(cl:defclass <Finger> (roslisp-msg-protocol:ros-message)
  ((proximal
    :reader proximal
    :initarg :proximal
    :type cl:float
    :initform 0.0)
   (distal_approx
    :reader distal_approx
    :initarg :distal_approx
    :type cl:float
    :initform 0.0)
   (contact
    :reader contact
    :initarg :contact
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 9 :element-type 'cl:boolean :initial-element cl:nil))
   (pressure
    :reader pressure
    :initarg :pressure
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Finger (<Finger>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Finger>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Finger)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reflex_one_msgs-msg:<Finger> is deprecated: use reflex_one_msgs-msg:Finger instead.")))

(cl:ensure-generic-function 'proximal-val :lambda-list '(m))
(cl:defmethod proximal-val ((m <Finger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_one_msgs-msg:proximal-val is deprecated.  Use reflex_one_msgs-msg:proximal instead.")
  (proximal m))

(cl:ensure-generic-function 'distal_approx-val :lambda-list '(m))
(cl:defmethod distal_approx-val ((m <Finger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_one_msgs-msg:distal_approx-val is deprecated.  Use reflex_one_msgs-msg:distal_approx instead.")
  (distal_approx m))

(cl:ensure-generic-function 'contact-val :lambda-list '(m))
(cl:defmethod contact-val ((m <Finger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_one_msgs-msg:contact-val is deprecated.  Use reflex_one_msgs-msg:contact instead.")
  (contact m))

(cl:ensure-generic-function 'pressure-val :lambda-list '(m))
(cl:defmethod pressure-val ((m <Finger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_one_msgs-msg:pressure-val is deprecated.  Use reflex_one_msgs-msg:pressure instead.")
  (pressure m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Finger>) ostream)
  "Serializes a message object of type '<Finger>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'proximal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distal_approx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'contact))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pressure))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Finger>) istream)
  "Deserializes a message object of type '<Finger>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'proximal) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distal_approx) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'contact) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'contact)))
    (cl:dotimes (i 9)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  (cl:setf (cl:slot-value msg 'pressure) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'pressure)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Finger>)))
  "Returns string type for a message object of type '<Finger>"
  "reflex_one_msgs/Finger")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Finger)))
  "Returns string type for a message object of type 'Finger"
  "reflex_one_msgs/Finger")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Finger>)))
  "Returns md5sum for a message object of type '<Finger>"
  "b5232f74e901b48063f64cfc32aefbe0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Finger)))
  "Returns md5sum for a message object of type 'Finger"
  "b5232f74e901b48063f64cfc32aefbe0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Finger>)))
  "Returns full string definition for message of type '<Finger>"
  (cl:format cl:nil "# message for ReFlex Fingers~%float32 proximal		# radians, measured from all open = 0, to pi = closed~%float32 distal_approx   # radians, measured from all open = 0, to roughly pi = against proximal pad, relative to prox link~%bool[9] contact			# binary, 0 = proximal, 8 = fingertip~%float32[9] pressure		# scalar, dimensionless units, 0 = proximal, 8 = fingertip (can go negative)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Finger)))
  "Returns full string definition for message of type 'Finger"
  (cl:format cl:nil "# message for ReFlex Fingers~%float32 proximal		# radians, measured from all open = 0, to pi = closed~%float32 distal_approx   # radians, measured from all open = 0, to roughly pi = against proximal pad, relative to prox link~%bool[9] contact			# binary, 0 = proximal, 8 = fingertip~%float32[9] pressure		# scalar, dimensionless units, 0 = proximal, 8 = fingertip (can go negative)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Finger>))
  (cl:+ 0
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'contact) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pressure) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Finger>))
  "Converts a ROS message object to a list"
  (cl:list 'Finger
    (cl:cons ':proximal (proximal msg))
    (cl:cons ':distal_approx (distal_approx msg))
    (cl:cons ':contact (contact msg))
    (cl:cons ':pressure (pressure msg))
))
