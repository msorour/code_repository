; Auto-generated. Do not edit!


(cl:in-package reflex_msgs2-msg)


;//! \htmlinclude PoseCommand.msg.html

(cl:defclass <PoseCommand> (roslisp-msg-protocol:ros-message)
  ((f1
    :reader f1
    :initarg :f1
    :type cl:float
    :initform 0.0)
   (f2
    :reader f2
    :initarg :f2
    :type cl:float
    :initform 0.0)
   (f3
    :reader f3
    :initarg :f3
    :type cl:float
    :initform 0.0)
   (preshape
    :reader preshape
    :initarg :preshape
    :type cl:float
    :initform 0.0))
)

(cl:defclass PoseCommand (<PoseCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PoseCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PoseCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reflex_msgs2-msg:<PoseCommand> is deprecated: use reflex_msgs2-msg:PoseCommand instead.")))

(cl:ensure-generic-function 'f1-val :lambda-list '(m))
(cl:defmethod f1-val ((m <PoseCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:f1-val is deprecated.  Use reflex_msgs2-msg:f1 instead.")
  (f1 m))

(cl:ensure-generic-function 'f2-val :lambda-list '(m))
(cl:defmethod f2-val ((m <PoseCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:f2-val is deprecated.  Use reflex_msgs2-msg:f2 instead.")
  (f2 m))

(cl:ensure-generic-function 'f3-val :lambda-list '(m))
(cl:defmethod f3-val ((m <PoseCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:f3-val is deprecated.  Use reflex_msgs2-msg:f3 instead.")
  (f3 m))

(cl:ensure-generic-function 'preshape-val :lambda-list '(m))
(cl:defmethod preshape-val ((m <PoseCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reflex_msgs2-msg:preshape-val is deprecated.  Use reflex_msgs2-msg:preshape instead.")
  (preshape m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PoseCommand>) ostream)
  "Serializes a message object of type '<PoseCommand>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'f1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'f2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'f3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'preshape))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PoseCommand>) istream)
  "Deserializes a message object of type '<PoseCommand>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'f1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'f2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'f3) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'preshape) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PoseCommand>)))
  "Returns string type for a message object of type '<PoseCommand>"
  "reflex_msgs2/PoseCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseCommand)))
  "Returns string type for a message object of type 'PoseCommand"
  "reflex_msgs2/PoseCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PoseCommand>)))
  "Returns md5sum for a message object of type '<PoseCommand>"
  "ec8e01f7c46594906539a78e3918a7c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PoseCommand)))
  "Returns md5sum for a message object of type 'PoseCommand"
  "ec8e01f7c46594906539a78e3918a7c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PoseCommand>)))
  "Returns full string definition for message of type '<PoseCommand>"
  (cl:format cl:nil "# Position in radians of various motors~%float64 f1~%float64 f2~%float64 f3~%float64 preshape~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PoseCommand)))
  "Returns full string definition for message of type 'PoseCommand"
  (cl:format cl:nil "# Position in radians of various motors~%float64 f1~%float64 f2~%float64 f3~%float64 preshape~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PoseCommand>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PoseCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'PoseCommand
    (cl:cons ':f1 (f1 msg))
    (cl:cons ':f2 (f2 msg))
    (cl:cons ':f3 (f3 msg))
    (cl:cons ':preshape (preshape msg))
))
