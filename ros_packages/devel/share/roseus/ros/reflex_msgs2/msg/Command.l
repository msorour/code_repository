;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs2::Command)
  (if (not (find-package "REFLEX_MSGS2"))
    (make-package "REFLEX_MSGS2"))
  (shadow 'Command (find-package "REFLEX_MSGS2")))
(unless (find-package "REFLEX_MSGS2::COMMAND")
  (make-package "REFLEX_MSGS2::COMMAND"))

(in-package "ROS")
;;//! \htmlinclude Command.msg.html


(defclass reflex_msgs2::Command
  :super ros::object
  :slots (_pose _velocity ))

(defmethod reflex_msgs2::Command
  (:init
   (&key
    ((:pose __pose) (instance reflex_msgs2::PoseCommand :init))
    ((:velocity __velocity) (instance reflex_msgs2::VelocityCommand :init))
    )
   (send-super :init)
   (setq _pose __pose)
   (setq _velocity __velocity)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:serialization-length
   ()
   (+
    ;; reflex_msgs2/PoseCommand _pose
    (send _pose :serialization-length)
    ;; reflex_msgs2/VelocityCommand _velocity
    (send _velocity :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; reflex_msgs2/PoseCommand _pose
       (send _pose :serialize s)
     ;; reflex_msgs2/VelocityCommand _velocity
       (send _velocity :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; reflex_msgs2/PoseCommand _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; reflex_msgs2/VelocityCommand _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;;
   self)
  )

(setf (get reflex_msgs2::Command :md5sum-) "bcad31578e17a6697c2483ccda6d52eb")
(setf (get reflex_msgs2::Command :datatype-) "reflex_msgs2/Command")
(setf (get reflex_msgs2::Command :definition-)
      "PoseCommand pose
VelocityCommand velocity

================================================================================
MSG: reflex_msgs2/PoseCommand
# Position in radians of various motors
float64 f1
float64 f2
float64 f3
float64 preshape

================================================================================
MSG: reflex_msgs2/VelocityCommand
# Velocity in radians/second of various motors
float64 f1
float64 f2
float64 f3
float64 preshape

")



(provide :reflex_msgs2/Command "bcad31578e17a6697c2483ccda6d52eb")


