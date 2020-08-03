;; Auto-generated. Do not edit!


(when (boundp 'reflex_one_msgs::Command)
  (if (not (find-package "REFLEX_ONE_MSGS"))
    (make-package "REFLEX_ONE_MSGS"))
  (shadow 'Command (find-package "REFLEX_ONE_MSGS")))
(unless (find-package "REFLEX_ONE_MSGS::COMMAND")
  (make-package "REFLEX_ONE_MSGS::COMMAND"))

(in-package "ROS")
;;//! \htmlinclude Command.msg.html


(defclass reflex_one_msgs::Command
  :super ros::object
  :slots (_pose _velocity ))

(defmethod reflex_one_msgs::Command
  (:init
   (&key
    ((:pose __pose) (instance reflex_one_msgs::PoseCommand :init))
    ((:velocity __velocity) (instance reflex_one_msgs::VelocityCommand :init))
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
    ;; reflex_one_msgs/PoseCommand _pose
    (send _pose :serialization-length)
    ;; reflex_one_msgs/VelocityCommand _velocity
    (send _velocity :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; reflex_one_msgs/PoseCommand _pose
       (send _pose :serialize s)
     ;; reflex_one_msgs/VelocityCommand _velocity
       (send _velocity :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; reflex_one_msgs/PoseCommand _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; reflex_one_msgs/VelocityCommand _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;;
   self)
  )

(setf (get reflex_one_msgs::Command :md5sum-) "cdbebc0876ac872616d6c63603a84a2e")
(setf (get reflex_one_msgs::Command :datatype-) "reflex_one_msgs/Command")
(setf (get reflex_one_msgs::Command :definition-)
      "PoseCommand pose
VelocityCommand velocity

================================================================================
MSG: reflex_one_msgs/PoseCommand
# Position in radians of various motors
float64 f1
float64 f2
float64 f3
float64 preshape1
float64 preshape2

================================================================================
MSG: reflex_one_msgs/VelocityCommand
# Velocity in radians/second of various motors
float64 f1
float64 f2
float64 f3
float64 preshape1
float64 preshape2

")



(provide :reflex_one_msgs/Command "cdbebc0876ac872616d6c63603a84a2e")


