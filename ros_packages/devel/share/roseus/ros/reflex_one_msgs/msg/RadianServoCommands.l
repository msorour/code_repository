;; Auto-generated. Do not edit!


(when (boundp 'reflex_one_msgs::RadianServoCommands)
  (if (not (find-package "REFLEX_ONE_MSGS"))
    (make-package "REFLEX_ONE_MSGS"))
  (shadow 'RadianServoCommands (find-package "REFLEX_ONE_MSGS")))
(unless (find-package "REFLEX_ONE_MSGS::RADIANSERVOCOMMANDS")
  (make-package "REFLEX_ONE_MSGS::RADIANSERVOCOMMANDS"))

(in-package "ROS")
;;//! \htmlinclude RadianServoCommands.msg.html


(defclass reflex_one_msgs::RadianServoCommands
  :super ros::object
  :slots (_radian_commands ))

(defmethod reflex_one_msgs::RadianServoCommands
  (:init
   (&key
    ((:radian_commands __radian_commands) (make-array 5 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _radian_commands __radian_commands)
   self)
  (:radian_commands
   (&optional __radian_commands)
   (if __radian_commands (setq _radian_commands __radian_commands)) _radian_commands)
  (:serialization-length
   ()
   (+
    ;; float32[5] _radian_commands
    (* 4    5)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[5] _radian_commands
     (dotimes (i 5)
       (sys::poke (elt _radian_commands i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[5] _radian_commands
   (dotimes (i (length _radian_commands))
     (setf (elt _radian_commands i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get reflex_one_msgs::RadianServoCommands :md5sum-) "ddb4def5315f885ff778f6a32c156e0f")
(setf (get reflex_one_msgs::RadianServoCommands :datatype-) "reflex_one_msgs/RadianServoCommands")
(setf (get reflex_one_msgs::RadianServoCommands :definition-)
      "# Sets either radian position or radian/s velocity, depending on control mode
float32[5] radian_commands

")



(provide :reflex_one_msgs/RadianServoCommands "ddb4def5315f885ff778f6a32c156e0f")


