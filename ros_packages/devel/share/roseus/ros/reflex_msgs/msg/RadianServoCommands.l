;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs::RadianServoCommands)
  (if (not (find-package "REFLEX_MSGS"))
    (make-package "REFLEX_MSGS"))
  (shadow 'RadianServoCommands (find-package "REFLEX_MSGS")))
(unless (find-package "REFLEX_MSGS::RADIANSERVOCOMMANDS")
  (make-package "REFLEX_MSGS::RADIANSERVOCOMMANDS"))

(in-package "ROS")
;;//! \htmlinclude RadianServoCommands.msg.html


(defclass reflex_msgs::RadianServoCommands
  :super ros::object
  :slots (_radian_commands ))

(defmethod reflex_msgs::RadianServoCommands
  (:init
   (&key
    ((:radian_commands __radian_commands) (make-array 4 :initial-element 0.0 :element-type :float))
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
    ;; float32[4] _radian_commands
    (* 4    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[4] _radian_commands
     (dotimes (i 4)
       (sys::poke (elt _radian_commands i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[4] _radian_commands
   (dotimes (i (length _radian_commands))
     (setf (elt _radian_commands i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get reflex_msgs::RadianServoCommands :md5sum-) "ba1e88d9da1745cdc1900895d8c434b9")
(setf (get reflex_msgs::RadianServoCommands :datatype-) "reflex_msgs/RadianServoCommands")
(setf (get reflex_msgs::RadianServoCommands :definition-)
      "# Sets either radian position or radian/s velocity, depending on control mode
float32[4] radian_commands
")



(provide :reflex_msgs/RadianServoCommands "ba1e88d9da1745cdc1900895d8c434b9")


