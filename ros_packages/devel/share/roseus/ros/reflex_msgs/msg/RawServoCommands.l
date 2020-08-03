;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs::RawServoCommands)
  (if (not (find-package "REFLEX_MSGS"))
    (make-package "REFLEX_MSGS"))
  (shadow 'RawServoCommands (find-package "REFLEX_MSGS")))
(unless (find-package "REFLEX_MSGS::RAWSERVOCOMMANDS")
  (make-package "REFLEX_MSGS::RAWSERVOCOMMANDS"))

(in-package "ROS")
;;//! \htmlinclude RawServoCommands.msg.html


(defclass reflex_msgs::RawServoCommands
  :super ros::object
  :slots (_raw_positions ))

(defmethod reflex_msgs::RawServoCommands
  (:init
   (&key
    ((:raw_positions __raw_positions) (make-array 4 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _raw_positions __raw_positions)
   self)
  (:raw_positions
   (&optional __raw_positions)
   (if __raw_positions (setq _raw_positions __raw_positions)) _raw_positions)
  (:serialization-length
   ()
   (+
    ;; uint16[4] _raw_positions
    (* 2    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16[4] _raw_positions
     (dotimes (i 4)
       (write-word (elt _raw_positions i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16[4] _raw_positions
   (dotimes (i (length _raw_positions))
     (setf (elt _raw_positions i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get reflex_msgs::RawServoCommands :md5sum-) "aea318ab5663e05395d3ab895fd67e1e")
(setf (get reflex_msgs::RawServoCommands :datatype-) "reflex_msgs/RawServoCommands")
(setf (get reflex_msgs::RawServoCommands :definition-)
      "uint16[4] raw_positions
")



(provide :reflex_msgs/RawServoCommands "aea318ab5663e05395d3ab895fd67e1e")


