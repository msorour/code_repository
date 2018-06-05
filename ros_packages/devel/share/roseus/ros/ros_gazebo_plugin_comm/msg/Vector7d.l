;; Auto-generated. Do not edit!


(when (boundp 'ros_gazebo_plugin_comm::Vector7d)
  (if (not (find-package "ROS_GAZEBO_PLUGIN_COMM"))
    (make-package "ROS_GAZEBO_PLUGIN_COMM"))
  (shadow 'Vector7d (find-package "ROS_GAZEBO_PLUGIN_COMM")))
(unless (find-package "ROS_GAZEBO_PLUGIN_COMM::VECTOR7D")
  (make-package "ROS_GAZEBO_PLUGIN_COMM::VECTOR7D"))

(in-package "ROS")
;;//! \htmlinclude Vector7d.msg.html


(defclass ros_gazebo_plugin_comm::Vector7d
  :super ros::object
  :slots (_data ))

(defmethod ros_gazebo_plugin_comm::Vector7d
  (:init
   (&key
    ((:data __data) (make-array 7 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float32[7] _data
    (* 4    7)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[7] _data
     (dotimes (i 7)
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[7] _data
   (dotimes (i (length _data))
     (setf (elt _data i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get ros_gazebo_plugin_comm::Vector7d :md5sum-) "e4844fe6885794a5295df1d81cbae79f")
(setf (get ros_gazebo_plugin_comm::Vector7d :datatype-) "ros_gazebo_plugin_comm/Vector7d")
(setf (get ros_gazebo_plugin_comm::Vector7d :definition-)
      "float32[7] data

")



(provide :ros_gazebo_plugin_comm/Vector7d "e4844fe6885794a5295df1d81cbae79f")


