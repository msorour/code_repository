;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs::ForceCommand)
  (if (not (find-package "REFLEX_MSGS"))
    (make-package "REFLEX_MSGS"))
  (shadow 'ForceCommand (find-package "REFLEX_MSGS")))
(unless (find-package "REFLEX_MSGS::FORCECOMMAND")
  (make-package "REFLEX_MSGS::FORCECOMMAND"))

(in-package "ROS")
;;//! \htmlinclude ForceCommand.msg.html


(defclass reflex_msgs::ForceCommand
  :super ros::object
  :slots (_f1 _f2 _f3 _preshape ))

(defmethod reflex_msgs::ForceCommand
  (:init
   (&key
    ((:f1 __f1) 0.0)
    ((:f2 __f2) 0.0)
    ((:f3 __f3) 0.0)
    ((:preshape __preshape) 0.0)
    )
   (send-super :init)
   (setq _f1 (float __f1))
   (setq _f2 (float __f2))
   (setq _f3 (float __f3))
   (setq _preshape (float __preshape))
   self)
  (:f1
   (&optional __f1)
   (if __f1 (setq _f1 __f1)) _f1)
  (:f2
   (&optional __f2)
   (if __f2 (setq _f2 __f2)) _f2)
  (:f3
   (&optional __f3)
   (if __f3 (setq _f3 __f3)) _f3)
  (:preshape
   (&optional __preshape)
   (if __preshape (setq _preshape __preshape)) _preshape)
  (:serialization-length
   ()
   (+
    ;; float64 _f1
    8
    ;; float64 _f2
    8
    ;; float64 _f3
    8
    ;; float64 _preshape
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _f1
       (sys::poke _f1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _f2
       (sys::poke _f2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _f3
       (sys::poke _f3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _preshape
       (sys::poke _preshape (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _f1
     (setq _f1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _f2
     (setq _f2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _f3
     (setq _f3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _preshape
     (setq _preshape (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get reflex_msgs::ForceCommand :md5sum-) "ec8e01f7c46594906539a78e3918a7c2")
(setf (get reflex_msgs::ForceCommand :datatype-) "reflex_msgs/ForceCommand")
(setf (get reflex_msgs::ForceCommand :definition-)
      "# Torque of various motors. UNITLESS due to the way Dynamixels measure torque
# More information on the force feedback here:
#	http://support.robotis.com/en/product/dynamixel/mx_series/mx-28.htm#Actuator_Address_28
float64 f1
float64 f2
float64 f3
float64 preshape

")



(provide :reflex_msgs/ForceCommand "ec8e01f7c46594906539a78e3918a7c2")


