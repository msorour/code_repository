;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs::SetSpeed)
  (if (not (find-package "REFLEX_MSGS"))
    (make-package "REFLEX_MSGS"))
  (shadow 'SetSpeed (find-package "REFLEX_MSGS")))
(unless (find-package "REFLEX_MSGS::SETSPEED")
  (make-package "REFLEX_MSGS::SETSPEED"))
(unless (find-package "REFLEX_MSGS::SETSPEEDREQUEST")
  (make-package "REFLEX_MSGS::SETSPEEDREQUEST"))
(unless (find-package "REFLEX_MSGS::SETSPEEDRESPONSE")
  (make-package "REFLEX_MSGS::SETSPEEDRESPONSE"))

(in-package "ROS")





(defclass reflex_msgs::SetSpeedRequest
  :super ros::object
  :slots (_motor ))

(defmethod reflex_msgs::SetSpeedRequest
  (:init
   (&key
    ((:motor __motor) (make-array 4 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _motor __motor)
   self)
  (:motor
   (&optional __motor)
   (if __motor (setq _motor __motor)) _motor)
  (:serialization-length
   ()
   (+
    ;; float64[4] _motor
    (* 8    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[4] _motor
     (dotimes (i 4)
       (sys::poke (elt _motor i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[4] _motor
   (dotimes (i (length _motor))
     (setf (elt _motor i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass reflex_msgs::SetSpeedResponse
  :super ros::object
  :slots ())

(defmethod reflex_msgs::SetSpeedResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass reflex_msgs::SetSpeed
  :super ros::object
  :slots ())

(setf (get reflex_msgs::SetSpeed :md5sum-) "7e4d6fd69113a7a3ab5caa755082c6b7")
(setf (get reflex_msgs::SetSpeed :datatype-) "reflex_msgs/SetSpeed")
(setf (get reflex_msgs::SetSpeed :request) reflex_msgs::SetSpeedRequest)
(setf (get reflex_msgs::SetSpeed :response) reflex_msgs::SetSpeedResponse)

(defmethod reflex_msgs::SetSpeedRequest
  (:response () (instance reflex_msgs::SetSpeedResponse :init)))

(setf (get reflex_msgs::SetSpeedRequest :md5sum-) "7e4d6fd69113a7a3ab5caa755082c6b7")
(setf (get reflex_msgs::SetSpeedRequest :datatype-) "reflex_msgs/SetSpeedRequest")
(setf (get reflex_msgs::SetSpeedRequest :definition-)
      "# message for calling hand commands
float64[4] motor
---

")

(setf (get reflex_msgs::SetSpeedResponse :md5sum-) "7e4d6fd69113a7a3ab5caa755082c6b7")
(setf (get reflex_msgs::SetSpeedResponse :datatype-) "reflex_msgs/SetSpeedResponse")
(setf (get reflex_msgs::SetSpeedResponse :definition-)
      "# message for calling hand commands
float64[4] motor
---

")



(provide :reflex_msgs/SetSpeed "7e4d6fd69113a7a3ab5caa755082c6b7")


