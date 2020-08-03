;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs2::SetSpeed)
  (if (not (find-package "REFLEX_MSGS2"))
    (make-package "REFLEX_MSGS2"))
  (shadow 'SetSpeed (find-package "REFLEX_MSGS2")))
(unless (find-package "REFLEX_MSGS2::SETSPEED")
  (make-package "REFLEX_MSGS2::SETSPEED"))
(unless (find-package "REFLEX_MSGS2::SETSPEEDREQUEST")
  (make-package "REFLEX_MSGS2::SETSPEEDREQUEST"))
(unless (find-package "REFLEX_MSGS2::SETSPEEDRESPONSE")
  (make-package "REFLEX_MSGS2::SETSPEEDRESPONSE"))

(in-package "ROS")





(defclass reflex_msgs2::SetSpeedRequest
  :super ros::object
  :slots (_motor ))

(defmethod reflex_msgs2::SetSpeedRequest
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

(defclass reflex_msgs2::SetSpeedResponse
  :super ros::object
  :slots ())

(defmethod reflex_msgs2::SetSpeedResponse
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

(defclass reflex_msgs2::SetSpeed
  :super ros::object
  :slots ())

(setf (get reflex_msgs2::SetSpeed :md5sum-) "7e4d6fd69113a7a3ab5caa755082c6b7")
(setf (get reflex_msgs2::SetSpeed :datatype-) "reflex_msgs2/SetSpeed")
(setf (get reflex_msgs2::SetSpeed :request) reflex_msgs2::SetSpeedRequest)
(setf (get reflex_msgs2::SetSpeed :response) reflex_msgs2::SetSpeedResponse)

(defmethod reflex_msgs2::SetSpeedRequest
  (:response () (instance reflex_msgs2::SetSpeedResponse :init)))

(setf (get reflex_msgs2::SetSpeedRequest :md5sum-) "7e4d6fd69113a7a3ab5caa755082c6b7")
(setf (get reflex_msgs2::SetSpeedRequest :datatype-) "reflex_msgs2/SetSpeedRequest")
(setf (get reflex_msgs2::SetSpeedRequest :definition-)
      "# message for calling hand commands
float64[4] motor
---

")

(setf (get reflex_msgs2::SetSpeedResponse :md5sum-) "7e4d6fd69113a7a3ab5caa755082c6b7")
(setf (get reflex_msgs2::SetSpeedResponse :datatype-) "reflex_msgs2/SetSpeedResponse")
(setf (get reflex_msgs2::SetSpeedResponse :definition-)
      "# message for calling hand commands
float64[4] motor
---

")



(provide :reflex_msgs2/SetSpeed "7e4d6fd69113a7a3ab5caa755082c6b7")


