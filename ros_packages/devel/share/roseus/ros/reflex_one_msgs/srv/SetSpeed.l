;; Auto-generated. Do not edit!


(when (boundp 'reflex_one_msgs::SetSpeed)
  (if (not (find-package "REFLEX_ONE_MSGS"))
    (make-package "REFLEX_ONE_MSGS"))
  (shadow 'SetSpeed (find-package "REFLEX_ONE_MSGS")))
(unless (find-package "REFLEX_ONE_MSGS::SETSPEED")
  (make-package "REFLEX_ONE_MSGS::SETSPEED"))
(unless (find-package "REFLEX_ONE_MSGS::SETSPEEDREQUEST")
  (make-package "REFLEX_ONE_MSGS::SETSPEEDREQUEST"))
(unless (find-package "REFLEX_ONE_MSGS::SETSPEEDRESPONSE")
  (make-package "REFLEX_ONE_MSGS::SETSPEEDRESPONSE"))

(in-package "ROS")





(defclass reflex_one_msgs::SetSpeedRequest
  :super ros::object
  :slots (_motor ))

(defmethod reflex_one_msgs::SetSpeedRequest
  (:init
   (&key
    ((:motor __motor) (make-array 5 :initial-element 0.0 :element-type :float))
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
    ;; float64[5] _motor
    (* 8    5)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[5] _motor
     (dotimes (i 5)
       (sys::poke (elt _motor i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[5] _motor
   (dotimes (i (length _motor))
     (setf (elt _motor i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass reflex_one_msgs::SetSpeedResponse
  :super ros::object
  :slots ())

(defmethod reflex_one_msgs::SetSpeedResponse
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

(defclass reflex_one_msgs::SetSpeed
  :super ros::object
  :slots ())

(setf (get reflex_one_msgs::SetSpeed :md5sum-) "04171ebc3d171407826cbe90dea5b330")
(setf (get reflex_one_msgs::SetSpeed :datatype-) "reflex_one_msgs/SetSpeed")
(setf (get reflex_one_msgs::SetSpeed :request) reflex_one_msgs::SetSpeedRequest)
(setf (get reflex_one_msgs::SetSpeed :response) reflex_one_msgs::SetSpeedResponse)

(defmethod reflex_one_msgs::SetSpeedRequest
  (:response () (instance reflex_one_msgs::SetSpeedResponse :init)))

(setf (get reflex_one_msgs::SetSpeedRequest :md5sum-) "04171ebc3d171407826cbe90dea5b330")
(setf (get reflex_one_msgs::SetSpeedRequest :datatype-) "reflex_one_msgs/SetSpeedRequest")
(setf (get reflex_one_msgs::SetSpeedRequest :definition-)
      "# message for calling hand commands
float64[5] motor
---

")

(setf (get reflex_one_msgs::SetSpeedResponse :md5sum-) "04171ebc3d171407826cbe90dea5b330")
(setf (get reflex_one_msgs::SetSpeedResponse :datatype-) "reflex_one_msgs/SetSpeedResponse")
(setf (get reflex_one_msgs::SetSpeedResponse :definition-)
      "# message for calling hand commands
float64[5] motor
---

")



(provide :reflex_one_msgs/SetSpeed "04171ebc3d171407826cbe90dea5b330")


