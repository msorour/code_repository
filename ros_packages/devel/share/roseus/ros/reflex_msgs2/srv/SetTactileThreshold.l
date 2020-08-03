;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs2::SetTactileThreshold)
  (if (not (find-package "REFLEX_MSGS2"))
    (make-package "REFLEX_MSGS2"))
  (shadow 'SetTactileThreshold (find-package "REFLEX_MSGS2")))
(unless (find-package "REFLEX_MSGS2::SETTACTILETHRESHOLD")
  (make-package "REFLEX_MSGS2::SETTACTILETHRESHOLD"))
(unless (find-package "REFLEX_MSGS2::SETTACTILETHRESHOLDREQUEST")
  (make-package "REFLEX_MSGS2::SETTACTILETHRESHOLDREQUEST"))
(unless (find-package "REFLEX_MSGS2::SETTACTILETHRESHOLDRESPONSE")
  (make-package "REFLEX_MSGS2::SETTACTILETHRESHOLDRESPONSE"))

(in-package "ROS")





(defclass reflex_msgs2::SetTactileThresholdRequest
  :super ros::object
  :slots (_finger ))

(defmethod reflex_msgs2::SetTactileThresholdRequest
  (:init
   (&key
    ((:finger __finger) (let (r) (dotimes (i 3) (push (instance reflex_msgs2::FingerPressure :init) r)) r))
    )
   (send-super :init)
   (setq _finger __finger)
   self)
  (:finger
   (&rest __finger)
   (if (keywordp (car __finger))
       (send* _finger __finger)
     (progn
       (if __finger (setq _finger (car __finger)))
       _finger)))
  (:serialization-length
   ()
   (+
    ;; reflex_msgs2/FingerPressure[3] _finger
    (apply #'+ (send-all _finger :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; reflex_msgs2/FingerPressure[3] _finger
     (dolist (elem _finger)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; reflex_msgs2/FingerPressure[3] _finger
   (dotimes (i 3)
     (send (elt _finger i) :deserialize buf ptr-) (incf ptr- (send (elt _finger i) :serialization-length))
     )
   ;;
   self)
  )

(defclass reflex_msgs2::SetTactileThresholdResponse
  :super ros::object
  :slots ())

(defmethod reflex_msgs2::SetTactileThresholdResponse
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

(defclass reflex_msgs2::SetTactileThreshold
  :super ros::object
  :slots ())

(setf (get reflex_msgs2::SetTactileThreshold :md5sum-) "b2d8992c5dc80dcc6a9e73ef557b5387")
(setf (get reflex_msgs2::SetTactileThreshold :datatype-) "reflex_msgs2/SetTactileThreshold")
(setf (get reflex_msgs2::SetTactileThreshold :request) reflex_msgs2::SetTactileThresholdRequest)
(setf (get reflex_msgs2::SetTactileThreshold :response) reflex_msgs2::SetTactileThresholdResponse)

(defmethod reflex_msgs2::SetTactileThresholdRequest
  (:response () (instance reflex_msgs2::SetTactileThresholdResponse :init)))

(setf (get reflex_msgs2::SetTactileThresholdRequest :md5sum-) "b2d8992c5dc80dcc6a9e73ef557b5387")
(setf (get reflex_msgs2::SetTactileThresholdRequest :datatype-) "reflex_msgs2/SetTactileThresholdRequest")
(setf (get reflex_msgs2::SetTactileThresholdRequest :definition-)
      "# message for calling setting pressure thresholds on various fingers
FingerPressure[3] finger

================================================================================
MSG: reflex_msgs2/FingerPressure
# message for pressure on a single finger
uint16[14] sensor    	# The sensors enumerate from the base of the finger to the tip
						# There are 5 on the proximal link, 4 on the distal
---

")

(setf (get reflex_msgs2::SetTactileThresholdResponse :md5sum-) "b2d8992c5dc80dcc6a9e73ef557b5387")
(setf (get reflex_msgs2::SetTactileThresholdResponse :datatype-) "reflex_msgs2/SetTactileThresholdResponse")
(setf (get reflex_msgs2::SetTactileThresholdResponse :definition-)
      "# message for calling setting pressure thresholds on various fingers
FingerPressure[3] finger

================================================================================
MSG: reflex_msgs2/FingerPressure
# message for pressure on a single finger
uint16[14] sensor    	# The sensors enumerate from the base of the finger to the tip
						# There are 5 on the proximal link, 4 on the distal
---

")



(provide :reflex_msgs2/SetTactileThreshold "b2d8992c5dc80dcc6a9e73ef557b5387")


