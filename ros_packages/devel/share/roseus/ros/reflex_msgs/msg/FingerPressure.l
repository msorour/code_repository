;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs::FingerPressure)
  (if (not (find-package "REFLEX_MSGS"))
    (make-package "REFLEX_MSGS"))
  (shadow 'FingerPressure (find-package "REFLEX_MSGS")))
(unless (find-package "REFLEX_MSGS::FINGERPRESSURE")
  (make-package "REFLEX_MSGS::FINGERPRESSURE"))

(in-package "ROS")
;;//! \htmlinclude FingerPressure.msg.html


(defclass reflex_msgs::FingerPressure
  :super ros::object
  :slots (_sensor ))

(defmethod reflex_msgs::FingerPressure
  (:init
   (&key
    ((:sensor __sensor) (make-array 9 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _sensor __sensor)
   self)
  (:sensor
   (&optional __sensor)
   (if __sensor (setq _sensor __sensor)) _sensor)
  (:serialization-length
   ()
   (+
    ;; uint16[9] _sensor
    (* 2    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16[9] _sensor
     (dotimes (i 9)
       (write-word (elt _sensor i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16[9] _sensor
   (dotimes (i (length _sensor))
     (setf (elt _sensor i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get reflex_msgs::FingerPressure :md5sum-) "fe4417140692b62d71a7ba82897b83f5")
(setf (get reflex_msgs::FingerPressure :datatype-) "reflex_msgs/FingerPressure")
(setf (get reflex_msgs::FingerPressure :definition-)
      "# message for pressure on a single finger
uint16[9] sensor    	# The sensors enumerate from the base of the finger to the tip
						# There are 5 on the proximal link, 4 on the distal

")



(provide :reflex_msgs/FingerPressure "fe4417140692b62d71a7ba82897b83f5")


