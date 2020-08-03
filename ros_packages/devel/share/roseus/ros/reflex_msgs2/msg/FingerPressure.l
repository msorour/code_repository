;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs2::FingerPressure)
  (if (not (find-package "REFLEX_MSGS2"))
    (make-package "REFLEX_MSGS2"))
  (shadow 'FingerPressure (find-package "REFLEX_MSGS2")))
(unless (find-package "REFLEX_MSGS2::FINGERPRESSURE")
  (make-package "REFLEX_MSGS2::FINGERPRESSURE"))

(in-package "ROS")
;;//! \htmlinclude FingerPressure.msg.html


(defclass reflex_msgs2::FingerPressure
  :super ros::object
  :slots (_sensor ))

(defmethod reflex_msgs2::FingerPressure
  (:init
   (&key
    ((:sensor __sensor) (make-array 14 :initial-element 0 :element-type :integer))
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
    ;; uint16[14] _sensor
    (* 2    14)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16[14] _sensor
     (dotimes (i 14)
       (write-word (elt _sensor i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16[14] _sensor
   (dotimes (i (length _sensor))
     (setf (elt _sensor i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get reflex_msgs2::FingerPressure :md5sum-) "2fecdd6322f86c468a82244a96936129")
(setf (get reflex_msgs2::FingerPressure :datatype-) "reflex_msgs2/FingerPressure")
(setf (get reflex_msgs2::FingerPressure :definition-)
      "# message for pressure on a single finger
uint16[14] sensor    	# The sensors enumerate from the base of the finger to the tip
						# There are 5 on the proximal link, 4 on the distal

")



(provide :reflex_msgs2/FingerPressure "2fecdd6322f86c468a82244a96936129")


