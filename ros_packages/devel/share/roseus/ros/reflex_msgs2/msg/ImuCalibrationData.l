;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs2::ImuCalibrationData)
  (if (not (find-package "REFLEX_MSGS2"))
    (make-package "REFLEX_MSGS2"))
  (shadow 'ImuCalibrationData (find-package "REFLEX_MSGS2")))
(unless (find-package "REFLEX_MSGS2::IMUCALIBRATIONDATA")
  (make-package "REFLEX_MSGS2::IMUCALIBRATIONDATA"))

(in-package "ROS")
;;//! \htmlinclude ImuCalibrationData.msg.html


(defclass reflex_msgs2::ImuCalibrationData
  :super ros::object
  :slots (_acc_offset_f1 _acc_offset_f2 _acc_offset_f3 _acc_offset_palm _mag_offset_f1 _mag_offset_f2 _mag_offset_f3 _mag_offset_palm _gyr_offset_f1 _gyr_offset_f2 _gyr_offset_f3 _gyr_offset_palm _acc_radius _gyr_radius ))

(defmethod reflex_msgs2::ImuCalibrationData
  (:init
   (&key
    ((:acc_offset_f1 __acc_offset_f1) (make-array 3 :initial-element 0 :element-type :integer))
    ((:acc_offset_f2 __acc_offset_f2) (make-array 3 :initial-element 0 :element-type :integer))
    ((:acc_offset_f3 __acc_offset_f3) (make-array 3 :initial-element 0 :element-type :integer))
    ((:acc_offset_palm __acc_offset_palm) (make-array 3 :initial-element 0 :element-type :integer))
    ((:mag_offset_f1 __mag_offset_f1) (make-array 3 :initial-element 0 :element-type :integer))
    ((:mag_offset_f2 __mag_offset_f2) (make-array 3 :initial-element 0 :element-type :integer))
    ((:mag_offset_f3 __mag_offset_f3) (make-array 3 :initial-element 0 :element-type :integer))
    ((:mag_offset_palm __mag_offset_palm) (make-array 3 :initial-element 0 :element-type :integer))
    ((:gyr_offset_f1 __gyr_offset_f1) (make-array 3 :initial-element 0 :element-type :integer))
    ((:gyr_offset_f2 __gyr_offset_f2) (make-array 3 :initial-element 0 :element-type :integer))
    ((:gyr_offset_f3 __gyr_offset_f3) (make-array 3 :initial-element 0 :element-type :integer))
    ((:gyr_offset_palm __gyr_offset_palm) (make-array 3 :initial-element 0 :element-type :integer))
    ((:acc_radius __acc_radius) (make-array 4 :initial-element 0 :element-type :integer))
    ((:gyr_radius __gyr_radius) (make-array 4 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _acc_offset_f1 __acc_offset_f1)
   (setq _acc_offset_f2 __acc_offset_f2)
   (setq _acc_offset_f3 __acc_offset_f3)
   (setq _acc_offset_palm __acc_offset_palm)
   (setq _mag_offset_f1 __mag_offset_f1)
   (setq _mag_offset_f2 __mag_offset_f2)
   (setq _mag_offset_f3 __mag_offset_f3)
   (setq _mag_offset_palm __mag_offset_palm)
   (setq _gyr_offset_f1 __gyr_offset_f1)
   (setq _gyr_offset_f2 __gyr_offset_f2)
   (setq _gyr_offset_f3 __gyr_offset_f3)
   (setq _gyr_offset_palm __gyr_offset_palm)
   (setq _acc_radius __acc_radius)
   (setq _gyr_radius __gyr_radius)
   self)
  (:acc_offset_f1
   (&optional __acc_offset_f1)
   (if __acc_offset_f1 (setq _acc_offset_f1 __acc_offset_f1)) _acc_offset_f1)
  (:acc_offset_f2
   (&optional __acc_offset_f2)
   (if __acc_offset_f2 (setq _acc_offset_f2 __acc_offset_f2)) _acc_offset_f2)
  (:acc_offset_f3
   (&optional __acc_offset_f3)
   (if __acc_offset_f3 (setq _acc_offset_f3 __acc_offset_f3)) _acc_offset_f3)
  (:acc_offset_palm
   (&optional __acc_offset_palm)
   (if __acc_offset_palm (setq _acc_offset_palm __acc_offset_palm)) _acc_offset_palm)
  (:mag_offset_f1
   (&optional __mag_offset_f1)
   (if __mag_offset_f1 (setq _mag_offset_f1 __mag_offset_f1)) _mag_offset_f1)
  (:mag_offset_f2
   (&optional __mag_offset_f2)
   (if __mag_offset_f2 (setq _mag_offset_f2 __mag_offset_f2)) _mag_offset_f2)
  (:mag_offset_f3
   (&optional __mag_offset_f3)
   (if __mag_offset_f3 (setq _mag_offset_f3 __mag_offset_f3)) _mag_offset_f3)
  (:mag_offset_palm
   (&optional __mag_offset_palm)
   (if __mag_offset_palm (setq _mag_offset_palm __mag_offset_palm)) _mag_offset_palm)
  (:gyr_offset_f1
   (&optional __gyr_offset_f1)
   (if __gyr_offset_f1 (setq _gyr_offset_f1 __gyr_offset_f1)) _gyr_offset_f1)
  (:gyr_offset_f2
   (&optional __gyr_offset_f2)
   (if __gyr_offset_f2 (setq _gyr_offset_f2 __gyr_offset_f2)) _gyr_offset_f2)
  (:gyr_offset_f3
   (&optional __gyr_offset_f3)
   (if __gyr_offset_f3 (setq _gyr_offset_f3 __gyr_offset_f3)) _gyr_offset_f3)
  (:gyr_offset_palm
   (&optional __gyr_offset_palm)
   (if __gyr_offset_palm (setq _gyr_offset_palm __gyr_offset_palm)) _gyr_offset_palm)
  (:acc_radius
   (&optional __acc_radius)
   (if __acc_radius (setq _acc_radius __acc_radius)) _acc_radius)
  (:gyr_radius
   (&optional __gyr_radius)
   (if __gyr_radius (setq _gyr_radius __gyr_radius)) _gyr_radius)
  (:serialization-length
   ()
   (+
    ;; uint16[3] _acc_offset_f1
    (* 2    3)
    ;; uint16[3] _acc_offset_f2
    (* 2    3)
    ;; uint16[3] _acc_offset_f3
    (* 2    3)
    ;; uint16[3] _acc_offset_palm
    (* 2    3)
    ;; uint16[3] _mag_offset_f1
    (* 2    3)
    ;; uint16[3] _mag_offset_f2
    (* 2    3)
    ;; uint16[3] _mag_offset_f3
    (* 2    3)
    ;; uint16[3] _mag_offset_palm
    (* 2    3)
    ;; uint16[3] _gyr_offset_f1
    (* 2    3)
    ;; uint16[3] _gyr_offset_f2
    (* 2    3)
    ;; uint16[3] _gyr_offset_f3
    (* 2    3)
    ;; uint16[3] _gyr_offset_palm
    (* 2    3)
    ;; uint16[4] _acc_radius
    (* 2    4)
    ;; uint16[4] _gyr_radius
    (* 2    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16[3] _acc_offset_f1
     (dotimes (i 3)
       (write-word (elt _acc_offset_f1 i) s)
       )
     ;; uint16[3] _acc_offset_f2
     (dotimes (i 3)
       (write-word (elt _acc_offset_f2 i) s)
       )
     ;; uint16[3] _acc_offset_f3
     (dotimes (i 3)
       (write-word (elt _acc_offset_f3 i) s)
       )
     ;; uint16[3] _acc_offset_palm
     (dotimes (i 3)
       (write-word (elt _acc_offset_palm i) s)
       )
     ;; uint16[3] _mag_offset_f1
     (dotimes (i 3)
       (write-word (elt _mag_offset_f1 i) s)
       )
     ;; uint16[3] _mag_offset_f2
     (dotimes (i 3)
       (write-word (elt _mag_offset_f2 i) s)
       )
     ;; uint16[3] _mag_offset_f3
     (dotimes (i 3)
       (write-word (elt _mag_offset_f3 i) s)
       )
     ;; uint16[3] _mag_offset_palm
     (dotimes (i 3)
       (write-word (elt _mag_offset_palm i) s)
       )
     ;; uint16[3] _gyr_offset_f1
     (dotimes (i 3)
       (write-word (elt _gyr_offset_f1 i) s)
       )
     ;; uint16[3] _gyr_offset_f2
     (dotimes (i 3)
       (write-word (elt _gyr_offset_f2 i) s)
       )
     ;; uint16[3] _gyr_offset_f3
     (dotimes (i 3)
       (write-word (elt _gyr_offset_f3 i) s)
       )
     ;; uint16[3] _gyr_offset_palm
     (dotimes (i 3)
       (write-word (elt _gyr_offset_palm i) s)
       )
     ;; uint16[4] _acc_radius
     (dotimes (i 4)
       (write-word (elt _acc_radius i) s)
       )
     ;; uint16[4] _gyr_radius
     (dotimes (i 4)
       (write-word (elt _gyr_radius i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16[3] _acc_offset_f1
   (dotimes (i (length _acc_offset_f1))
     (setf (elt _acc_offset_f1 i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _acc_offset_f2
   (dotimes (i (length _acc_offset_f2))
     (setf (elt _acc_offset_f2 i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _acc_offset_f3
   (dotimes (i (length _acc_offset_f3))
     (setf (elt _acc_offset_f3 i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _acc_offset_palm
   (dotimes (i (length _acc_offset_palm))
     (setf (elt _acc_offset_palm i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _mag_offset_f1
   (dotimes (i (length _mag_offset_f1))
     (setf (elt _mag_offset_f1 i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _mag_offset_f2
   (dotimes (i (length _mag_offset_f2))
     (setf (elt _mag_offset_f2 i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _mag_offset_f3
   (dotimes (i (length _mag_offset_f3))
     (setf (elt _mag_offset_f3 i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _mag_offset_palm
   (dotimes (i (length _mag_offset_palm))
     (setf (elt _mag_offset_palm i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _gyr_offset_f1
   (dotimes (i (length _gyr_offset_f1))
     (setf (elt _gyr_offset_f1 i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _gyr_offset_f2
   (dotimes (i (length _gyr_offset_f2))
     (setf (elt _gyr_offset_f2 i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _gyr_offset_f3
   (dotimes (i (length _gyr_offset_f3))
     (setf (elt _gyr_offset_f3 i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _gyr_offset_palm
   (dotimes (i (length _gyr_offset_palm))
     (setf (elt _gyr_offset_palm i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[4] _acc_radius
   (dotimes (i (length _acc_radius))
     (setf (elt _acc_radius i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[4] _gyr_radius
   (dotimes (i (length _gyr_radius))
     (setf (elt _gyr_radius i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get reflex_msgs2::ImuCalibrationData :md5sum-) "1ef3e1b102a68813a645fa51b970838b")
(setf (get reflex_msgs2::ImuCalibrationData :datatype-) "reflex_msgs2/ImuCalibrationData")
(setf (get reflex_msgs2::ImuCalibrationData :definition-)
      "# Acceleration offsets for each dimension in the format [x,y,z]
uint16[3] acc_offset_f1
uint16[3] acc_offset_f2
uint16[3] acc_offset_f3
uint16[3] acc_offset_palm

# Magnetometer offsets for each dimension in the format [x,y,z]
uint16[3] mag_offset_f1
uint16[3] mag_offset_f2
uint16[3] mag_offset_f3
uint16[3] mag_offset_palm

# Gyroscope offsets for each dimension in the format [x,y,z]
uint16[3] gyr_offset_f1
uint16[3] gyr_offset_f2
uint16[3] gyr_offset_f3
uint16[3] gyr_offset_palm

# Accelerometer and Gyroscope radius in the format [f1,f2,f3,palm]
uint16[4] acc_radius
uint16[4] gyr_radius
")



(provide :reflex_msgs2/ImuCalibrationData "1ef3e1b102a68813a645fa51b970838b")


