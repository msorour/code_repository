;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs2::Finger)
  (if (not (find-package "REFLEX_MSGS2"))
    (make-package "REFLEX_MSGS2"))
  (shadow 'Finger (find-package "REFLEX_MSGS2")))
(unless (find-package "REFLEX_MSGS2::FINGER")
  (make-package "REFLEX_MSGS2::FINGER"))

(in-package "ROS")
;;//! \htmlinclude Finger.msg.html


(defclass reflex_msgs2::Finger
  :super ros::object
  :slots (_proximal _distal_approx _contact _pressure _imu ))

(defmethod reflex_msgs2::Finger
  (:init
   (&key
    ((:proximal __proximal) 0.0)
    ((:distal_approx __distal_approx) 0.0)
    ((:contact __contact) (let (r) (dotimes (i 14) (push nil r)) r))
    ((:pressure __pressure) (make-array 14 :initial-element 0.0 :element-type :float))
    ((:imu __imu) (instance reflex_msgs2::Imu :init))
    )
   (send-super :init)
   (setq _proximal (float __proximal))
   (setq _distal_approx (float __distal_approx))
   (setq _contact __contact)
   (setq _pressure __pressure)
   (setq _imu __imu)
   self)
  (:proximal
   (&optional __proximal)
   (if __proximal (setq _proximal __proximal)) _proximal)
  (:distal_approx
   (&optional __distal_approx)
   (if __distal_approx (setq _distal_approx __distal_approx)) _distal_approx)
  (:contact
   (&optional __contact)
   (if __contact (setq _contact __contact)) _contact)
  (:pressure
   (&optional __pressure)
   (if __pressure (setq _pressure __pressure)) _pressure)
  (:imu
   (&rest __imu)
   (if (keywordp (car __imu))
       (send* _imu __imu)
     (progn
       (if __imu (setq _imu (car __imu)))
       _imu)))
  (:serialization-length
   ()
   (+
    ;; float32 _proximal
    4
    ;; float32 _distal_approx
    4
    ;; bool[14] _contact
    (* 1    14)
    ;; float32[14] _pressure
    (* 4    14)
    ;; reflex_msgs2/Imu _imu
    (send _imu :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _proximal
       (sys::poke _proximal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _distal_approx
       (sys::poke _distal_approx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool[14] _contact
     (dotimes (i 14)
       (if (elt _contact i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; float32[14] _pressure
     (dotimes (i 14)
       (sys::poke (elt _pressure i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; reflex_msgs2/Imu _imu
       (send _imu :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _proximal
     (setq _proximal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _distal_approx
     (setq _distal_approx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool[14] _contact
   (dotimes (i (length _contact))
     (setf (elt _contact i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; float32[14] _pressure
   (dotimes (i (length _pressure))
     (setf (elt _pressure i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; reflex_msgs2/Imu _imu
     (send _imu :deserialize buf ptr-) (incf ptr- (send _imu :serialization-length))
   ;;
   self)
  )

(setf (get reflex_msgs2::Finger :md5sum-) "dd9fd34e802991303a3664f122843051")
(setf (get reflex_msgs2::Finger :datatype-) "reflex_msgs2/Finger")
(setf (get reflex_msgs2::Finger :definition-)
      "# message for ReFlex Fingers
float32 proximal		# radians, measured from all open = 0, to pi = closed
float32 distal_approx   # radians, measured from all open = 0, to roughly pi = against proximal pad, relative to prox link

bool[14] contact			# binary, 0 = proximal, 8 = fingertip
float32[14] pressure		# scalar, dimensionless units, 0 = proximal, 8 = fingertip (can go negative)
Imu imu
================================================================================
MSG: reflex_msgs2/Imu
# quaternion reading from IMU (w, x, y, z)
float32[4] quat	
float32[3] euler_angles 

# these are defined in reflex_hand.h driver
uint8 calibration_status 
uint16[11] calibration_data

")



(provide :reflex_msgs2/Finger "dd9fd34e802991303a3664f122843051")


