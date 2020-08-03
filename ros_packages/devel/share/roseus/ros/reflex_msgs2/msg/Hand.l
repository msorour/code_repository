;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs2::Hand)
  (if (not (find-package "REFLEX_MSGS2"))
    (make-package "REFLEX_MSGS2"))
  (shadow 'Hand (find-package "REFLEX_MSGS2")))
(unless (find-package "REFLEX_MSGS2::HAND")
  (make-package "REFLEX_MSGS2::HAND"))

(in-package "ROS")
;;//! \htmlinclude Hand.msg.html


(defclass reflex_msgs2::Hand
  :super ros::object
  :slots (_finger _motor _palmImu ))

(defmethod reflex_msgs2::Hand
  (:init
   (&key
    ((:finger __finger) (let (r) (dotimes (i 3) (push (instance reflex_msgs2::Finger :init) r)) r))
    ((:motor __motor) (let (r) (dotimes (i 4) (push (instance reflex_msgs2::Motor :init) r)) r))
    ((:palmImu __palmImu) (instance reflex_msgs2::Imu :init))
    )
   (send-super :init)
   (setq _finger __finger)
   (setq _motor __motor)
   (setq _palmImu __palmImu)
   self)
  (:finger
   (&rest __finger)
   (if (keywordp (car __finger))
       (send* _finger __finger)
     (progn
       (if __finger (setq _finger (car __finger)))
       _finger)))
  (:motor
   (&rest __motor)
   (if (keywordp (car __motor))
       (send* _motor __motor)
     (progn
       (if __motor (setq _motor (car __motor)))
       _motor)))
  (:palmImu
   (&rest __palmImu)
   (if (keywordp (car __palmImu))
       (send* _palmImu __palmImu)
     (progn
       (if __palmImu (setq _palmImu (car __palmImu)))
       _palmImu)))
  (:serialization-length
   ()
   (+
    ;; reflex_msgs2/Finger[3] _finger
    (apply #'+ (send-all _finger :serialization-length))
    ;; reflex_msgs2/Motor[4] _motor
    (apply #'+ (send-all _motor :serialization-length))
    ;; reflex_msgs2/Imu _palmImu
    (send _palmImu :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; reflex_msgs2/Finger[3] _finger
     (dolist (elem _finger)
       (send elem :serialize s)
       )
     ;; reflex_msgs2/Motor[4] _motor
     (dolist (elem _motor)
       (send elem :serialize s)
       )
     ;; reflex_msgs2/Imu _palmImu
       (send _palmImu :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; reflex_msgs2/Finger[3] _finger
   (dotimes (i 3)
     (send (elt _finger i) :deserialize buf ptr-) (incf ptr- (send (elt _finger i) :serialization-length))
     )
   ;; reflex_msgs2/Motor[4] _motor
   (dotimes (i 4)
     (send (elt _motor i) :deserialize buf ptr-) (incf ptr- (send (elt _motor i) :serialization-length))
     )
   ;; reflex_msgs2/Imu _palmImu
     (send _palmImu :deserialize buf ptr-) (incf ptr- (send _palmImu :serialization-length))
   ;;
   self)
  )

(setf (get reflex_msgs2::Hand :md5sum-) "d9d385c8f6333f652b7d358710209f43")
(setf (get reflex_msgs2::Hand :datatype-) "reflex_msgs2/Hand")
(setf (get reflex_msgs2::Hand :definition-)
      "# message for ReFlex Hand
Finger[3] finger       # Hold out your right hand palm up, with pointer finger, middle finger and thumb extended
                       # Pointer = finger[0], Middle = finger[1], Thumb = finger[2]
Motor[4] motor         # Finger 1, Finger 2, Finger 3, and Preshape
Imu palmImu
#ImuCalibrationData LANCE

#CHANGE IMU TO ARRAY!!!
================================================================================
MSG: reflex_msgs2/Finger
# message for ReFlex Fingers
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

================================================================================
MSG: reflex_msgs2/Motor
float64 joint_angle
float64 raw_angle
float64 velocity
float64 load
float64 voltage
int32 temperature
string error_state

")



(provide :reflex_msgs2/Hand "d9d385c8f6333f652b7d358710209f43")


