;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs2::Motor)
  (if (not (find-package "REFLEX_MSGS2"))
    (make-package "REFLEX_MSGS2"))
  (shadow 'Motor (find-package "REFLEX_MSGS2")))
(unless (find-package "REFLEX_MSGS2::MOTOR")
  (make-package "REFLEX_MSGS2::MOTOR"))

(in-package "ROS")
;;//! \htmlinclude Motor.msg.html


(defclass reflex_msgs2::Motor
  :super ros::object
  :slots (_joint_angle _raw_angle _velocity _load _voltage _temperature _error_state ))

(defmethod reflex_msgs2::Motor
  (:init
   (&key
    ((:joint_angle __joint_angle) 0.0)
    ((:raw_angle __raw_angle) 0.0)
    ((:velocity __velocity) 0.0)
    ((:load __load) 0.0)
    ((:voltage __voltage) 0.0)
    ((:temperature __temperature) 0)
    ((:error_state __error_state) "")
    )
   (send-super :init)
   (setq _joint_angle (float __joint_angle))
   (setq _raw_angle (float __raw_angle))
   (setq _velocity (float __velocity))
   (setq _load (float __load))
   (setq _voltage (float __voltage))
   (setq _temperature (round __temperature))
   (setq _error_state (string __error_state))
   self)
  (:joint_angle
   (&optional __joint_angle)
   (if __joint_angle (setq _joint_angle __joint_angle)) _joint_angle)
  (:raw_angle
   (&optional __raw_angle)
   (if __raw_angle (setq _raw_angle __raw_angle)) _raw_angle)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:load
   (&optional __load)
   (if __load (setq _load __load)) _load)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:error_state
   (&optional __error_state)
   (if __error_state (setq _error_state __error_state)) _error_state)
  (:serialization-length
   ()
   (+
    ;; float64 _joint_angle
    8
    ;; float64 _raw_angle
    8
    ;; float64 _velocity
    8
    ;; float64 _load
    8
    ;; float64 _voltage
    8
    ;; int32 _temperature
    4
    ;; string _error_state
    4 (length _error_state)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _joint_angle
       (sys::poke _joint_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _raw_angle
       (sys::poke _raw_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _load
       (sys::poke _load (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _temperature
       (write-long _temperature s)
     ;; string _error_state
       (write-long (length _error_state) s) (princ _error_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _joint_angle
     (setq _joint_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _raw_angle
     (setq _raw_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _velocity
     (setq _velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _load
     (setq _load (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _voltage
     (setq _voltage (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _temperature
     (setq _temperature (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _error_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _error_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get reflex_msgs2::Motor :md5sum-) "66d6779b4fae4b7b68e0863263c3993c")
(setf (get reflex_msgs2::Motor :datatype-) "reflex_msgs2/Motor")
(setf (get reflex_msgs2::Motor :definition-)
      "float64 joint_angle
float64 raw_angle
float64 velocity
float64 load
float64 voltage
int32 temperature
string error_state

")



(provide :reflex_msgs2/Motor "66d6779b4fae4b7b68e0863263c3993c")


