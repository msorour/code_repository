;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs2::Imu)
  (if (not (find-package "REFLEX_MSGS2"))
    (make-package "REFLEX_MSGS2"))
  (shadow 'Imu (find-package "REFLEX_MSGS2")))
(unless (find-package "REFLEX_MSGS2::IMU")
  (make-package "REFLEX_MSGS2::IMU"))

(in-package "ROS")
;;//! \htmlinclude Imu.msg.html


(defclass reflex_msgs2::Imu
  :super ros::object
  :slots (_quat _euler_angles _calibration_status _calibration_data ))

(defmethod reflex_msgs2::Imu
  (:init
   (&key
    ((:quat __quat) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:euler_angles __euler_angles) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:calibration_status __calibration_status) 0)
    ((:calibration_data __calibration_data) (make-array 11 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _quat __quat)
   (setq _euler_angles __euler_angles)
   (setq _calibration_status (round __calibration_status))
   (setq _calibration_data __calibration_data)
   self)
  (:quat
   (&optional __quat)
   (if __quat (setq _quat __quat)) _quat)
  (:euler_angles
   (&optional __euler_angles)
   (if __euler_angles (setq _euler_angles __euler_angles)) _euler_angles)
  (:calibration_status
   (&optional __calibration_status)
   (if __calibration_status (setq _calibration_status __calibration_status)) _calibration_status)
  (:calibration_data
   (&optional __calibration_data)
   (if __calibration_data (setq _calibration_data __calibration_data)) _calibration_data)
  (:serialization-length
   ()
   (+
    ;; float32[4] _quat
    (* 4    4)
    ;; float32[3] _euler_angles
    (* 4    3)
    ;; uint8 _calibration_status
    1
    ;; uint16[11] _calibration_data
    (* 2    11)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[4] _quat
     (dotimes (i 4)
       (sys::poke (elt _quat i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _euler_angles
     (dotimes (i 3)
       (sys::poke (elt _euler_angles i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint8 _calibration_status
       (write-byte _calibration_status s)
     ;; uint16[11] _calibration_data
     (dotimes (i 11)
       (write-word (elt _calibration_data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[4] _quat
   (dotimes (i (length _quat))
     (setf (elt _quat i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _euler_angles
   (dotimes (i (length _euler_angles))
     (setf (elt _euler_angles i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; uint8 _calibration_status
     (setq _calibration_status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16[11] _calibration_data
   (dotimes (i (length _calibration_data))
     (setf (elt _calibration_data i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get reflex_msgs2::Imu :md5sum-) "dea5c53d0c934f48b1ee6e5a6eed4389")
(setf (get reflex_msgs2::Imu :datatype-) "reflex_msgs2/Imu")
(setf (get reflex_msgs2::Imu :definition-)
      "# quaternion reading from IMU (w, x, y, z)
float32[4] quat	
float32[3] euler_angles 

# these are defined in reflex_hand.h driver
uint8 calibration_status 
uint16[11] calibration_data

")



(provide :reflex_msgs2/Imu "dea5c53d0c934f48b1ee6e5a6eed4389")


