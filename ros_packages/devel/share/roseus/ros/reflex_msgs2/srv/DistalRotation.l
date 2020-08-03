;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs2::DistalRotation)
  (if (not (find-package "REFLEX_MSGS2"))
    (make-package "REFLEX_MSGS2"))
  (shadow 'DistalRotation (find-package "REFLEX_MSGS2")))
(unless (find-package "REFLEX_MSGS2::DISTALROTATION")
  (make-package "REFLEX_MSGS2::DISTALROTATION"))
(unless (find-package "REFLEX_MSGS2::DISTALROTATIONREQUEST")
  (make-package "REFLEX_MSGS2::DISTALROTATIONREQUEST"))
(unless (find-package "REFLEX_MSGS2::DISTALROTATIONRESPONSE")
  (make-package "REFLEX_MSGS2::DISTALROTATIONRESPONSE"))

(in-package "ROS")





(defclass reflex_msgs2::DistalRotationRequest
  :super ros::object
  :slots (_palm_imu_quat _joint_angle _proximal _finger_imu_quat ))

(defmethod reflex_msgs2::DistalRotationRequest
  (:init
   (&key
    ((:palm_imu_quat __palm_imu_quat) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:joint_angle __joint_angle) 0.0)
    ((:proximal __proximal) 0.0)
    ((:finger_imu_quat __finger_imu_quat) (make-array 4 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _palm_imu_quat __palm_imu_quat)
   (setq _joint_angle (float __joint_angle))
   (setq _proximal (float __proximal))
   (setq _finger_imu_quat __finger_imu_quat)
   self)
  (:palm_imu_quat
   (&optional __palm_imu_quat)
   (if __palm_imu_quat (setq _palm_imu_quat __palm_imu_quat)) _palm_imu_quat)
  (:joint_angle
   (&optional __joint_angle)
   (if __joint_angle (setq _joint_angle __joint_angle)) _joint_angle)
  (:proximal
   (&optional __proximal)
   (if __proximal (setq _proximal __proximal)) _proximal)
  (:finger_imu_quat
   (&optional __finger_imu_quat)
   (if __finger_imu_quat (setq _finger_imu_quat __finger_imu_quat)) _finger_imu_quat)
  (:serialization-length
   ()
   (+
    ;; float32[4] _palm_imu_quat
    (* 4    4)
    ;; float32 _joint_angle
    4
    ;; float32 _proximal
    4
    ;; float32[4] _finger_imu_quat
    (* 4    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[4] _palm_imu_quat
     (dotimes (i 4)
       (sys::poke (elt _palm_imu_quat i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _joint_angle
       (sys::poke _joint_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _proximal
       (sys::poke _proximal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[4] _finger_imu_quat
     (dotimes (i 4)
       (sys::poke (elt _finger_imu_quat i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[4] _palm_imu_quat
   (dotimes (i (length _palm_imu_quat))
     (setf (elt _palm_imu_quat i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32 _joint_angle
     (setq _joint_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _proximal
     (setq _proximal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[4] _finger_imu_quat
   (dotimes (i (length _finger_imu_quat))
     (setf (elt _finger_imu_quat i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(defclass reflex_msgs2::DistalRotationResponse
  :super ros::object
  :slots (_rotation ))

(defmethod reflex_msgs2::DistalRotationResponse
  (:init
   (&key
    ((:rotation __rotation) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _rotation __rotation)
   self)
  (:rotation
   (&optional __rotation)
   (if __rotation (setq _rotation __rotation)) _rotation)
  (:serialization-length
   ()
   (+
    ;; float32[3] _rotation
    (* 4    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[3] _rotation
     (dotimes (i 3)
       (sys::poke (elt _rotation i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[3] _rotation
   (dotimes (i (length _rotation))
     (setf (elt _rotation i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(defclass reflex_msgs2::DistalRotation
  :super ros::object
  :slots ())

(setf (get reflex_msgs2::DistalRotation :md5sum-) "96436694e12dc909600e6760ed1174cd")
(setf (get reflex_msgs2::DistalRotation :datatype-) "reflex_msgs2/DistalRotation")
(setf (get reflex_msgs2::DistalRotation :request) reflex_msgs2::DistalRotationRequest)
(setf (get reflex_msgs2::DistalRotation :response) reflex_msgs2::DistalRotationResponse)

(defmethod reflex_msgs2::DistalRotationRequest
  (:response () (instance reflex_msgs2::DistalRotationResponse :init)))

(setf (get reflex_msgs2::DistalRotationRequest :md5sum-) "96436694e12dc909600e6760ed1174cd")
(setf (get reflex_msgs2::DistalRotationRequest :datatype-) "reflex_msgs2/DistalRotationRequest")
(setf (get reflex_msgs2::DistalRotationRequest :definition-)
      "# message for reading IMU rotations
float32[4] palm_imu_quat
float32 joint_angle
float32 proximal
float32[4] finger_imu_quat
---
float32[3] rotation

")

(setf (get reflex_msgs2::DistalRotationResponse :md5sum-) "96436694e12dc909600e6760ed1174cd")
(setf (get reflex_msgs2::DistalRotationResponse :datatype-) "reflex_msgs2/DistalRotationResponse")
(setf (get reflex_msgs2::DistalRotationResponse :definition-)
      "# message for reading IMU rotations
float32[4] palm_imu_quat
float32 joint_angle
float32 proximal
float32[4] finger_imu_quat
---
float32[3] rotation

")



(provide :reflex_msgs2/DistalRotation "96436694e12dc909600e6760ed1174cd")


