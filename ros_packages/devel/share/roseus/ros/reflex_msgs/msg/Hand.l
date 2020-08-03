;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs::Hand)
  (if (not (find-package "REFLEX_MSGS"))
    (make-package "REFLEX_MSGS"))
  (shadow 'Hand (find-package "REFLEX_MSGS")))
(unless (find-package "REFLEX_MSGS::HAND")
  (make-package "REFLEX_MSGS::HAND"))

(in-package "ROS")
;;//! \htmlinclude Hand.msg.html


(defclass reflex_msgs::Hand
  :super ros::object
  :slots (_finger _motor ))

(defmethod reflex_msgs::Hand
  (:init
   (&key
    ((:finger __finger) (let (r) (dotimes (i 3) (push (instance reflex_msgs::Finger :init) r)) r))
    ((:motor __motor) (let (r) (dotimes (i 4) (push (instance reflex_msgs::Motor :init) r)) r))
    )
   (send-super :init)
   (setq _finger __finger)
   (setq _motor __motor)
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
  (:serialization-length
   ()
   (+
    ;; reflex_msgs/Finger[3] _finger
    (apply #'+ (send-all _finger :serialization-length))
    ;; reflex_msgs/Motor[4] _motor
    (apply #'+ (send-all _motor :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; reflex_msgs/Finger[3] _finger
     (dolist (elem _finger)
       (send elem :serialize s)
       )
     ;; reflex_msgs/Motor[4] _motor
     (dolist (elem _motor)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; reflex_msgs/Finger[3] _finger
   (dotimes (i 3)
     (send (elt _finger i) :deserialize buf ptr-) (incf ptr- (send (elt _finger i) :serialization-length))
     )
   ;; reflex_msgs/Motor[4] _motor
   (dotimes (i 4)
     (send (elt _motor i) :deserialize buf ptr-) (incf ptr- (send (elt _motor i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get reflex_msgs::Hand :md5sum-) "5328a4ca4ff4a136db2555c3178a3e1b")
(setf (get reflex_msgs::Hand :datatype-) "reflex_msgs/Hand")
(setf (get reflex_msgs::Hand :definition-)
      "# message for ReFlex Hand
Finger[3] finger       # Hold out your right hand palm up, with pointer finger, middle finger and thumb extended
                       # Pointer = finger[0], Middle = finger[1], Thumb = finger[2]
Motor[4] motor         # Finger 1, Finger 2, Finger 3, and Preshape

================================================================================
MSG: reflex_msgs/Finger
# message for ReFlex Fingers
float32 proximal		# radians, measured from all open = 0, to pi = closed
float32 distal_approx   # radians, measured from all open = 0, to roughly pi = against proximal pad, relative to prox link
bool[9] contact			# binary, 0 = proximal, 8 = fingertip
float32[9] pressure		# scalar, dimensionless units, 0 = proximal, 8 = fingertip (can go negative)

================================================================================
MSG: reflex_msgs/Motor
float64 joint_angle
float64 raw_angle
float64 velocity
float64 load
float64 voltage
int32 temperature
string error_state

")



(provide :reflex_msgs/Hand "5328a4ca4ff4a136db2555c3178a3e1b")


