;; Auto-generated. Do not edit!


(when (boundp 'reflex_msgs::Finger)
  (if (not (find-package "REFLEX_MSGS"))
    (make-package "REFLEX_MSGS"))
  (shadow 'Finger (find-package "REFLEX_MSGS")))
(unless (find-package "REFLEX_MSGS::FINGER")
  (make-package "REFLEX_MSGS::FINGER"))

(in-package "ROS")
;;//! \htmlinclude Finger.msg.html


(defclass reflex_msgs::Finger
  :super ros::object
  :slots (_proximal _distal_approx _contact _pressure ))

(defmethod reflex_msgs::Finger
  (:init
   (&key
    ((:proximal __proximal) 0.0)
    ((:distal_approx __distal_approx) 0.0)
    ((:contact __contact) (let (r) (dotimes (i 9) (push nil r)) r))
    ((:pressure __pressure) (make-array 9 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _proximal (float __proximal))
   (setq _distal_approx (float __distal_approx))
   (setq _contact __contact)
   (setq _pressure __pressure)
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
  (:serialization-length
   ()
   (+
    ;; float32 _proximal
    4
    ;; float32 _distal_approx
    4
    ;; bool[9] _contact
    (* 1    9)
    ;; float32[9] _pressure
    (* 4    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _proximal
       (sys::poke _proximal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _distal_approx
       (sys::poke _distal_approx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool[9] _contact
     (dotimes (i 9)
       (if (elt _contact i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; float32[9] _pressure
     (dotimes (i 9)
       (sys::poke (elt _pressure i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _proximal
     (setq _proximal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _distal_approx
     (setq _distal_approx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool[9] _contact
   (dotimes (i (length _contact))
     (setf (elt _contact i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; float32[9] _pressure
   (dotimes (i (length _pressure))
     (setf (elt _pressure i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get reflex_msgs::Finger :md5sum-) "b5232f74e901b48063f64cfc32aefbe0")
(setf (get reflex_msgs::Finger :datatype-) "reflex_msgs/Finger")
(setf (get reflex_msgs::Finger :definition-)
      "# message for ReFlex Fingers
float32 proximal		# radians, measured from all open = 0, to pi = closed
float32 distal_approx   # radians, measured from all open = 0, to roughly pi = against proximal pad, relative to prox link
bool[9] contact			# binary, 0 = proximal, 8 = fingertip
float32[9] pressure		# scalar, dimensionless units, 0 = proximal, 8 = fingertip (can go negative)

")



(provide :reflex_msgs/Finger "b5232f74e901b48063f64cfc32aefbe0")


