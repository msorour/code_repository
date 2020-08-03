;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_msgs::MotorState)
  (if (not (find-package "DYNAMIXEL_MSGS"))
    (make-package "DYNAMIXEL_MSGS"))
  (shadow 'MotorState (find-package "DYNAMIXEL_MSGS")))
(unless (find-package "DYNAMIXEL_MSGS::MOTORSTATE")
  (make-package "DYNAMIXEL_MSGS::MOTORSTATE"))

(in-package "ROS")
;;//! \htmlinclude MotorState.msg.html


(defclass dynamixel_msgs::MotorState
  :super ros::object
  :slots (_timestamp _id _goal _position _error _speed _load _voltage _temperature _moving ))

(defmethod dynamixel_msgs::MotorState
  (:init
   (&key
    ((:timestamp __timestamp) 0.0)
    ((:id __id) 0)
    ((:goal __goal) 0)
    ((:position __position) 0)
    ((:error __error) 0)
    ((:speed __speed) 0)
    ((:load __load) 0.0)
    ((:voltage __voltage) 0.0)
    ((:temperature __temperature) 0)
    ((:moving __moving) nil)
    )
   (send-super :init)
   (setq _timestamp (float __timestamp))
   (setq _id (round __id))
   (setq _goal (round __goal))
   (setq _position (round __position))
   (setq _error (round __error))
   (setq _speed (round __speed))
   (setq _load (float __load))
   (setq _voltage (float __voltage))
   (setq _temperature (round __temperature))
   (setq _moving __moving)
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:goal
   (&optional __goal)
   (if __goal (setq _goal __goal)) _goal)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:load
   (&optional __load)
   (if __load (setq _load __load)) _load)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:moving
   (&optional __moving)
   (if __moving (setq _moving __moving)) _moving)
  (:serialization-length
   ()
   (+
    ;; float64 _timestamp
    8
    ;; int32 _id
    4
    ;; int64 _goal
    8
    ;; int64 _position
    8
    ;; int64 _error
    8
    ;; int64 _speed
    8
    ;; float64 _load
    8
    ;; float64 _voltage
    8
    ;; int32 _temperature
    4
    ;; bool _moving
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _id
       (write-long _id s)
     ;; int64 _goal
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _goal (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _goal) (= (length (_goal . bv)) 2)) ;; bignum
              (write-long (ash (elt (_goal . bv) 0) 0) s)
              (write-long (ash (elt (_goal . bv) 1) -1) s))
             ((and (class _goal) (= (length (_goal . bv)) 1)) ;; big1
              (write-long (elt (_goal . bv) 0) s)
              (write-long (if (>= _goal 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _goal s)(write-long (if (>= _goal 0) 0 #xffffffff) s)))
     ;; int64 _position
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _position (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _position) (= (length (_position . bv)) 2)) ;; bignum
              (write-long (ash (elt (_position . bv) 0) 0) s)
              (write-long (ash (elt (_position . bv) 1) -1) s))
             ((and (class _position) (= (length (_position . bv)) 1)) ;; big1
              (write-long (elt (_position . bv) 0) s)
              (write-long (if (>= _position 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _position s)(write-long (if (>= _position 0) 0 #xffffffff) s)))
     ;; int64 _error
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _error (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _error) (= (length (_error . bv)) 2)) ;; bignum
              (write-long (ash (elt (_error . bv) 0) 0) s)
              (write-long (ash (elt (_error . bv) 1) -1) s))
             ((and (class _error) (= (length (_error . bv)) 1)) ;; big1
              (write-long (elt (_error . bv) 0) s)
              (write-long (if (>= _error 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _error s)(write-long (if (>= _error 0) 0 #xffffffff) s)))
     ;; int64 _speed
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _speed (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _speed) (= (length (_speed . bv)) 2)) ;; bignum
              (write-long (ash (elt (_speed . bv) 0) 0) s)
              (write-long (ash (elt (_speed . bv) 1) -1) s))
             ((and (class _speed) (= (length (_speed . bv)) 1)) ;; big1
              (write-long (elt (_speed . bv) 0) s)
              (write-long (if (>= _speed 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _speed s)(write-long (if (>= _speed 0) 0 #xffffffff) s)))
     ;; float64 _load
       (sys::poke _load (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _temperature
       (write-long _temperature s)
     ;; bool _moving
       (if _moving (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int64 _goal
#+(or :alpha :irix6 :x86_64)
      (setf _goal (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _goal (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _position
#+(or :alpha :irix6 :x86_64)
      (setf _position (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _position (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _error
#+(or :alpha :irix6 :x86_64)
      (setf _error (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _error (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _speed
#+(or :alpha :irix6 :x86_64)
      (setf _speed (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _speed (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _load
     (setq _load (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _voltage
     (setq _voltage (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _temperature
     (setq _temperature (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _moving
     (setq _moving (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get dynamixel_msgs::MotorState :md5sum-) "2184111e0e6b6ad4961bf5f12a5055e3")
(setf (get dynamixel_msgs::MotorState :datatype-) "dynamixel_msgs/MotorState")
(setf (get dynamixel_msgs::MotorState :definition-)
      "float64 timestamp   # motor state is at this time
int32 id            # motor id
int64 goal          # commanded position (in encoder units)
int64 position      # current position (in encoder units)
int64 error         # difference between current and goal positions
int64 speed         # current speed (0.111 rpm per unit)
float64 load        # current load - ratio of applied torque over maximum torque
float64 voltage     # current voltage (V)
int32 temperature   # current temperature (degrees Celsius)
bool moving         # whether the motor is currently in motion
#TODO check int64 vs int32
")



(provide :dynamixel_msgs/MotorState "2184111e0e6b6ad4961bf5f12a5055e3")


