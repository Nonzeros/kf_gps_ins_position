;; Auto-generated. Do not edit!


(when (boundp 'gtec_msgs::DoorCounterEvent)
  (if (not (find-package "GTEC_MSGS"))
    (make-package "GTEC_MSGS"))
  (shadow 'DoorCounterEvent (find-package "GTEC_MSGS")))
(unless (find-package "GTEC_MSGS::DOORCOUNTEREVENT")
  (make-package "GTEC_MSGS::DOORCOUNTEREVENT"))

(in-package "ROS")
;;//! \htmlinclude DoorCounterEvent.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass gtec_msgs::DoorCounterEvent
  :super ros::object
  :slots (_header _lth _htl ))

(defmethod gtec_msgs::DoorCounterEvent
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lth __lth) 0)
    ((:htl __htl) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _lth (round __lth))
   (setq _htl (round __htl))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lth
   (&optional __lth)
   (if __lth (setq _lth __lth)) _lth)
  (:htl
   (&optional __htl)
   (if __htl (setq _htl __htl)) _htl)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16 _lth
    2
    ;; int16 _htl
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16 _lth
       (write-word _lth s)
     ;; int16 _htl
       (write-word _htl s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16 _lth
     (setq _lth (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _htl
     (setq _htl (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get gtec_msgs::DoorCounterEvent :md5sum-) "8e9928bb0cb3ba644932e73b3a112f9c")
(setf (get gtec_msgs::DoorCounterEvent :datatype-) "gtec_msgs/DoorCounterEvent")
(setf (get gtec_msgs::DoorCounterEvent :definition-)
      "Header header
int16 lth
int16 htl
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :gtec_msgs/DoorCounterEvent "8e9928bb0cb3ba644932e73b3a112f9c")


