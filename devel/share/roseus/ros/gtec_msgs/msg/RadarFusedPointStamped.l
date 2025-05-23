;; Auto-generated. Do not edit!


(when (boundp 'gtec_msgs::RadarFusedPointStamped)
  (if (not (find-package "GTEC_MSGS"))
    (make-package "GTEC_MSGS"))
  (shadow 'RadarFusedPointStamped (find-package "GTEC_MSGS")))
(unless (find-package "GTEC_MSGS::RADARFUSEDPOINTSTAMPED")
  (make-package "GTEC_MSGS::RADARFUSEDPOINTSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude RadarFusedPointStamped.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass gtec_msgs::RadarFusedPointStamped
  :super ros::object
  :slots (_header _point _targetId ))

(defmethod gtec_msgs::RadarFusedPointStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:point __point) (instance geometry_msgs::Point :init))
    ((:targetId __targetId) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _point __point)
   (setq _targetId (round __targetId))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:targetId
   (&optional __targetId)
   (if __targetId (setq _targetId __targetId)) _targetId)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ;; int16 _targetId
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;; int16 _targetId
       (write-word _targetId s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; int16 _targetId
     (setq _targetId (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get gtec_msgs::RadarFusedPointStamped :md5sum-) "67b06ac8c7ada8b7d8c8daf91df0c483")
(setf (get gtec_msgs::RadarFusedPointStamped :datatype-) "gtec_msgs/RadarFusedPointStamped")
(setf (get gtec_msgs::RadarFusedPointStamped :definition-)
      "Header header
geometry_msgs/Point point
int16 targetId
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

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :gtec_msgs/RadarFusedPointStamped "67b06ac8c7ada8b7d8c8daf91df0c483")


