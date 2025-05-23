;; Auto-generated. Do not edit!


(when (boundp 'gtec_msgs::ZoneOccupancy)
  (if (not (find-package "GTEC_MSGS"))
    (make-package "GTEC_MSGS"))
  (shadow 'ZoneOccupancy (find-package "GTEC_MSGS")))
(unless (find-package "GTEC_MSGS::ZONEOCCUPANCY")
  (make-package "GTEC_MSGS::ZONEOCCUPANCY"))

(in-package "ROS")
;;//! \htmlinclude ZoneOccupancy.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass gtec_msgs::ZoneOccupancy
  :super ros::object
  :slots (_header _zoneId _targetIds _count ))

(defmethod gtec_msgs::ZoneOccupancy
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:zoneId __zoneId) "")
    ((:targetIds __targetIds) (make-array 0 :initial-element 0 :element-type :integer))
    ((:count __count) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _zoneId (string __zoneId))
   (setq _targetIds __targetIds)
   (setq _count (round __count))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:zoneId
   (&optional __zoneId)
   (if __zoneId (setq _zoneId __zoneId)) _zoneId)
  (:targetIds
   (&optional __targetIds)
   (if __targetIds (setq _targetIds __targetIds)) _targetIds)
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _zoneId
    4 (length _zoneId)
    ;; int16[] _targetIds
    (* 2    (length _targetIds)) 4
    ;; int16 _count
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _zoneId
       (write-long (length _zoneId) s) (princ _zoneId s)
     ;; int16[] _targetIds
     (write-long (length _targetIds) s)
     (dotimes (i (length _targetIds))
       (write-word (elt _targetIds i) s)
       )
     ;; int16 _count
       (write-word _count s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _zoneId
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _zoneId (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16[] _targetIds
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _targetIds (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _targetIds i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; int16 _count
     (setq _count (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get gtec_msgs::ZoneOccupancy :md5sum-) "ca4a300ff4c235be9feab5048bc4c41f")
(setf (get gtec_msgs::ZoneOccupancy :datatype-) "gtec_msgs/ZoneOccupancy")
(setf (get gtec_msgs::ZoneOccupancy :definition-)
      "Header header
string zoneId
int16[] targetIds
int16 count
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



(provide :gtec_msgs/ZoneOccupancy "ca4a300ff4c235be9feab5048bc4c41f")


