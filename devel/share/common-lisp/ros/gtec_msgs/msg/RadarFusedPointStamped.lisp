; Auto-generated. Do not edit!


(cl:in-package gtec_msgs-msg)


;//! \htmlinclude RadarFusedPointStamped.msg.html

(cl:defclass <RadarFusedPointStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (targetId
    :reader targetId
    :initarg :targetId
    :type cl:fixnum
    :initform 0))
)

(cl:defclass RadarFusedPointStamped (<RadarFusedPointStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RadarFusedPointStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RadarFusedPointStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gtec_msgs-msg:<RadarFusedPointStamped> is deprecated: use gtec_msgs-msg:RadarFusedPointStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RadarFusedPointStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gtec_msgs-msg:header-val is deprecated.  Use gtec_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <RadarFusedPointStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gtec_msgs-msg:point-val is deprecated.  Use gtec_msgs-msg:point instead.")
  (point m))

(cl:ensure-generic-function 'targetId-val :lambda-list '(m))
(cl:defmethod targetId-val ((m <RadarFusedPointStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gtec_msgs-msg:targetId-val is deprecated.  Use gtec_msgs-msg:targetId instead.")
  (targetId m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RadarFusedPointStamped>) ostream)
  "Serializes a message object of type '<RadarFusedPointStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
  (cl:let* ((signed (cl:slot-value msg 'targetId)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RadarFusedPointStamped>) istream)
  "Deserializes a message object of type '<RadarFusedPointStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'targetId) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RadarFusedPointStamped>)))
  "Returns string type for a message object of type '<RadarFusedPointStamped>"
  "gtec_msgs/RadarFusedPointStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RadarFusedPointStamped)))
  "Returns string type for a message object of type 'RadarFusedPointStamped"
  "gtec_msgs/RadarFusedPointStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RadarFusedPointStamped>)))
  "Returns md5sum for a message object of type '<RadarFusedPointStamped>"
  "67b06ac8c7ada8b7d8c8daf91df0c483")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RadarFusedPointStamped)))
  "Returns md5sum for a message object of type 'RadarFusedPointStamped"
  "67b06ac8c7ada8b7d8c8daf91df0c483")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RadarFusedPointStamped>)))
  "Returns full string definition for message of type '<RadarFusedPointStamped>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point point~%int16 targetId~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RadarFusedPointStamped)))
  "Returns full string definition for message of type 'RadarFusedPointStamped"
  (cl:format cl:nil "Header header~%geometry_msgs/Point point~%int16 targetId~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RadarFusedPointStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RadarFusedPointStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'RadarFusedPointStamped
    (cl:cons ':header (header msg))
    (cl:cons ':point (point msg))
    (cl:cons ':targetId (targetId msg))
))
