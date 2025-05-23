; Auto-generated. Do not edit!


(cl:in-package gtec_msgs-msg)


;//! \htmlinclude DoorCounterEvent.msg.html

(cl:defclass <DoorCounterEvent> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (lth
    :reader lth
    :initarg :lth
    :type cl:fixnum
    :initform 0)
   (htl
    :reader htl
    :initarg :htl
    :type cl:fixnum
    :initform 0))
)

(cl:defclass DoorCounterEvent (<DoorCounterEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DoorCounterEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DoorCounterEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gtec_msgs-msg:<DoorCounterEvent> is deprecated: use gtec_msgs-msg:DoorCounterEvent instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DoorCounterEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gtec_msgs-msg:header-val is deprecated.  Use gtec_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'lth-val :lambda-list '(m))
(cl:defmethod lth-val ((m <DoorCounterEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gtec_msgs-msg:lth-val is deprecated.  Use gtec_msgs-msg:lth instead.")
  (lth m))

(cl:ensure-generic-function 'htl-val :lambda-list '(m))
(cl:defmethod htl-val ((m <DoorCounterEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gtec_msgs-msg:htl-val is deprecated.  Use gtec_msgs-msg:htl instead.")
  (htl m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DoorCounterEvent>) ostream)
  "Serializes a message object of type '<DoorCounterEvent>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'lth)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'htl)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DoorCounterEvent>) istream)
  "Deserializes a message object of type '<DoorCounterEvent>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lth) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'htl) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DoorCounterEvent>)))
  "Returns string type for a message object of type '<DoorCounterEvent>"
  "gtec_msgs/DoorCounterEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DoorCounterEvent)))
  "Returns string type for a message object of type 'DoorCounterEvent"
  "gtec_msgs/DoorCounterEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DoorCounterEvent>)))
  "Returns md5sum for a message object of type '<DoorCounterEvent>"
  "8e9928bb0cb3ba644932e73b3a112f9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DoorCounterEvent)))
  "Returns md5sum for a message object of type 'DoorCounterEvent"
  "8e9928bb0cb3ba644932e73b3a112f9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DoorCounterEvent>)))
  "Returns full string definition for message of type '<DoorCounterEvent>"
  (cl:format cl:nil "Header header~%int16 lth~%int16 htl~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DoorCounterEvent)))
  "Returns full string definition for message of type 'DoorCounterEvent"
  (cl:format cl:nil "Header header~%int16 lth~%int16 htl~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DoorCounterEvent>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DoorCounterEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'DoorCounterEvent
    (cl:cons ':header (header msg))
    (cl:cons ':lth (lth msg))
    (cl:cons ':htl (htl msg))
))
