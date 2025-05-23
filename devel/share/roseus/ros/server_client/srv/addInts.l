;; Auto-generated. Do not edit!


(when (boundp 'server_client::addInts)
  (if (not (find-package "SERVER_CLIENT"))
    (make-package "SERVER_CLIENT"))
  (shadow 'addInts (find-package "SERVER_CLIENT")))
(unless (find-package "SERVER_CLIENT::ADDINTS")
  (make-package "SERVER_CLIENT::ADDINTS"))
(unless (find-package "SERVER_CLIENT::ADDINTSREQUEST")
  (make-package "SERVER_CLIENT::ADDINTSREQUEST"))
(unless (find-package "SERVER_CLIENT::ADDINTSRESPONSE")
  (make-package "SERVER_CLIENT::ADDINTSRESPONSE"))

(in-package "ROS")





(defclass server_client::addIntsRequest
  :super ros::object
  :slots (_num1 _num2 ))

(defmethod server_client::addIntsRequest
  (:init
   (&key
    ((:num1 __num1) 0)
    ((:num2 __num2) 0)
    )
   (send-super :init)
   (setq _num1 (round __num1))
   (setq _num2 (round __num2))
   self)
  (:num1
   (&optional __num1)
   (if __num1 (setq _num1 __num1)) _num1)
  (:num2
   (&optional __num2)
   (if __num2 (setq _num2 __num2)) _num2)
  (:serialization-length
   ()
   (+
    ;; int32 _num1
    4
    ;; int32 _num2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _num1
       (write-long _num1 s)
     ;; int32 _num2
       (write-long _num2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _num1
     (setq _num1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num2
     (setq _num2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass server_client::addIntsResponse
  :super ros::object
  :slots (_sum ))

(defmethod server_client::addIntsResponse
  (:init
   (&key
    ((:sum __sum) 0)
    )
   (send-super :init)
   (setq _sum (round __sum))
   self)
  (:sum
   (&optional __sum)
   (if __sum (setq _sum __sum)) _sum)
  (:serialization-length
   ()
   (+
    ;; int32 _sum
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _sum
       (write-long _sum s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _sum
     (setq _sum (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass server_client::addInts
  :super ros::object
  :slots ())

(setf (get server_client::addInts :md5sum-) "4781436a0c2affec8025955a6041e481")
(setf (get server_client::addInts :datatype-) "server_client/addInts")
(setf (get server_client::addInts :request) server_client::addIntsRequest)
(setf (get server_client::addInts :response) server_client::addIntsResponse)

(defmethod server_client::addIntsRequest
  (:response () (instance server_client::addIntsResponse :init)))

(setf (get server_client::addIntsRequest :md5sum-) "4781436a0c2affec8025955a6041e481")
(setf (get server_client::addIntsRequest :datatype-) "server_client/addIntsRequest")
(setf (get server_client::addIntsRequest :definition-)
      "int32 num1
int32 num2
---
int32 sum

")

(setf (get server_client::addIntsResponse :md5sum-) "4781436a0c2affec8025955a6041e481")
(setf (get server_client::addIntsResponse :datatype-) "server_client/addIntsResponse")
(setf (get server_client::addIntsResponse :definition-)
      "int32 num1
int32 num2
---
int32 sum

")



(provide :server_client/addInts "4781436a0c2affec8025955a6041e481")


