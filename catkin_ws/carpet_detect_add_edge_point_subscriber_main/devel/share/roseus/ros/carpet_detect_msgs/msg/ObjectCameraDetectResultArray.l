;; Auto-generated. Do not edit!


(when (boundp 'carpet_detect_msgs::ObjectCameraDetectResultArray)
  (if (not (find-package "CARPET_DETECT_MSGS"))
    (make-package "CARPET_DETECT_MSGS"))
  (shadow 'ObjectCameraDetectResultArray (find-package "CARPET_DETECT_MSGS")))
(unless (find-package "CARPET_DETECT_MSGS::OBJECTCAMERADETECTRESULTARRAY")
  (make-package "CARPET_DETECT_MSGS::OBJECTCAMERADETECTRESULTARRAY"))

(in-package "ROS")
;;//! \htmlinclude ObjectCameraDetectResultArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass carpet_detect_msgs::ObjectCameraDetectResultArray
  :super ros::object
  :slots (_header _results ))

(defmethod carpet_detect_msgs::ObjectCameraDetectResultArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:results __results) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _results __results)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:results
   (&rest __results)
   (if (keywordp (car __results))
       (send* _results __results)
     (progn
       (if __results (setq _results (car __results)))
       _results)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; carpet_detect_msgs/ObjectCameraDetectResult[] _results
    (apply #'+ (send-all _results :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; carpet_detect_msgs/ObjectCameraDetectResult[] _results
     (write-long (length _results) s)
     (dolist (elem _results)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; carpet_detect_msgs/ObjectCameraDetectResult[] _results
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _results (let (r) (dotimes (i n) (push (instance carpet_detect_msgs::ObjectCameraDetectResult :init) r)) r))
     (dolist (elem- _results)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get carpet_detect_msgs::ObjectCameraDetectResultArray :md5sum-) "144956ded8fbb06613a9be71d49669f9")
(setf (get carpet_detect_msgs::ObjectCameraDetectResultArray :datatype-) "carpet_detect_msgs/ObjectCameraDetectResultArray")
(setf (get carpet_detect_msgs::ObjectCameraDetectResultArray :definition-)
      "std_msgs/Header header
ObjectCameraDetectResult[] results

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
MSG: carpet_detect_msgs/ObjectCameraDetectResult
float32 prop
int32 cls_id
CameraCoordinate[24] coords
================================================================================
MSG: carpet_detect_msgs/CameraCoordinate
float32 x
float32 y
float32 z

")



(provide :carpet_detect_msgs/ObjectCameraDetectResultArray "144956ded8fbb06613a9be71d49669f9")


