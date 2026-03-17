; Auto-generated. Do not edit!


(cl:in-package carpet_detect_msgs-msg)


;//! \htmlinclude ObjectCameraDetectResultArray.msg.html

(cl:defclass <ObjectCameraDetectResultArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (results
    :reader results
    :initarg :results
    :type (cl:vector carpet_detect_msgs-msg:ObjectCameraDetectResult)
   :initform (cl:make-array 0 :element-type 'carpet_detect_msgs-msg:ObjectCameraDetectResult :initial-element (cl:make-instance 'carpet_detect_msgs-msg:ObjectCameraDetectResult))))
)

(cl:defclass ObjectCameraDetectResultArray (<ObjectCameraDetectResultArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectCameraDetectResultArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectCameraDetectResultArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name carpet_detect_msgs-msg:<ObjectCameraDetectResultArray> is deprecated: use carpet_detect_msgs-msg:ObjectCameraDetectResultArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObjectCameraDetectResultArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader carpet_detect_msgs-msg:header-val is deprecated.  Use carpet_detect_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'results-val :lambda-list '(m))
(cl:defmethod results-val ((m <ObjectCameraDetectResultArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader carpet_detect_msgs-msg:results-val is deprecated.  Use carpet_detect_msgs-msg:results instead.")
  (results m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectCameraDetectResultArray>) ostream)
  "Serializes a message object of type '<ObjectCameraDetectResultArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'results))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'results))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectCameraDetectResultArray>) istream)
  "Deserializes a message object of type '<ObjectCameraDetectResultArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'results) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'results)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'carpet_detect_msgs-msg:ObjectCameraDetectResult))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectCameraDetectResultArray>)))
  "Returns string type for a message object of type '<ObjectCameraDetectResultArray>"
  "carpet_detect_msgs/ObjectCameraDetectResultArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectCameraDetectResultArray)))
  "Returns string type for a message object of type 'ObjectCameraDetectResultArray"
  "carpet_detect_msgs/ObjectCameraDetectResultArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectCameraDetectResultArray>)))
  "Returns md5sum for a message object of type '<ObjectCameraDetectResultArray>"
  "144956ded8fbb06613a9be71d49669f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectCameraDetectResultArray)))
  "Returns md5sum for a message object of type 'ObjectCameraDetectResultArray"
  "144956ded8fbb06613a9be71d49669f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectCameraDetectResultArray>)))
  "Returns full string definition for message of type '<ObjectCameraDetectResultArray>"
  (cl:format cl:nil "std_msgs/Header header~%ObjectCameraDetectResult[] results~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: carpet_detect_msgs/ObjectCameraDetectResult~%float32 prop~%int32 cls_id~%CameraCoordinate[24] coords~%================================================================================~%MSG: carpet_detect_msgs/CameraCoordinate~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectCameraDetectResultArray)))
  "Returns full string definition for message of type 'ObjectCameraDetectResultArray"
  (cl:format cl:nil "std_msgs/Header header~%ObjectCameraDetectResult[] results~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: carpet_detect_msgs/ObjectCameraDetectResult~%float32 prop~%int32 cls_id~%CameraCoordinate[24] coords~%================================================================================~%MSG: carpet_detect_msgs/CameraCoordinate~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectCameraDetectResultArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'results) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectCameraDetectResultArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectCameraDetectResultArray
    (cl:cons ':header (header msg))
    (cl:cons ':results (results msg))
))
