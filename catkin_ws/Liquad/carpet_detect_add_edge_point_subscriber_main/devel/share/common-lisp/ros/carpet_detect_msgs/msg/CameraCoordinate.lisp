; Auto-generated. Do not edit!


(cl:in-package carpet_detect_msgs-msg)


;//! \htmlinclude CameraCoordinate.msg.html

(cl:defclass <CameraCoordinate> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass CameraCoordinate (<CameraCoordinate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraCoordinate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraCoordinate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name carpet_detect_msgs-msg:<CameraCoordinate> is deprecated: use carpet_detect_msgs-msg:CameraCoordinate instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <CameraCoordinate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader carpet_detect_msgs-msg:x-val is deprecated.  Use carpet_detect_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <CameraCoordinate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader carpet_detect_msgs-msg:y-val is deprecated.  Use carpet_detect_msgs-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <CameraCoordinate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader carpet_detect_msgs-msg:z-val is deprecated.  Use carpet_detect_msgs-msg:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraCoordinate>) ostream)
  "Serializes a message object of type '<CameraCoordinate>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraCoordinate>) istream)
  "Deserializes a message object of type '<CameraCoordinate>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraCoordinate>)))
  "Returns string type for a message object of type '<CameraCoordinate>"
  "carpet_detect_msgs/CameraCoordinate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraCoordinate)))
  "Returns string type for a message object of type 'CameraCoordinate"
  "carpet_detect_msgs/CameraCoordinate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraCoordinate>)))
  "Returns md5sum for a message object of type '<CameraCoordinate>"
  "cc153912f1453b708d221682bc23d9ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraCoordinate)))
  "Returns md5sum for a message object of type 'CameraCoordinate"
  "cc153912f1453b708d221682bc23d9ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraCoordinate>)))
  "Returns full string definition for message of type '<CameraCoordinate>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraCoordinate)))
  "Returns full string definition for message of type 'CameraCoordinate"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraCoordinate>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraCoordinate>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraCoordinate
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
