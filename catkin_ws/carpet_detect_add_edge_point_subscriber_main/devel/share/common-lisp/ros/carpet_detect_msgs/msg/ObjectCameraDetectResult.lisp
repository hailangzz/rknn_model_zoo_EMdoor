; Auto-generated. Do not edit!


(cl:in-package carpet_detect_msgs-msg)


;//! \htmlinclude ObjectCameraDetectResult.msg.html

(cl:defclass <ObjectCameraDetectResult> (roslisp-msg-protocol:ros-message)
  ((prop
    :reader prop
    :initarg :prop
    :type cl:float
    :initform 0.0)
   (cls_id
    :reader cls_id
    :initarg :cls_id
    :type cl:integer
    :initform 0)
   (coords
    :reader coords
    :initarg :coords
    :type (cl:vector carpet_detect_msgs-msg:CameraCoordinate)
   :initform (cl:make-array 24 :element-type 'carpet_detect_msgs-msg:CameraCoordinate :initial-element (cl:make-instance 'carpet_detect_msgs-msg:CameraCoordinate))))
)

(cl:defclass ObjectCameraDetectResult (<ObjectCameraDetectResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectCameraDetectResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectCameraDetectResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name carpet_detect_msgs-msg:<ObjectCameraDetectResult> is deprecated: use carpet_detect_msgs-msg:ObjectCameraDetectResult instead.")))

(cl:ensure-generic-function 'prop-val :lambda-list '(m))
(cl:defmethod prop-val ((m <ObjectCameraDetectResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader carpet_detect_msgs-msg:prop-val is deprecated.  Use carpet_detect_msgs-msg:prop instead.")
  (prop m))

(cl:ensure-generic-function 'cls_id-val :lambda-list '(m))
(cl:defmethod cls_id-val ((m <ObjectCameraDetectResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader carpet_detect_msgs-msg:cls_id-val is deprecated.  Use carpet_detect_msgs-msg:cls_id instead.")
  (cls_id m))

(cl:ensure-generic-function 'coords-val :lambda-list '(m))
(cl:defmethod coords-val ((m <ObjectCameraDetectResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader carpet_detect_msgs-msg:coords-val is deprecated.  Use carpet_detect_msgs-msg:coords instead.")
  (coords m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectCameraDetectResult>) ostream)
  "Serializes a message object of type '<ObjectCameraDetectResult>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'prop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'cls_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'coords))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectCameraDetectResult>) istream)
  "Deserializes a message object of type '<ObjectCameraDetectResult>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'prop) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cls_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:setf (cl:slot-value msg 'coords) (cl:make-array 24))
  (cl:let ((vals (cl:slot-value msg 'coords)))
    (cl:dotimes (i 24)
    (cl:setf (cl:aref vals i) (cl:make-instance 'carpet_detect_msgs-msg:CameraCoordinate))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectCameraDetectResult>)))
  "Returns string type for a message object of type '<ObjectCameraDetectResult>"
  "carpet_detect_msgs/ObjectCameraDetectResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectCameraDetectResult)))
  "Returns string type for a message object of type 'ObjectCameraDetectResult"
  "carpet_detect_msgs/ObjectCameraDetectResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectCameraDetectResult>)))
  "Returns md5sum for a message object of type '<ObjectCameraDetectResult>"
  "3d362a9701db027a22375f917c033511")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectCameraDetectResult)))
  "Returns md5sum for a message object of type 'ObjectCameraDetectResult"
  "3d362a9701db027a22375f917c033511")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectCameraDetectResult>)))
  "Returns full string definition for message of type '<ObjectCameraDetectResult>"
  (cl:format cl:nil "float32 prop~%int32 cls_id~%CameraCoordinate[24] coords~%================================================================================~%MSG: carpet_detect_msgs/CameraCoordinate~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectCameraDetectResult)))
  "Returns full string definition for message of type 'ObjectCameraDetectResult"
  (cl:format cl:nil "float32 prop~%int32 cls_id~%CameraCoordinate[24] coords~%================================================================================~%MSG: carpet_detect_msgs/CameraCoordinate~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectCameraDetectResult>))
  (cl:+ 0
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'coords) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectCameraDetectResult>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectCameraDetectResult
    (cl:cons ':prop (prop msg))
    (cl:cons ':cls_id (cls_id msg))
    (cl:cons ':coords (coords msg))
))
