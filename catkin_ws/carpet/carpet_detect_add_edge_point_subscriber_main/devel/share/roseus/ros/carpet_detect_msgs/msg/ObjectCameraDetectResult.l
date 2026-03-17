;; Auto-generated. Do not edit!


(when (boundp 'carpet_detect_msgs::ObjectCameraDetectResult)
  (if (not (find-package "CARPET_DETECT_MSGS"))
    (make-package "CARPET_DETECT_MSGS"))
  (shadow 'ObjectCameraDetectResult (find-package "CARPET_DETECT_MSGS")))
(unless (find-package "CARPET_DETECT_MSGS::OBJECTCAMERADETECTRESULT")
  (make-package "CARPET_DETECT_MSGS::OBJECTCAMERADETECTRESULT"))

(in-package "ROS")
;;//! \htmlinclude ObjectCameraDetectResult.msg.html


(defclass carpet_detect_msgs::ObjectCameraDetectResult
  :super ros::object
  :slots (_prop _cls_id _coords ))

(defmethod carpet_detect_msgs::ObjectCameraDetectResult
  (:init
   (&key
    ((:prop __prop) 0.0)
    ((:cls_id __cls_id) 0)
    ((:coords __coords) (let (r) (dotimes (i 24) (push (instance carpet_detect_msgs::CameraCoordinate :init) r)) r))
    )
   (send-super :init)
   (setq _prop (float __prop))
   (setq _cls_id (round __cls_id))
   (setq _coords __coords)
   self)
  (:prop
   (&optional __prop)
   (if __prop (setq _prop __prop)) _prop)
  (:cls_id
   (&optional __cls_id)
   (if __cls_id (setq _cls_id __cls_id)) _cls_id)
  (:coords
   (&rest __coords)
   (if (keywordp (car __coords))
       (send* _coords __coords)
     (progn
       (if __coords (setq _coords (car __coords)))
       _coords)))
  (:serialization-length
   ()
   (+
    ;; float32 _prop
    4
    ;; int32 _cls_id
    4
    ;; carpet_detect_msgs/CameraCoordinate[24] _coords
    (apply #'+ (send-all _coords :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _prop
       (sys::poke _prop (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _cls_id
       (write-long _cls_id s)
     ;; carpet_detect_msgs/CameraCoordinate[24] _coords
     (dolist (elem _coords)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _prop
     (setq _prop (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _cls_id
     (setq _cls_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; carpet_detect_msgs/CameraCoordinate[24] _coords
   (dotimes (i 24)
     (send (elt _coords i) :deserialize buf ptr-) (incf ptr- (send (elt _coords i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get carpet_detect_msgs::ObjectCameraDetectResult :md5sum-) "3d362a9701db027a22375f917c033511")
(setf (get carpet_detect_msgs::ObjectCameraDetectResult :datatype-) "carpet_detect_msgs/ObjectCameraDetectResult")
(setf (get carpet_detect_msgs::ObjectCameraDetectResult :definition-)
      "float32 prop
int32 cls_id
CameraCoordinate[24] coords
================================================================================
MSG: carpet_detect_msgs/CameraCoordinate
float32 x
float32 y
float32 z

")



(provide :carpet_detect_msgs/ObjectCameraDetectResult "3d362a9701db027a22375f917c033511")


