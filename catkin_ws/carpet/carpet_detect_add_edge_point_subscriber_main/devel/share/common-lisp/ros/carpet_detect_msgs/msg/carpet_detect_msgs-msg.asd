
(cl:in-package :asdf)

(defsystem "carpet_detect_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CameraCoordinate" :depends-on ("_package_CameraCoordinate"))
    (:file "_package_CameraCoordinate" :depends-on ("_package"))
    (:file "ObjectCameraDetectResult" :depends-on ("_package_ObjectCameraDetectResult"))
    (:file "_package_ObjectCameraDetectResult" :depends-on ("_package"))
    (:file "ObjectCameraDetectResultArray" :depends-on ("_package_ObjectCameraDetectResultArray"))
    (:file "_package_ObjectCameraDetectResultArray" :depends-on ("_package"))
  ))