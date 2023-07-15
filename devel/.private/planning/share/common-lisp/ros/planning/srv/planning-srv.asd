
(cl:in-package :asdf)

(defsystem "planning-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "CameraMsg" :depends-on ("_package_CameraMsg"))
    (:file "_package_CameraMsg" :depends-on ("_package"))
  ))