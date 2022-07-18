
(cl:in-package :asdf)

(defsystem "grasp_pointcloud-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PointBoundingBox" :depends-on ("_package_PointBoundingBox"))
    (:file "_package_PointBoundingBox" :depends-on ("_package"))
  ))