
(cl:in-package :asdf)

(defsystem "grasp_pointcloud-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AdjustParams" :depends-on ("_package_AdjustParams"))
    (:file "_package_AdjustParams" :depends-on ("_package"))
    (:file "GraspParams" :depends-on ("_package_GraspParams"))
    (:file "_package_GraspParams" :depends-on ("_package"))
    (:file "PointBoundingBox" :depends-on ("_package_PointBoundingBox"))
    (:file "_package_PointBoundingBox" :depends-on ("_package"))
    (:file "VolumeParams" :depends-on ("_package_VolumeParams"))
    (:file "_package_VolumeParams" :depends-on ("_package"))
  ))