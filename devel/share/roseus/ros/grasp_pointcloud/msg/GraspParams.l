;; Auto-generated. Do not edit!


(when (boundp 'grasp_pointcloud::GraspParams)
  (if (not (find-package "GRASP_POINTCLOUD"))
    (make-package "GRASP_POINTCLOUD"))
  (shadow 'GraspParams (find-package "GRASP_POINTCLOUD")))
(unless (find-package "GRASP_POINTCLOUD::GRASPPARAMS")
  (make-package "GRASP_POINTCLOUD::GRASPPARAMS"))

(in-package "ROS")
;;//! \htmlinclude GraspParams.msg.html


(defclass grasp_pointcloud::GraspParams
  :super ros::object
  :slots (_x _y _z _rotate_angle _tilt_angle _grasp_width_first _grasp_width_second ))

(defmethod grasp_pointcloud::GraspParams
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:rotate_angle __rotate_angle) 0.0)
    ((:tilt_angle __tilt_angle) 0.0)
    ((:grasp_width_first __grasp_width_first) 0.0)
    ((:grasp_width_second __grasp_width_second) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _rotate_angle (float __rotate_angle))
   (setq _tilt_angle (float __tilt_angle))
   (setq _grasp_width_first (float __grasp_width_first))
   (setq _grasp_width_second (float __grasp_width_second))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:rotate_angle
   (&optional __rotate_angle)
   (if __rotate_angle (setq _rotate_angle __rotate_angle)) _rotate_angle)
  (:tilt_angle
   (&optional __tilt_angle)
   (if __tilt_angle (setq _tilt_angle __tilt_angle)) _tilt_angle)
  (:grasp_width_first
   (&optional __grasp_width_first)
   (if __grasp_width_first (setq _grasp_width_first __grasp_width_first)) _grasp_width_first)
  (:grasp_width_second
   (&optional __grasp_width_second)
   (if __grasp_width_second (setq _grasp_width_second __grasp_width_second)) _grasp_width_second)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _rotate_angle
    8
    ;; float64 _tilt_angle
    8
    ;; float64 _grasp_width_first
    8
    ;; float64 _grasp_width_second
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rotate_angle
       (sys::poke _rotate_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _tilt_angle
       (sys::poke _tilt_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _grasp_width_first
       (sys::poke _grasp_width_first (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _grasp_width_second
       (sys::poke _grasp_width_second (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rotate_angle
     (setq _rotate_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _tilt_angle
     (setq _tilt_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _grasp_width_first
     (setq _grasp_width_first (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _grasp_width_second
     (setq _grasp_width_second (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get grasp_pointcloud::GraspParams :md5sum-) "28bf0de5a6fc99a5d51ec496c974cccc")
(setf (get grasp_pointcloud::GraspParams :datatype-) "grasp_pointcloud/GraspParams")
(setf (get grasp_pointcloud::GraspParams :definition-)
      "float64 x
float64 y
float64 z
float64 rotate_angle
float64 tilt_angle
float64 grasp_width_first
float64 grasp_width_second

")



(provide :grasp_pointcloud/GraspParams "28bf0de5a6fc99a5d51ec496c974cccc")


