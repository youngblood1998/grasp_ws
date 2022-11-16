;; Auto-generated. Do not edit!


(when (boundp 'grasp_pointcloud::AdjustParams)
  (if (not (find-package "GRASP_POINTCLOUD"))
    (make-package "GRASP_POINTCLOUD"))
  (shadow 'AdjustParams (find-package "GRASP_POINTCLOUD")))
(unless (find-package "GRASP_POINTCLOUD::ADJUSTPARAMS")
  (make-package "GRASP_POINTCLOUD::ADJUSTPARAMS"))

(in-package "ROS")
;;//! \htmlinclude AdjustParams.msg.html


(defclass grasp_pointcloud::AdjustParams
  :super ros::object
  :slots (_y _z _y_rotate_angle _z_rotate_angle ))

(defmethod grasp_pointcloud::AdjustParams
  (:init
   (&key
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:y_rotate_angle __y_rotate_angle) 0.0)
    ((:z_rotate_angle __z_rotate_angle) 0.0)
    )
   (send-super :init)
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _y_rotate_angle (float __y_rotate_angle))
   (setq _z_rotate_angle (float __z_rotate_angle))
   self)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:y_rotate_angle
   (&optional __y_rotate_angle)
   (if __y_rotate_angle (setq _y_rotate_angle __y_rotate_angle)) _y_rotate_angle)
  (:z_rotate_angle
   (&optional __z_rotate_angle)
   (if __z_rotate_angle (setq _z_rotate_angle __z_rotate_angle)) _z_rotate_angle)
  (:serialization-length
   ()
   (+
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _y_rotate_angle
    8
    ;; float64 _z_rotate_angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_rotate_angle
       (sys::poke _y_rotate_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z_rotate_angle
       (sys::poke _z_rotate_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_rotate_angle
     (setq _y_rotate_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z_rotate_angle
     (setq _z_rotate_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get grasp_pointcloud::AdjustParams :md5sum-) "57feed5981af266697038b66e7af4ff4")
(setf (get grasp_pointcloud::AdjustParams :datatype-) "grasp_pointcloud/AdjustParams")
(setf (get grasp_pointcloud::AdjustParams :definition-)
      "float64 y
float64 z
float64 y_rotate_angle
float64 z_rotate_angle
")



(provide :grasp_pointcloud/AdjustParams "57feed5981af266697038b66e7af4ff4")


