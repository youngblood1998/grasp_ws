;; Auto-generated. Do not edit!


(when (boundp 'grasp_pointcloud::VolumeParams)
  (if (not (find-package "GRASP_POINTCLOUD"))
    (make-package "GRASP_POINTCLOUD"))
  (shadow 'VolumeParams (find-package "GRASP_POINTCLOUD")))
(unless (find-package "GRASP_POINTCLOUD::VOLUMEPARAMS")
  (make-package "GRASP_POINTCLOUD::VOLUMEPARAMS"))

(in-package "ROS")
;;//! \htmlinclude VolumeParams.msg.html


(defclass grasp_pointcloud::VolumeParams
  :super ros::object
  :slots (_reverse _x _y _z _rotate_angle _volume _width ))

(defmethod grasp_pointcloud::VolumeParams
  (:init
   (&key
    ((:reverse __reverse) nil)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:rotate_angle __rotate_angle) 0.0)
    ((:volume __volume) 0.0)
    ((:width __width) 0.0)
    )
   (send-super :init)
   (setq _reverse __reverse)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _rotate_angle (float __rotate_angle))
   (setq _volume (float __volume))
   (setq _width (float __width))
   self)
  (:reverse
   (&optional __reverse)
   (if __reverse (setq _reverse __reverse)) _reverse)
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
  (:volume
   (&optional __volume)
   (if __volume (setq _volume __volume)) _volume)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:serialization-length
   ()
   (+
    ;; bool _reverse
    1
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _rotate_angle
    8
    ;; float64 _volume
    8
    ;; float64 _width
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _reverse
       (if _reverse (write-byte -1 s) (write-byte 0 s))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rotate_angle
       (sys::poke _rotate_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _volume
       (sys::poke _volume (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _width
       (sys::poke _width (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _reverse
     (setq _reverse (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rotate_angle
     (setq _rotate_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _volume
     (setq _volume (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _width
     (setq _width (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get grasp_pointcloud::VolumeParams :md5sum-) "f7eed6e54339e802bc0a68ef8afe8226")
(setf (get grasp_pointcloud::VolumeParams :datatype-) "grasp_pointcloud/VolumeParams")
(setf (get grasp_pointcloud::VolumeParams :definition-)
      "bool reverse
float64 x
float64 y
float64 z
float64 rotate_angle
float64 volume
float64 width
")



(provide :grasp_pointcloud/VolumeParams "f7eed6e54339e802bc0a68ef8afe8226")


