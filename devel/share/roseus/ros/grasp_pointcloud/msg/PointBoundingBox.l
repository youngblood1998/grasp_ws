;; Auto-generated. Do not edit!


(when (boundp 'grasp_pointcloud::PointBoundingBox)
  (if (not (find-package "GRASP_POINTCLOUD"))
    (make-package "GRASP_POINTCLOUD"))
  (shadow 'PointBoundingBox (find-package "GRASP_POINTCLOUD")))
(unless (find-package "GRASP_POINTCLOUD::POINTBOUNDINGBOX")
  (make-package "GRASP_POINTCLOUD::POINTBOUNDINGBOX"))

(in-package "ROS")
;;//! \htmlinclude PointBoundingBox.msg.html


(defclass grasp_pointcloud::PointBoundingBox
  :super ros::object
  :slots (_xmin _ymin _xmax _ymax ))

(defmethod grasp_pointcloud::PointBoundingBox
  (:init
   (&key
    ((:xmin __xmin) 0.0)
    ((:ymin __ymin) 0.0)
    ((:xmax __xmax) 0.0)
    ((:ymax __ymax) 0.0)
    )
   (send-super :init)
   (setq _xmin (float __xmin))
   (setq _ymin (float __ymin))
   (setq _xmax (float __xmax))
   (setq _ymax (float __ymax))
   self)
  (:xmin
   (&optional __xmin)
   (if __xmin (setq _xmin __xmin)) _xmin)
  (:ymin
   (&optional __ymin)
   (if __ymin (setq _ymin __ymin)) _ymin)
  (:xmax
   (&optional __xmax)
   (if __xmax (setq _xmax __xmax)) _xmax)
  (:ymax
   (&optional __ymax)
   (if __ymax (setq _ymax __ymax)) _ymax)
  (:serialization-length
   ()
   (+
    ;; float64 _xmin
    8
    ;; float64 _ymin
    8
    ;; float64 _xmax
    8
    ;; float64 _ymax
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _xmin
       (sys::poke _xmin (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ymin
       (sys::poke _ymin (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _xmax
       (sys::poke _xmax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ymax
       (sys::poke _ymax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _xmin
     (setq _xmin (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ymin
     (setq _ymin (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _xmax
     (setq _xmax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ymax
     (setq _ymax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get grasp_pointcloud::PointBoundingBox :md5sum-) "2da0e8eb269cd8cc187fb4c7d113d543")
(setf (get grasp_pointcloud::PointBoundingBox :datatype-) "grasp_pointcloud/PointBoundingBox")
(setf (get grasp_pointcloud::PointBoundingBox :definition-)
      "float64 xmin
float64 ymin
float64 xmax
float64 ymax

")



(provide :grasp_pointcloud/PointBoundingBox "2da0e8eb269cd8cc187fb4c7d113d543")


