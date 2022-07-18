;; Auto-generated. Do not edit!


(when (boundp 'grasp_pointcloud::PointBoundingBox)
  (if (not (find-package "GRASP_POINTCLOUD"))
    (make-package "GRASP_POINTCLOUD"))
  (shadow 'PointBoundingBox (find-package "GRASP_POINTCLOUD")))
(unless (find-package "GRASP_POINTCLOUD::POINTBOUNDINGBOX")
  (make-package "GRASP_POINTCLOUD::POINTBOUNDINGBOX"))

(in-package "ROS")
;;//! \htmlinclude PointBoundingBox.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass grasp_pointcloud::PointBoundingBox
  :super ros::object
  :slots (_header _xmin _ymin _xmax _ymax ))

(defmethod grasp_pointcloud::PointBoundingBox
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:xmin __xmin) 0.0)
    ((:ymin __ymin) 0.0)
    ((:xmax __xmax) 0.0)
    ((:ymax __ymax) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _xmin (float __xmin))
   (setq _ymin (float __ymin))
   (setq _xmax (float __xmax))
   (setq _ymax (float __ymax))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
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
    ;; std_msgs/Header _header
    (send _header :serialization-length)
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
     ;; std_msgs/Header _header
       (send _header :serialize s)
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
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
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

(setf (get grasp_pointcloud::PointBoundingBox :md5sum-) "8325290e99152964b5a41f6290f76ea2")
(setf (get grasp_pointcloud::PointBoundingBox :datatype-) "grasp_pointcloud/PointBoundingBox")
(setf (get grasp_pointcloud::PointBoundingBox :definition-)
      "Header header
float64 xmin
float64 ymin
float64 xmax
float64 ymax

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :grasp_pointcloud/PointBoundingBox "8325290e99152964b5a41f6290f76ea2")


