; Auto-generated. Do not edit!


(cl:in-package grasp_pointcloud-msg)


;//! \htmlinclude AdjustParams.msg.html

(cl:defclass <AdjustParams> (roslisp-msg-protocol:ros-message)
  ((y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (y_rotate_angle
    :reader y_rotate_angle
    :initarg :y_rotate_angle
    :type cl:float
    :initform 0.0)
   (z_rotate_angle
    :reader z_rotate_angle
    :initarg :z_rotate_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass AdjustParams (<AdjustParams>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AdjustParams>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AdjustParams)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name grasp_pointcloud-msg:<AdjustParams> is deprecated: use grasp_pointcloud-msg:AdjustParams instead.")))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <AdjustParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grasp_pointcloud-msg:y-val is deprecated.  Use grasp_pointcloud-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <AdjustParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grasp_pointcloud-msg:z-val is deprecated.  Use grasp_pointcloud-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'y_rotate_angle-val :lambda-list '(m))
(cl:defmethod y_rotate_angle-val ((m <AdjustParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grasp_pointcloud-msg:y_rotate_angle-val is deprecated.  Use grasp_pointcloud-msg:y_rotate_angle instead.")
  (y_rotate_angle m))

(cl:ensure-generic-function 'z_rotate_angle-val :lambda-list '(m))
(cl:defmethod z_rotate_angle-val ((m <AdjustParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grasp_pointcloud-msg:z_rotate_angle-val is deprecated.  Use grasp_pointcloud-msg:z_rotate_angle instead.")
  (z_rotate_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AdjustParams>) ostream)
  "Serializes a message object of type '<AdjustParams>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_rotate_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z_rotate_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AdjustParams>) istream)
  "Deserializes a message object of type '<AdjustParams>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_rotate_angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_rotate_angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AdjustParams>)))
  "Returns string type for a message object of type '<AdjustParams>"
  "grasp_pointcloud/AdjustParams")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AdjustParams)))
  "Returns string type for a message object of type 'AdjustParams"
  "grasp_pointcloud/AdjustParams")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AdjustParams>)))
  "Returns md5sum for a message object of type '<AdjustParams>"
  "57feed5981af266697038b66e7af4ff4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AdjustParams)))
  "Returns md5sum for a message object of type 'AdjustParams"
  "57feed5981af266697038b66e7af4ff4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AdjustParams>)))
  "Returns full string definition for message of type '<AdjustParams>"
  (cl:format cl:nil "float64 y~%float64 z~%float64 y_rotate_angle~%float64 z_rotate_angle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AdjustParams)))
  "Returns full string definition for message of type 'AdjustParams"
  (cl:format cl:nil "float64 y~%float64 z~%float64 y_rotate_angle~%float64 z_rotate_angle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AdjustParams>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AdjustParams>))
  "Converts a ROS message object to a list"
  (cl:list 'AdjustParams
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':y_rotate_angle (y_rotate_angle msg))
    (cl:cons ':z_rotate_angle (z_rotate_angle msg))
))
