; Auto-generated. Do not edit!


(cl:in-package gps_simulator-msg)


;//! \htmlinclude gps_measurement.msg.html

(cl:defclass <gps_measurement> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass gps_measurement (<gps_measurement>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gps_measurement>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gps_measurement)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gps_simulator-msg:<gps_measurement> is deprecated: use gps_simulator-msg:gps_measurement instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <gps_measurement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_simulator-msg:x-val is deprecated.  Use gps_simulator-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <gps_measurement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_simulator-msg:z-val is deprecated.  Use gps_simulator-msg:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gps_measurement>) ostream)
  "Serializes a message object of type '<gps_measurement>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gps_measurement>) istream)
  "Deserializes a message object of type '<gps_measurement>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gps_measurement>)))
  "Returns string type for a message object of type '<gps_measurement>"
  "gps_simulator/gps_measurement")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gps_measurement)))
  "Returns string type for a message object of type 'gps_measurement"
  "gps_simulator/gps_measurement")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gps_measurement>)))
  "Returns md5sum for a message object of type '<gps_measurement>"
  "3d990ebeae1ee5ba6990ba82cc74e4c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gps_measurement)))
  "Returns md5sum for a message object of type 'gps_measurement"
  "3d990ebeae1ee5ba6990ba82cc74e4c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gps_measurement>)))
  "Returns full string definition for message of type '<gps_measurement>"
  (cl:format cl:nil "float32 x~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gps_measurement)))
  "Returns full string definition for message of type 'gps_measurement"
  (cl:format cl:nil "float32 x~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gps_measurement>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gps_measurement>))
  "Converts a ROS message object to a list"
  (cl:list 'gps_measurement
    (cl:cons ':x (x msg))
    (cl:cons ':z (z msg))
))
