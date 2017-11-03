; Auto-generated. Do not edit!


(cl:in-package ASV-msg)


;//! \htmlinclude rudder.msg.html

(cl:defclass <rudder> (roslisp-msg-protocol:ros-message)
  ((rudder_demand
    :reader rudder_demand
    :initarg :rudder_demand
    :type cl:float
    :initform 0.0))
)

(cl:defclass rudder (<rudder>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rudder>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rudder)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ASV-msg:<rudder> is deprecated: use ASV-msg:rudder instead.")))

(cl:ensure-generic-function 'rudder_demand-val :lambda-list '(m))
(cl:defmethod rudder_demand-val ((m <rudder>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:rudder_demand-val is deprecated.  Use ASV-msg:rudder_demand instead.")
  (rudder_demand m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rudder>) ostream)
  "Serializes a message object of type '<rudder>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rudder_demand))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rudder>) istream)
  "Deserializes a message object of type '<rudder>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rudder_demand) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rudder>)))
  "Returns string type for a message object of type '<rudder>"
  "ASV/rudder")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rudder)))
  "Returns string type for a message object of type 'rudder"
  "ASV/rudder")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rudder>)))
  "Returns md5sum for a message object of type '<rudder>"
  "60078da9d4c3d9307b5235a0a0d7c95a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rudder)))
  "Returns md5sum for a message object of type 'rudder"
  "60078da9d4c3d9307b5235a0a0d7c95a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rudder>)))
  "Returns full string definition for message of type '<rudder>"
  (cl:format cl:nil "float32 rudder_demand~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rudder)))
  "Returns full string definition for message of type 'rudder"
  (cl:format cl:nil "float32 rudder_demand~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rudder>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rudder>))
  "Converts a ROS message object to a list"
  (cl:list 'rudder
    (cl:cons ':rudder_demand (rudder_demand msg))
))
