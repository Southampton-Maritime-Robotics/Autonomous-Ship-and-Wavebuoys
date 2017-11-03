; Auto-generated. Do not edit!


(cl:in-package ASV-msg)


;//! \htmlinclude arduino.msg.html

(cl:defclass <arduino> (roslisp-msg-protocol:ros-message)
  ((battery_voltage
    :reader battery_voltage
    :initarg :battery_voltage
    :type cl:float
    :initform 0.0)
   (prop_demand
    :reader prop_demand
    :initarg :prop_demand
    :type cl:float
    :initform 0.0)
   (voltage_demand
    :reader voltage_demand
    :initarg :voltage_demand
    :type cl:float
    :initform 0.0))
)

(cl:defclass arduino (<arduino>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <arduino>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'arduino)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ASV-msg:<arduino> is deprecated: use ASV-msg:arduino instead.")))

(cl:ensure-generic-function 'battery_voltage-val :lambda-list '(m))
(cl:defmethod battery_voltage-val ((m <arduino>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:battery_voltage-val is deprecated.  Use ASV-msg:battery_voltage instead.")
  (battery_voltage m))

(cl:ensure-generic-function 'prop_demand-val :lambda-list '(m))
(cl:defmethod prop_demand-val ((m <arduino>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:prop_demand-val is deprecated.  Use ASV-msg:prop_demand instead.")
  (prop_demand m))

(cl:ensure-generic-function 'voltage_demand-val :lambda-list '(m))
(cl:defmethod voltage_demand-val ((m <arduino>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:voltage_demand-val is deprecated.  Use ASV-msg:voltage_demand instead.")
  (voltage_demand m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <arduino>) ostream)
  "Serializes a message object of type '<arduino>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'battery_voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'prop_demand))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'voltage_demand))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <arduino>) istream)
  "Deserializes a message object of type '<arduino>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'battery_voltage) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'prop_demand) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'voltage_demand) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<arduino>)))
  "Returns string type for a message object of type '<arduino>"
  "ASV/arduino")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'arduino)))
  "Returns string type for a message object of type 'arduino"
  "ASV/arduino")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<arduino>)))
  "Returns md5sum for a message object of type '<arduino>"
  "c206d8bc0371a0b0ba4915d805666d5f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'arduino)))
  "Returns md5sum for a message object of type 'arduino"
  "c206d8bc0371a0b0ba4915d805666d5f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<arduino>)))
  "Returns full string definition for message of type '<arduino>"
  (cl:format cl:nil "float32 battery_voltage~%float32 prop_demand~%float32 voltage_demand~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'arduino)))
  "Returns full string definition for message of type 'arduino"
  (cl:format cl:nil "float32 battery_voltage~%float32 prop_demand~%float32 voltage_demand~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <arduino>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <arduino>))
  "Converts a ROS message object to a list"
  (cl:list 'arduino
    (cl:cons ':battery_voltage (battery_voltage msg))
    (cl:cons ':prop_demand (prop_demand msg))
    (cl:cons ':voltage_demand (voltage_demand msg))
))
