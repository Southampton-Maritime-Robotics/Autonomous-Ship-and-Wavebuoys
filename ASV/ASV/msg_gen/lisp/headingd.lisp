; Auto-generated. Do not edit!


(cl:in-package ASV-msg)


;//! \htmlinclude headingd.msg.html

(cl:defclass <headingd> (roslisp-msg-protocol:ros-message)
  ((heading_demand
    :reader heading_demand
    :initarg :heading_demand
    :type cl:float
    :initform 0.0))
)

(cl:defclass headingd (<headingd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <headingd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'headingd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ASV-msg:<headingd> is deprecated: use ASV-msg:headingd instead.")))

(cl:ensure-generic-function 'heading_demand-val :lambda-list '(m))
(cl:defmethod heading_demand-val ((m <headingd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:heading_demand-val is deprecated.  Use ASV-msg:heading_demand instead.")
  (heading_demand m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <headingd>) ostream)
  "Serializes a message object of type '<headingd>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'heading_demand))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <headingd>) istream)
  "Deserializes a message object of type '<headingd>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_demand) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<headingd>)))
  "Returns string type for a message object of type '<headingd>"
  "ASV/headingd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'headingd)))
  "Returns string type for a message object of type 'headingd"
  "ASV/headingd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<headingd>)))
  "Returns md5sum for a message object of type '<headingd>"
  "a033fb24cad4dc624033484eb21eea63")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'headingd)))
  "Returns md5sum for a message object of type 'headingd"
  "a033fb24cad4dc624033484eb21eea63")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<headingd>)))
  "Returns full string definition for message of type '<headingd>"
  (cl:format cl:nil "float32 heading_demand~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'headingd)))
  "Returns full string definition for message of type 'headingd"
  (cl:format cl:nil "float32 heading_demand~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <headingd>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <headingd>))
  "Converts a ROS message object to a list"
  (cl:list 'headingd
    (cl:cons ':heading_demand (heading_demand msg))
))
