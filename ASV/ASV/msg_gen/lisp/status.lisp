; Auto-generated. Do not edit!


(cl:in-package ASV-msg)


;//! \htmlinclude status.msg.html

(cl:defclass <status> (roslisp-msg-protocol:ros-message)
  ((nodeID
    :reader nodeID
    :initarg :nodeID
    :type cl:integer
    :initform 0)
   (status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass status (<status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ASV-msg:<status> is deprecated: use ASV-msg:status instead.")))

(cl:ensure-generic-function 'nodeID-val :lambda-list '(m))
(cl:defmethod nodeID-val ((m <status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:nodeID-val is deprecated.  Use ASV-msg:nodeID instead.")
  (nodeID m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:status-val is deprecated.  Use ASV-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <status>) ostream)
  "Serializes a message object of type '<status>"
  (cl:let* ((signed (cl:slot-value msg 'nodeID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <status>) istream)
  "Deserializes a message object of type '<status>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nodeID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<status>)))
  "Returns string type for a message object of type '<status>"
  "ASV/status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'status)))
  "Returns string type for a message object of type 'status"
  "ASV/status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<status>)))
  "Returns md5sum for a message object of type '<status>"
  "722a3850d404093d2b94283695c2a465")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'status)))
  "Returns md5sum for a message object of type 'status"
  "722a3850d404093d2b94283695c2a465")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<status>)))
  "Returns full string definition for message of type '<status>"
  (cl:format cl:nil "int32 nodeID~%bool status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'status)))
  "Returns full string definition for message of type 'status"
  (cl:format cl:nil "int32 nodeID~%bool status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <status>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <status>))
  "Converts a ROS message object to a list"
  (cl:list 'status
    (cl:cons ':nodeID (nodeID msg))
    (cl:cons ':status (status msg))
))
