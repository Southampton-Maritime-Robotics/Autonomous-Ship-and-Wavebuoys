; Auto-generated. Do not edit!


(cl:in-package ASV-msg)


;//! \htmlinclude position.msg.html

(cl:defclass <position> (roslisp-msg-protocol:ros-message)
  ((X
    :reader X
    :initarg :X
    :type cl:float
    :initform 0.0)
   (Y
    :reader Y
    :initarg :Y
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (lat
    :reader lat
    :initarg :lat
    :type cl:float
    :initform 0.0)
   (long
    :reader long
    :initarg :long
    :type cl:float
    :initform 0.0)
   (time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0)
   (ValidGPSfix
    :reader ValidGPSfix
    :initarg :ValidGPSfix
    :type cl:fixnum
    :initform 0)
   (number_of_satellites
    :reader number_of_satellites
    :initarg :number_of_satellites
    :type cl:fixnum
    :initform 0))
)

(cl:defclass position (<position>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <position>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'position)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ASV-msg:<position> is deprecated: use ASV-msg:position instead.")))

(cl:ensure-generic-function 'X-val :lambda-list '(m))
(cl:defmethod X-val ((m <position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:X-val is deprecated.  Use ASV-msg:X instead.")
  (X m))

(cl:ensure-generic-function 'Y-val :lambda-list '(m))
(cl:defmethod Y-val ((m <position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:Y-val is deprecated.  Use ASV-msg:Y instead.")
  (Y m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:speed-val is deprecated.  Use ASV-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'lat-val :lambda-list '(m))
(cl:defmethod lat-val ((m <position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:lat-val is deprecated.  Use ASV-msg:lat instead.")
  (lat m))

(cl:ensure-generic-function 'long-val :lambda-list '(m))
(cl:defmethod long-val ((m <position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:long-val is deprecated.  Use ASV-msg:long instead.")
  (long m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:time-val is deprecated.  Use ASV-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'ValidGPSfix-val :lambda-list '(m))
(cl:defmethod ValidGPSfix-val ((m <position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:ValidGPSfix-val is deprecated.  Use ASV-msg:ValidGPSfix instead.")
  (ValidGPSfix m))

(cl:ensure-generic-function 'number_of_satellites-val :lambda-list '(m))
(cl:defmethod number_of_satellites-val ((m <position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:number_of_satellites-val is deprecated.  Use ASV-msg:number_of_satellites instead.")
  (number_of_satellites m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <position>) ostream)
  "Serializes a message object of type '<position>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'X))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'long))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'ValidGPSfix)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'number_of_satellites)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <position>) istream)
  "Deserializes a message object of type '<position>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'X) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lat) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'long) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ValidGPSfix) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'number_of_satellites) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<position>)))
  "Returns string type for a message object of type '<position>"
  "ASV/position")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'position)))
  "Returns string type for a message object of type 'position"
  "ASV/position")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<position>)))
  "Returns md5sum for a message object of type '<position>"
  "0ff27e09213a0df37db82ae00fb8aecd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'position)))
  "Returns md5sum for a message object of type 'position"
  "0ff27e09213a0df37db82ae00fb8aecd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<position>)))
  "Returns full string definition for message of type '<position>"
  (cl:format cl:nil "float32 X~%float32 Y~%float32 speed~%float64 lat~%float64 long~%float64 time~%int8 ValidGPSfix~%int8 number_of_satellites~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'position)))
  "Returns full string definition for message of type 'position"
  (cl:format cl:nil "float32 X~%float32 Y~%float32 speed~%float64 lat~%float64 long~%float64 time~%int8 ValidGPSfix~%int8 number_of_satellites~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <position>))
  (cl:+ 0
     4
     4
     4
     8
     8
     8
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <position>))
  "Converts a ROS message object to a list"
  (cl:list 'position
    (cl:cons ':X (X msg))
    (cl:cons ':Y (Y msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':lat (lat msg))
    (cl:cons ':long (long msg))
    (cl:cons ':time (time msg))
    (cl:cons ':ValidGPSfix (ValidGPSfix msg))
    (cl:cons ':number_of_satellites (number_of_satellites msg))
))
