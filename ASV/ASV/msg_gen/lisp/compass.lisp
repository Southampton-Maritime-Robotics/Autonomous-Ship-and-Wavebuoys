; Auto-generated. Do not edit!


(cl:in-package ASV-msg)


;//! \htmlinclude compass.msg.html

(cl:defclass <compass> (roslisp-msg-protocol:ros-message)
  ((heading
    :reader heading
    :initarg :heading
    :type cl:float
    :initform 0.0)
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (pitch_der
    :reader pitch_der
    :initarg :pitch_der
    :type cl:float
    :initform 0.0)
   (temperature
    :reader temperature
    :initarg :temperature
    :type cl:float
    :initform 0.0)
   (depth
    :reader depth
    :initarg :depth
    :type cl:float
    :initform 0.0)
   (m
    :reader m
    :initarg :m
    :type cl:float
    :initform 0.0)
   (mx
    :reader mx
    :initarg :mx
    :type cl:float
    :initform 0.0)
   (my
    :reader my
    :initarg :my
    :type cl:float
    :initform 0.0)
   (mz
    :reader mz
    :initarg :mz
    :type cl:float
    :initform 0.0)
   (a
    :reader a
    :initarg :a
    :type cl:float
    :initform 0.0)
   (ax
    :reader ax
    :initarg :ax
    :type cl:float
    :initform 0.0)
   (ay
    :reader ay
    :initarg :ay
    :type cl:float
    :initform 0.0)
   (az
    :reader az
    :initarg :az
    :type cl:float
    :initform 0.0))
)

(cl:defclass compass (<compass>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <compass>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'compass)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ASV-msg:<compass> is deprecated: use ASV-msg:compass instead.")))

(cl:ensure-generic-function 'heading-val :lambda-list '(m))
(cl:defmethod heading-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:heading-val is deprecated.  Use ASV-msg:heading instead.")
  (heading m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:roll-val is deprecated.  Use ASV-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:pitch-val is deprecated.  Use ASV-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'pitch_der-val :lambda-list '(m))
(cl:defmethod pitch_der-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:pitch_der-val is deprecated.  Use ASV-msg:pitch_der instead.")
  (pitch_der m))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:temperature-val is deprecated.  Use ASV-msg:temperature instead.")
  (temperature m))

(cl:ensure-generic-function 'depth-val :lambda-list '(m))
(cl:defmethod depth-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:depth-val is deprecated.  Use ASV-msg:depth instead.")
  (depth m))

(cl:ensure-generic-function 'm-val :lambda-list '(m))
(cl:defmethod m-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:m-val is deprecated.  Use ASV-msg:m instead.")
  (m m))

(cl:ensure-generic-function 'mx-val :lambda-list '(m))
(cl:defmethod mx-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:mx-val is deprecated.  Use ASV-msg:mx instead.")
  (mx m))

(cl:ensure-generic-function 'my-val :lambda-list '(m))
(cl:defmethod my-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:my-val is deprecated.  Use ASV-msg:my instead.")
  (my m))

(cl:ensure-generic-function 'mz-val :lambda-list '(m))
(cl:defmethod mz-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:mz-val is deprecated.  Use ASV-msg:mz instead.")
  (mz m))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:a-val is deprecated.  Use ASV-msg:a instead.")
  (a m))

(cl:ensure-generic-function 'ax-val :lambda-list '(m))
(cl:defmethod ax-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:ax-val is deprecated.  Use ASV-msg:ax instead.")
  (ax m))

(cl:ensure-generic-function 'ay-val :lambda-list '(m))
(cl:defmethod ay-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:ay-val is deprecated.  Use ASV-msg:ay instead.")
  (ay m))

(cl:ensure-generic-function 'az-val :lambda-list '(m))
(cl:defmethod az-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ASV-msg:az-val is deprecated.  Use ASV-msg:az instead.")
  (az m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <compass>) ostream)
  "Serializes a message object of type '<compass>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'heading))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_der))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'my))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ay))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'az))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <compass>) istream)
  "Deserializes a message object of type '<compass>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_der) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'depth) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'my) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ax) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ay) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'az) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<compass>)))
  "Returns string type for a message object of type '<compass>"
  "ASV/compass")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'compass)))
  "Returns string type for a message object of type 'compass"
  "ASV/compass")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<compass>)))
  "Returns md5sum for a message object of type '<compass>"
  "88afbcfc5596f414ffeacd3d60dc1ebd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'compass)))
  "Returns md5sum for a message object of type 'compass"
  "88afbcfc5596f414ffeacd3d60dc1ebd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<compass>)))
  "Returns full string definition for message of type '<compass>"
  (cl:format cl:nil "float32 heading~%float32 roll~%float32 pitch~%float32 pitch_der~%float32 temperature~%float32 depth        ~%float32 m~%float32 mx~%float32 my~%float32 mz~%float32 a~%float32 ax~%float32 ay~%float32 az~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'compass)))
  "Returns full string definition for message of type 'compass"
  (cl:format cl:nil "float32 heading~%float32 roll~%float32 pitch~%float32 pitch_der~%float32 temperature~%float32 depth        ~%float32 m~%float32 mx~%float32 my~%float32 mz~%float32 a~%float32 ax~%float32 ay~%float32 az~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <compass>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <compass>))
  "Converts a ROS message object to a list"
  (cl:list 'compass
    (cl:cons ':heading (heading msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':pitch_der (pitch_der msg))
    (cl:cons ':temperature (temperature msg))
    (cl:cons ':depth (depth msg))
    (cl:cons ':m (m msg))
    (cl:cons ':mx (mx msg))
    (cl:cons ':my (my msg))
    (cl:cons ':mz (mz msg))
    (cl:cons ':a (a msg))
    (cl:cons ':ax (ax msg))
    (cl:cons ':ay (ay msg))
    (cl:cons ':az (az msg))
))
