; Auto-generated. Do not edit!


(cl:in-package CDIO-msg)


;//! \htmlinclude circle_msg.msg.html

(cl:defclass <circle_msg> (roslisp-msg-protocol:ros-message)
  ((centerX
    :reader centerX
    :initarg :centerX
    :type cl:float
    :initform 0.0)
   (centerY
    :reader centerY
    :initarg :centerY
    :type cl:float
    :initform 0.0)
   (radius
    :reader radius
    :initarg :radius
    :type cl:integer
    :initform 0))
)

(cl:defclass circle_msg (<circle_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <circle_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'circle_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CDIO-msg:<circle_msg> is deprecated: use CDIO-msg:circle_msg instead.")))

(cl:ensure-generic-function 'centerX-val :lambda-list '(m))
(cl:defmethod centerX-val ((m <circle_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CDIO-msg:centerX-val is deprecated.  Use CDIO-msg:centerX instead.")
  (centerX m))

(cl:ensure-generic-function 'centerY-val :lambda-list '(m))
(cl:defmethod centerY-val ((m <circle_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CDIO-msg:centerY-val is deprecated.  Use CDIO-msg:centerY instead.")
  (centerY m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <circle_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CDIO-msg:radius-val is deprecated.  Use CDIO-msg:radius instead.")
  (radius m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <circle_msg>) ostream)
  "Serializes a message object of type '<circle_msg>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'centerX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'centerY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'radius)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <circle_msg>) istream)
  "Deserializes a message object of type '<circle_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'centerX) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'centerY) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'radius) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<circle_msg>)))
  "Returns string type for a message object of type '<circle_msg>"
  "CDIO/circle_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'circle_msg)))
  "Returns string type for a message object of type 'circle_msg"
  "CDIO/circle_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<circle_msg>)))
  "Returns md5sum for a message object of type '<circle_msg>"
  "0b999b3bc2ae701b533430a967d74b94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'circle_msg)))
  "Returns md5sum for a message object of type 'circle_msg"
  "0b999b3bc2ae701b533430a967d74b94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<circle_msg>)))
  "Returns full string definition for message of type '<circle_msg>"
  (cl:format cl:nil "float64 centerX~%float64 centerY~%int32 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'circle_msg)))
  "Returns full string definition for message of type 'circle_msg"
  (cl:format cl:nil "float64 centerX~%float64 centerY~%int32 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <circle_msg>))
  (cl:+ 0
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <circle_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'circle_msg
    (cl:cons ':centerX (centerX msg))
    (cl:cons ':centerY (centerY msg))
    (cl:cons ':radius (radius msg))
))
