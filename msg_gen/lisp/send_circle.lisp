; Auto-generated. Do not edit!


(cl:in-package CDIO-msg)


;//! \htmlinclude send_circle.msg.html

(cl:defclass <send_circle> (roslisp-msg-protocol:ros-message)
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
    :type cl:float
    :initform 0.0))
)

(cl:defclass send_circle (<send_circle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <send_circle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'send_circle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CDIO-msg:<send_circle> is deprecated: use CDIO-msg:send_circle instead.")))

(cl:ensure-generic-function 'centerX-val :lambda-list '(m))
(cl:defmethod centerX-val ((m <send_circle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CDIO-msg:centerX-val is deprecated.  Use CDIO-msg:centerX instead.")
  (centerX m))

(cl:ensure-generic-function 'centerY-val :lambda-list '(m))
(cl:defmethod centerY-val ((m <send_circle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CDIO-msg:centerY-val is deprecated.  Use CDIO-msg:centerY instead.")
  (centerY m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <send_circle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CDIO-msg:radius-val is deprecated.  Use CDIO-msg:radius instead.")
  (radius m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <send_circle>) ostream)
  "Serializes a message object of type '<send_circle>"
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
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <send_circle>) istream)
  "Deserializes a message object of type '<send_circle>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<send_circle>)))
  "Returns string type for a message object of type '<send_circle>"
  "CDIO/send_circle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'send_circle)))
  "Returns string type for a message object of type 'send_circle"
  "CDIO/send_circle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<send_circle>)))
  "Returns md5sum for a message object of type '<send_circle>"
  "34eb6927a044b5d4aa33e7f8dcea08d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'send_circle)))
  "Returns md5sum for a message object of type 'send_circle"
  "34eb6927a044b5d4aa33e7f8dcea08d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<send_circle>)))
  "Returns full string definition for message of type '<send_circle>"
  (cl:format cl:nil "float64 centerX~%float64 centerY~%float64 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'send_circle)))
  "Returns full string definition for message of type 'send_circle"
  (cl:format cl:nil "float64 centerX~%float64 centerY~%float64 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <send_circle>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <send_circle>))
  "Converts a ROS message object to a list"
  (cl:list 'send_circle
    (cl:cons ':centerX (centerX msg))
    (cl:cons ':centerY (centerY msg))
    (cl:cons ':radius (radius msg))
))
