; Auto-generated. Do not edit!


(cl:in-package CDIO-msg)


;//! \htmlinclude circle.msg.html

(cl:defclass <circle> (roslisp-msg-protocol:ros-message)
  ((centerX
    :reader centerX
    :initarg :centerX
    :type cl:float
    :initform 0.0)
   (centerY
    :reader centerY
    :initarg :centerY
    :type cl:float
    :initform 0.0))
)

(cl:defclass circle (<circle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <circle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'circle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CDIO-msg:<circle> is deprecated: use CDIO-msg:circle instead.")))

(cl:ensure-generic-function 'centerX-val :lambda-list '(m))
(cl:defmethod centerX-val ((m <circle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CDIO-msg:centerX-val is deprecated.  Use CDIO-msg:centerX instead.")
  (centerX m))

(cl:ensure-generic-function 'centerY-val :lambda-list '(m))
(cl:defmethod centerY-val ((m <circle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CDIO-msg:centerY-val is deprecated.  Use CDIO-msg:centerY instead.")
  (centerY m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <circle>) ostream)
  "Serializes a message object of type '<circle>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <circle>) istream)
  "Deserializes a message object of type '<circle>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<circle>)))
  "Returns string type for a message object of type '<circle>"
  "CDIO/circle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'circle)))
  "Returns string type for a message object of type 'circle"
  "CDIO/circle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<circle>)))
  "Returns md5sum for a message object of type '<circle>"
  "b28d7e6fe6bf4c2cda9d06343dd83f7c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'circle)))
  "Returns md5sum for a message object of type 'circle"
  "b28d7e6fe6bf4c2cda9d06343dd83f7c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<circle>)))
  "Returns full string definition for message of type '<circle>"
  (cl:format cl:nil "float64 centerX~%float64 centerY~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'circle)))
  "Returns full string definition for message of type 'circle"
  (cl:format cl:nil "float64 centerX~%float64 centerY~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <circle>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <circle>))
  "Converts a ROS message object to a list"
  (cl:list 'circle
    (cl:cons ':centerX (centerX msg))
    (cl:cons ':centerY (centerY msg))
))
