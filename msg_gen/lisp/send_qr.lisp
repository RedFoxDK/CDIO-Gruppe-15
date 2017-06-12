; Auto-generated. Do not edit!


(cl:in-package CDIO-msg)


;//! \htmlinclude send_qr.msg.html

(cl:defclass <send_qr> (roslisp-msg-protocol:ros-message)
  ((qr_value
    :reader qr_value
    :initarg :qr_value
    :type cl:string
    :initform ""))
)

(cl:defclass send_qr (<send_qr>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <send_qr>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'send_qr)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CDIO-msg:<send_qr> is deprecated: use CDIO-msg:send_qr instead.")))

(cl:ensure-generic-function 'qr_value-val :lambda-list '(m))
(cl:defmethod qr_value-val ((m <send_qr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CDIO-msg:qr_value-val is deprecated.  Use CDIO-msg:qr_value instead.")
  (qr_value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <send_qr>) ostream)
  "Serializes a message object of type '<send_qr>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'qr_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'qr_value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <send_qr>) istream)
  "Deserializes a message object of type '<send_qr>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'qr_value) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'qr_value) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<send_qr>)))
  "Returns string type for a message object of type '<send_qr>"
  "CDIO/send_qr")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'send_qr)))
  "Returns string type for a message object of type 'send_qr"
  "CDIO/send_qr")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<send_qr>)))
  "Returns md5sum for a message object of type '<send_qr>"
  "758a1ea8398e60ee51c78ecf4c697c80")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'send_qr)))
  "Returns md5sum for a message object of type 'send_qr"
  "758a1ea8398e60ee51c78ecf4c697c80")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<send_qr>)))
  "Returns full string definition for message of type '<send_qr>"
  (cl:format cl:nil "string qr_value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'send_qr)))
  "Returns full string definition for message of type 'send_qr"
  (cl:format cl:nil "string qr_value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <send_qr>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'qr_value))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <send_qr>))
  "Converts a ROS message object to a list"
  (cl:list 'send_qr
    (cl:cons ':qr_value (qr_value msg))
))
