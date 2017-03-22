; Auto-generated. Do not edit!


(cl:in-package CDIO-srv)


;//! \htmlinclude Get_battery_srv-request.msg.html

(cl:defclass <Get_battery_srv-request> (roslisp-msg-protocol:ros-message)
  ((batteryPercent
    :reader batteryPercent
    :initarg :batteryPercent
    :type cl:float
    :initform 0.0))
)

(cl:defclass Get_battery_srv-request (<Get_battery_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Get_battery_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Get_battery_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CDIO-srv:<Get_battery_srv-request> is deprecated: use CDIO-srv:Get_battery_srv-request instead.")))

(cl:ensure-generic-function 'batteryPercent-val :lambda-list '(m))
(cl:defmethod batteryPercent-val ((m <Get_battery_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CDIO-srv:batteryPercent-val is deprecated.  Use CDIO-srv:batteryPercent instead.")
  (batteryPercent m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Get_battery_srv-request>) ostream)
  "Serializes a message object of type '<Get_battery_srv-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'batteryPercent))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Get_battery_srv-request>) istream)
  "Deserializes a message object of type '<Get_battery_srv-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'batteryPercent) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Get_battery_srv-request>)))
  "Returns string type for a service object of type '<Get_battery_srv-request>"
  "CDIO/Get_battery_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Get_battery_srv-request)))
  "Returns string type for a service object of type 'Get_battery_srv-request"
  "CDIO/Get_battery_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Get_battery_srv-request>)))
  "Returns md5sum for a message object of type '<Get_battery_srv-request>"
  "d9d979398f77f188497919a4fccf58f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Get_battery_srv-request)))
  "Returns md5sum for a message object of type 'Get_battery_srv-request"
  "d9d979398f77f188497919a4fccf58f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Get_battery_srv-request>)))
  "Returns full string definition for message of type '<Get_battery_srv-request>"
  (cl:format cl:nil "float32 batteryPercent~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Get_battery_srv-request)))
  "Returns full string definition for message of type 'Get_battery_srv-request"
  (cl:format cl:nil "float32 batteryPercent~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Get_battery_srv-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Get_battery_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Get_battery_srv-request
    (cl:cons ':batteryPercent (batteryPercent msg))
))
;//! \htmlinclude Get_battery_srv-response.msg.html

(cl:defclass <Get_battery_srv-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Get_battery_srv-response (<Get_battery_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Get_battery_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Get_battery_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CDIO-srv:<Get_battery_srv-response> is deprecated: use CDIO-srv:Get_battery_srv-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Get_battery_srv-response>) ostream)
  "Serializes a message object of type '<Get_battery_srv-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Get_battery_srv-response>) istream)
  "Deserializes a message object of type '<Get_battery_srv-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Get_battery_srv-response>)))
  "Returns string type for a service object of type '<Get_battery_srv-response>"
  "CDIO/Get_battery_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Get_battery_srv-response)))
  "Returns string type for a service object of type 'Get_battery_srv-response"
  "CDIO/Get_battery_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Get_battery_srv-response>)))
  "Returns md5sum for a message object of type '<Get_battery_srv-response>"
  "d9d979398f77f188497919a4fccf58f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Get_battery_srv-response)))
  "Returns md5sum for a message object of type 'Get_battery_srv-response"
  "d9d979398f77f188497919a4fccf58f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Get_battery_srv-response>)))
  "Returns full string definition for message of type '<Get_battery_srv-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Get_battery_srv-response)))
  "Returns full string definition for message of type 'Get_battery_srv-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Get_battery_srv-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Get_battery_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Get_battery_srv-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Get_battery_srv)))
  'Get_battery_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Get_battery_srv)))
  'Get_battery_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Get_battery_srv)))
  "Returns string type for a service object of type '<Get_battery_srv>"
  "CDIO/Get_battery_srv")