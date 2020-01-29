; Auto-generated. Do not edit!


(cl:in-package aa274_s2-msg)


;//! \htmlinclude Section6msg.msg.html

(cl:defclass <Section6msg> (roslisp-msg-protocol:ros-message)
  ((rho
    :reader rho
    :initarg :rho
    :type cl:float
    :initform 0.0)
   (delta
    :reader delta
    :initarg :delta
    :type cl:float
    :initform 0.0)
   (alpha
    :reader alpha
    :initarg :alpha
    :type cl:float
    :initform 0.0))
)

(cl:defclass Section6msg (<Section6msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Section6msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Section6msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aa274_s2-msg:<Section6msg> is deprecated: use aa274_s2-msg:Section6msg instead.")))

(cl:ensure-generic-function 'rho-val :lambda-list '(m))
(cl:defmethod rho-val ((m <Section6msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aa274_s2-msg:rho-val is deprecated.  Use aa274_s2-msg:rho instead.")
  (rho m))

(cl:ensure-generic-function 'delta-val :lambda-list '(m))
(cl:defmethod delta-val ((m <Section6msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aa274_s2-msg:delta-val is deprecated.  Use aa274_s2-msg:delta instead.")
  (delta m))

(cl:ensure-generic-function 'alpha-val :lambda-list '(m))
(cl:defmethod alpha-val ((m <Section6msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aa274_s2-msg:alpha-val is deprecated.  Use aa274_s2-msg:alpha instead.")
  (alpha m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Section6msg>) ostream)
  "Serializes a message object of type '<Section6msg>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rho))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'alpha))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Section6msg>) istream)
  "Deserializes a message object of type '<Section6msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rho) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'alpha) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Section6msg>)))
  "Returns string type for a message object of type '<Section6msg>"
  "aa274_s2/Section6msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Section6msg)))
  "Returns string type for a message object of type 'Section6msg"
  "aa274_s2/Section6msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Section6msg>)))
  "Returns md5sum for a message object of type '<Section6msg>"
  "745d32393bd07b7f1c1cd05adaf03d1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Section6msg)))
  "Returns md5sum for a message object of type 'Section6msg"
  "745d32393bd07b7f1c1cd05adaf03d1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Section6msg>)))
  "Returns full string definition for message of type '<Section6msg>"
  (cl:format cl:nil "float64 rho~%float64 delta~%float64 alpha~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Section6msg)))
  "Returns full string definition for message of type 'Section6msg"
  (cl:format cl:nil "float64 rho~%float64 delta~%float64 alpha~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Section6msg>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Section6msg>))
  "Converts a ROS message object to a list"
  (cl:list 'Section6msg
    (cl:cons ':rho (rho msg))
    (cl:cons ':delta (delta msg))
    (cl:cons ':alpha (alpha msg))
))
