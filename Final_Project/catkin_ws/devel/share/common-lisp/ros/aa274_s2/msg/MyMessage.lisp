; Auto-generated. Do not edit!


(cl:in-package aa274_s2-msg)


;//! \htmlinclude MyMessage.msg.html

(cl:defclass <MyMessage> (roslisp-msg-protocol:ros-message)
  ((text
    :reader text
    :initarg :text
    :type cl:string
    :initform "")
   (flag
    :reader flag
    :initarg :flag
    :type cl:boolean
    :initform cl:nil)
   (symbol
    :reader symbol
    :initarg :symbol
    :type cl:integer
    :initform 0)
   (ratio
    :reader ratio
    :initarg :ratio
    :type cl:float
    :initform 0.0)
   (clock
    :reader clock
    :initarg :clock
    :type cl:real
    :initform 0)
   (period
    :reader period
    :initarg :period
    :type cl:real
    :initform 0)
   (para
    :reader para
    :initarg :para
    :type cl:integer
    :initform 0))
)

(cl:defclass MyMessage (<MyMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MyMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MyMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aa274_s2-msg:<MyMessage> is deprecated: use aa274_s2-msg:MyMessage instead.")))

(cl:ensure-generic-function 'text-val :lambda-list '(m))
(cl:defmethod text-val ((m <MyMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aa274_s2-msg:text-val is deprecated.  Use aa274_s2-msg:text instead.")
  (text m))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <MyMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aa274_s2-msg:flag-val is deprecated.  Use aa274_s2-msg:flag instead.")
  (flag m))

(cl:ensure-generic-function 'symbol-val :lambda-list '(m))
(cl:defmethod symbol-val ((m <MyMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aa274_s2-msg:symbol-val is deprecated.  Use aa274_s2-msg:symbol instead.")
  (symbol m))

(cl:ensure-generic-function 'ratio-val :lambda-list '(m))
(cl:defmethod ratio-val ((m <MyMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aa274_s2-msg:ratio-val is deprecated.  Use aa274_s2-msg:ratio instead.")
  (ratio m))

(cl:ensure-generic-function 'clock-val :lambda-list '(m))
(cl:defmethod clock-val ((m <MyMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aa274_s2-msg:clock-val is deprecated.  Use aa274_s2-msg:clock instead.")
  (clock m))

(cl:ensure-generic-function 'period-val :lambda-list '(m))
(cl:defmethod period-val ((m <MyMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aa274_s2-msg:period-val is deprecated.  Use aa274_s2-msg:period instead.")
  (period m))

(cl:ensure-generic-function 'para-val :lambda-list '(m))
(cl:defmethod para-val ((m <MyMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aa274_s2-msg:para-val is deprecated.  Use aa274_s2-msg:para instead.")
  (para m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MyMessage>) ostream)
  "Serializes a message object of type '<MyMessage>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'text))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'symbol)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'clock)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'clock) (cl:floor (cl:slot-value msg 'clock)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'period)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'period) (cl:floor (cl:slot-value msg 'period)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let* ((signed (cl:slot-value msg 'para)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MyMessage>) istream)
  "Deserializes a message object of type '<MyMessage>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'symbol)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ratio) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'clock) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'period) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'para) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MyMessage>)))
  "Returns string type for a message object of type '<MyMessage>"
  "aa274_s2/MyMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MyMessage)))
  "Returns string type for a message object of type 'MyMessage"
  "aa274_s2/MyMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MyMessage>)))
  "Returns md5sum for a message object of type '<MyMessage>"
  "239c79299df89155b13fc3feae022d2f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MyMessage)))
  "Returns md5sum for a message object of type 'MyMessage"
  "239c79299df89155b13fc3feae022d2f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MyMessage>)))
  "Returns full string definition for message of type '<MyMessage>"
  (cl:format cl:nil "string text~%bool flag~%char symbol~%float64 ratio~%time clock~%duration period~%int64 para~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MyMessage)))
  "Returns full string definition for message of type 'MyMessage"
  (cl:format cl:nil "string text~%bool flag~%char symbol~%float64 ratio~%time clock~%duration period~%int64 para~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MyMessage>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'text))
     1
     1
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MyMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'MyMessage
    (cl:cons ':text (text msg))
    (cl:cons ':flag (flag msg))
    (cl:cons ':symbol (symbol msg))
    (cl:cons ':ratio (ratio msg))
    (cl:cons ':clock (clock msg))
    (cl:cons ':period (period msg))
    (cl:cons ':para (para msg))
))
