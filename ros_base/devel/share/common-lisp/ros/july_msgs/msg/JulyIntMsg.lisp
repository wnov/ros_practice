; Auto-generated. Do not edit!


(cl:in-package july_msgs-msg)


;//! \htmlinclude JulyIntMsg.msg.html

(cl:defclass <JulyIntMsg> (roslisp-msg-protocol:ros-message)
  ((msg_a
    :reader msg_a
    :initarg :msg_a
    :type cl:integer
    :initform 0)
   (msg_b
    :reader msg_b
    :initarg :msg_b
    :type cl:integer
    :initform 0))
)

(cl:defclass JulyIntMsg (<JulyIntMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JulyIntMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JulyIntMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name july_msgs-msg:<JulyIntMsg> is deprecated: use july_msgs-msg:JulyIntMsg instead.")))

(cl:ensure-generic-function 'msg_a-val :lambda-list '(m))
(cl:defmethod msg_a-val ((m <JulyIntMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader july_msgs-msg:msg_a-val is deprecated.  Use july_msgs-msg:msg_a instead.")
  (msg_a m))

(cl:ensure-generic-function 'msg_b-val :lambda-list '(m))
(cl:defmethod msg_b-val ((m <JulyIntMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader july_msgs-msg:msg_b-val is deprecated.  Use july_msgs-msg:msg_b instead.")
  (msg_b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JulyIntMsg>) ostream)
  "Serializes a message object of type '<JulyIntMsg>"
  (cl:let* ((signed (cl:slot-value msg 'msg_a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'msg_b)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JulyIntMsg>) istream)
  "Deserializes a message object of type '<JulyIntMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg_a) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg_b) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JulyIntMsg>)))
  "Returns string type for a message object of type '<JulyIntMsg>"
  "july_msgs/JulyIntMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JulyIntMsg)))
  "Returns string type for a message object of type 'JulyIntMsg"
  "july_msgs/JulyIntMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JulyIntMsg>)))
  "Returns md5sum for a message object of type '<JulyIntMsg>"
  "d09a3f7a5a94e2671033e5fccab47e31")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JulyIntMsg)))
  "Returns md5sum for a message object of type 'JulyIntMsg"
  "d09a3f7a5a94e2671033e5fccab47e31")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JulyIntMsg>)))
  "Returns full string definition for message of type '<JulyIntMsg>"
  (cl:format cl:nil "int32 msg_a~%int32 msg_b~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JulyIntMsg)))
  "Returns full string definition for message of type 'JulyIntMsg"
  (cl:format cl:nil "int32 msg_a~%int32 msg_b~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JulyIntMsg>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JulyIntMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'JulyIntMsg
    (cl:cons ':msg_a (msg_a msg))
    (cl:cons ':msg_b (msg_b msg))
))
