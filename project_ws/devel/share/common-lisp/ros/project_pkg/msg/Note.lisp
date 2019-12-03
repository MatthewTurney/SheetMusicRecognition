; Auto-generated. Do not edit!


(cl:in-package project_pkg-msg)


;//! \htmlinclude Note.msg.html

(cl:defclass <Note> (roslisp-msg-protocol:ros-message)
  ((key
    :reader key
    :initarg :key
    :type cl:integer
    :initform 0)
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0)
   (rest_before
    :reader rest_before
    :initarg :rest_before
    :type cl:float
    :initform 0.0))
)

(cl:defclass Note (<Note>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Note>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Note)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name project_pkg-msg:<Note> is deprecated: use project_pkg-msg:Note instead.")))

(cl:ensure-generic-function 'key-val :lambda-list '(m))
(cl:defmethod key-val ((m <Note>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader project_pkg-msg:key-val is deprecated.  Use project_pkg-msg:key instead.")
  (key m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <Note>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader project_pkg-msg:duration-val is deprecated.  Use project_pkg-msg:duration instead.")
  (duration m))

(cl:ensure-generic-function 'rest_before-val :lambda-list '(m))
(cl:defmethod rest_before-val ((m <Note>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader project_pkg-msg:rest_before-val is deprecated.  Use project_pkg-msg:rest_before instead.")
  (rest_before m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Note>) ostream)
  "Serializes a message object of type '<Note>"
  (cl:let* ((signed (cl:slot-value msg 'key)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rest_before))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Note>) istream)
  "Deserializes a message object of type '<Note>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'key) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rest_before) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Note>)))
  "Returns string type for a message object of type '<Note>"
  "project_pkg/Note")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Note)))
  "Returns string type for a message object of type 'Note"
  "project_pkg/Note")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Note>)))
  "Returns md5sum for a message object of type '<Note>"
  "ff5d283ef6af3a9b602a9758c35d7198")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Note)))
  "Returns md5sum for a message object of type 'Note"
  "ff5d283ef6af3a9b602a9758c35d7198")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Note>)))
  "Returns full string definition for message of type '<Note>"
  (cl:format cl:nil "int32 key~%float32 duration~%float32 rest_before~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Note)))
  "Returns full string definition for message of type 'Note"
  (cl:format cl:nil "int32 key~%float32 duration~%float32 rest_before~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Note>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Note>))
  "Converts a ROS message object to a list"
  (cl:list 'Note
    (cl:cons ':key (key msg))
    (cl:cons ':duration (duration msg))
    (cl:cons ':rest_before (rest_before msg))
))
