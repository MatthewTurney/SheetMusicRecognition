; Auto-generated. Do not edit!


(cl:in-package project_pkg-msg)


;//! \htmlinclude Music.msg.html

(cl:defclass <Music> (roslisp-msg-protocol:ros-message)
  ((notes
    :reader notes
    :initarg :notes
    :type (cl:vector project_pkg-msg:Note)
   :initform (cl:make-array 0 :element-type 'project_pkg-msg:Note :initial-element (cl:make-instance 'project_pkg-msg:Note))))
)

(cl:defclass Music (<Music>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Music>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Music)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name project_pkg-msg:<Music> is deprecated: use project_pkg-msg:Music instead.")))

(cl:ensure-generic-function 'notes-val :lambda-list '(m))
(cl:defmethod notes-val ((m <Music>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader project_pkg-msg:notes-val is deprecated.  Use project_pkg-msg:notes instead.")
  (notes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Music>) ostream)
  "Serializes a message object of type '<Music>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'notes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'notes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Music>) istream)
  "Deserializes a message object of type '<Music>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'notes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'notes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'project_pkg-msg:Note))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Music>)))
  "Returns string type for a message object of type '<Music>"
  "project_pkg/Music")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Music)))
  "Returns string type for a message object of type 'Music"
  "project_pkg/Music")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Music>)))
  "Returns md5sum for a message object of type '<Music>"
  "336014d282bfb921a1299cf4a485af06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Music)))
  "Returns md5sum for a message object of type 'Music"
  "336014d282bfb921a1299cf4a485af06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Music>)))
  "Returns full string definition for message of type '<Music>"
  (cl:format cl:nil "project_pkg/Note[] notes~%~%================================================================================~%MSG: project_pkg/Note~%int32 key~%float32 duration~%float32 rest_before~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Music)))
  "Returns full string definition for message of type 'Music"
  (cl:format cl:nil "project_pkg/Note[] notes~%~%================================================================================~%MSG: project_pkg/Note~%int32 key~%float32 duration~%float32 rest_before~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Music>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'notes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Music>))
  "Converts a ROS message object to a list"
  (cl:list 'Music
    (cl:cons ':notes (notes msg))
))
