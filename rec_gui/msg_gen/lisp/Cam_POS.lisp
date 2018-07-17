; Auto-generated. Do not edit!


(cl:in-package rec_gui-msg)


;//! \htmlinclude Cam_POS.msg.html

(cl:defclass <Cam_POS> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (Cam_x
    :reader Cam_x
    :initarg :Cam_x
    :type cl:float
    :initform 0.0)
   (Cam_y
    :reader Cam_y
    :initarg :Cam_y
    :type cl:float
    :initform 0.0)
   (Cam_z
    :reader Cam_z
    :initarg :Cam_z
    :type cl:float
    :initform 0.0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0))
)

(cl:defclass Cam_POS (<Cam_POS>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Cam_POS>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Cam_POS)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rec_gui-msg:<Cam_POS> is deprecated: use rec_gui-msg:Cam_POS instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Cam_POS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rec_gui-msg:status-val is deprecated.  Use rec_gui-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'Cam_x-val :lambda-list '(m))
(cl:defmethod Cam_x-val ((m <Cam_POS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rec_gui-msg:Cam_x-val is deprecated.  Use rec_gui-msg:Cam_x instead.")
  (Cam_x m))

(cl:ensure-generic-function 'Cam_y-val :lambda-list '(m))
(cl:defmethod Cam_y-val ((m <Cam_POS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rec_gui-msg:Cam_y-val is deprecated.  Use rec_gui-msg:Cam_y instead.")
  (Cam_y m))

(cl:ensure-generic-function 'Cam_z-val :lambda-list '(m))
(cl:defmethod Cam_z-val ((m <Cam_POS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rec_gui-msg:Cam_z-val is deprecated.  Use rec_gui-msg:Cam_z instead.")
  (Cam_z m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <Cam_POS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rec_gui-msg:theta-val is deprecated.  Use rec_gui-msg:theta instead.")
  (theta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Cam_POS>) ostream)
  "Serializes a message object of type '<Cam_POS>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Cam_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Cam_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Cam_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Cam_POS>) istream)
  "Deserializes a message object of type '<Cam_POS>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Cam_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Cam_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Cam_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Cam_POS>)))
  "Returns string type for a message object of type '<Cam_POS>"
  "rec_gui/Cam_POS")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Cam_POS)))
  "Returns string type for a message object of type 'Cam_POS"
  "rec_gui/Cam_POS")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Cam_POS>)))
  "Returns md5sum for a message object of type '<Cam_POS>"
  "8eacc8191fac19b5c7149b1f76464ccd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Cam_POS)))
  "Returns md5sum for a message object of type 'Cam_POS"
  "8eacc8191fac19b5c7149b1f76464ccd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Cam_POS>)))
  "Returns full string definition for message of type '<Cam_POS>"
  (cl:format cl:nil "uint8 status~%float32 Cam_x~%float32 Cam_y~%float32 Cam_z~%float32 theta~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Cam_POS)))
  "Returns full string definition for message of type 'Cam_POS"
  (cl:format cl:nil "uint8 status~%float32 Cam_x~%float32 Cam_y~%float32 Cam_z~%float32 theta~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Cam_POS>))
  (cl:+ 0
     1
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Cam_POS>))
  "Converts a ROS message object to a list"
  (cl:list 'Cam_POS
    (cl:cons ':status (status msg))
    (cl:cons ':Cam_x (Cam_x msg))
    (cl:cons ':Cam_y (Cam_y msg))
    (cl:cons ':Cam_z (Cam_z msg))
    (cl:cons ':theta (theta msg))
))
