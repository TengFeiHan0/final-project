; Auto-generated. Do not edit!


(cl:in-package SLAM-msg)


;//! \htmlinclude SFM_msg.msg.html

(cl:defclass <SFM_msg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (size
    :reader size
    :initarg :size
    :type cl:integer
    :initform 0)
   (P
    :reader P
    :initarg :P
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass SFM_msg (<SFM_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SFM_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SFM_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name SLAM-msg:<SFM_msg> is deprecated: use SLAM-msg:SFM_msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SFM_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SLAM-msg:header-val is deprecated.  Use SLAM-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <SFM_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SLAM-msg:size-val is deprecated.  Use SLAM-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'P-val :lambda-list '(m))
(cl:defmethod P-val ((m <SFM_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SLAM-msg:P-val is deprecated.  Use SLAM-msg:P instead.")
  (P m))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <SFM_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SLAM-msg:image-val is deprecated.  Use SLAM-msg:image instead.")
  (image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SFM_msg>) ostream)
  "Serializes a message object of type '<SFM_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'P))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'P))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SFM_msg>) istream)
  "Deserializes a message object of type '<SFM_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'P) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'P)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SFM_msg>)))
  "Returns string type for a message object of type '<SFM_msg>"
  "SLAM/SFM_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SFM_msg)))
  "Returns string type for a message object of type 'SFM_msg"
  "SLAM/SFM_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SFM_msg>)))
  "Returns md5sum for a message object of type '<SFM_msg>"
  "0f7da4217e07aa4196e821a466e25362")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SFM_msg)))
  "Returns md5sum for a message object of type 'SFM_msg"
  "0f7da4217e07aa4196e821a466e25362")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SFM_msg>)))
  "Returns full string definition for message of type '<SFM_msg>"
  (cl:format cl:nil "Header header~%uint32 size~%float32[] P~%sensor_msgs/Image image~%~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SFM_msg)))
  "Returns full string definition for message of type 'SFM_msg"
  (cl:format cl:nil "Header header~%uint32 size~%float32[] P~%sensor_msgs/Image image~%~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SFM_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'P) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SFM_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'SFM_msg
    (cl:cons ':header (header msg))
    (cl:cons ':size (size msg))
    (cl:cons ':P (P msg))
    (cl:cons ':image (image msg))
))
