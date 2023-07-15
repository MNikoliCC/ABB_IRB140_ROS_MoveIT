; Auto-generated. Do not edit!


(cl:in-package planning-srv)


;//! \htmlinclude CameraMsg-request.msg.html

(cl:defclass <CameraMsg-request> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass CameraMsg-request (<CameraMsg-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraMsg-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraMsg-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planning-srv:<CameraMsg-request> is deprecated: use planning-srv:CameraMsg-request instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <CameraMsg-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planning-srv:angle-val is deprecated.  Use planning-srv:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraMsg-request>) ostream)
  "Serializes a message object of type '<CameraMsg-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraMsg-request>) istream)
  "Deserializes a message object of type '<CameraMsg-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraMsg-request>)))
  "Returns string type for a service object of type '<CameraMsg-request>"
  "planning/CameraMsgRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraMsg-request)))
  "Returns string type for a service object of type 'CameraMsg-request"
  "planning/CameraMsgRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraMsg-request>)))
  "Returns md5sum for a message object of type '<CameraMsg-request>"
  "e5775eef75e8eba6b10486d9b6b6d54e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraMsg-request)))
  "Returns md5sum for a message object of type 'CameraMsg-request"
  "e5775eef75e8eba6b10486d9b6b6d54e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraMsg-request>)))
  "Returns full string definition for message of type '<CameraMsg-request>"
  (cl:format cl:nil "float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraMsg-request)))
  "Returns full string definition for message of type 'CameraMsg-request"
  (cl:format cl:nil "float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraMsg-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraMsg-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraMsg-request
    (cl:cons ':angle (angle msg))
))
;//! \htmlinclude CameraMsg-response.msg.html

(cl:defclass <CameraMsg-response> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass CameraMsg-response (<CameraMsg-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraMsg-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraMsg-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planning-srv:<CameraMsg-response> is deprecated: use planning-srv:CameraMsg-response instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <CameraMsg-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planning-srv:image-val is deprecated.  Use planning-srv:image instead.")
  (image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraMsg-response>) ostream)
  "Serializes a message object of type '<CameraMsg-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraMsg-response>) istream)
  "Deserializes a message object of type '<CameraMsg-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraMsg-response>)))
  "Returns string type for a service object of type '<CameraMsg-response>"
  "planning/CameraMsgResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraMsg-response)))
  "Returns string type for a service object of type 'CameraMsg-response"
  "planning/CameraMsgResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraMsg-response>)))
  "Returns md5sum for a message object of type '<CameraMsg-response>"
  "e5775eef75e8eba6b10486d9b6b6d54e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraMsg-response)))
  "Returns md5sum for a message object of type 'CameraMsg-response"
  "e5775eef75e8eba6b10486d9b6b6d54e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraMsg-response>)))
  "Returns full string definition for message of type '<CameraMsg-response>"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraMsg-response)))
  "Returns full string definition for message of type 'CameraMsg-response"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraMsg-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraMsg-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraMsg-response
    (cl:cons ':image (image msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CameraMsg)))
  'CameraMsg-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CameraMsg)))
  'CameraMsg-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraMsg)))
  "Returns string type for a service object of type '<CameraMsg>"
  "planning/CameraMsg")