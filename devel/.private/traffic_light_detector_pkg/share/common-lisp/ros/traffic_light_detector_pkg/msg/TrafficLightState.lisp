; Auto-generated. Do not edit!


(cl:in-package traffic_light_detector_pkg-msg)


;//! \htmlinclude TrafficLightState.msg.html

(cl:defclass <TrafficLightState> (roslisp-msg-protocol:ros-message)
  ((color
    :reader color
    :initarg :color
    :type cl:string
    :initform ""))
)

(cl:defclass TrafficLightState (<TrafficLightState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrafficLightState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrafficLightState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traffic_light_detector_pkg-msg:<TrafficLightState> is deprecated: use traffic_light_detector_pkg-msg:TrafficLightState instead.")))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <TrafficLightState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traffic_light_detector_pkg-msg:color-val is deprecated.  Use traffic_light_detector_pkg-msg:color instead.")
  (color m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrafficLightState>) ostream)
  "Serializes a message object of type '<TrafficLightState>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'color))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'color))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrafficLightState>) istream)
  "Deserializes a message object of type '<TrafficLightState>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'color) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'color) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrafficLightState>)))
  "Returns string type for a message object of type '<TrafficLightState>"
  "traffic_light_detector_pkg/TrafficLightState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrafficLightState)))
  "Returns string type for a message object of type 'TrafficLightState"
  "traffic_light_detector_pkg/TrafficLightState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrafficLightState>)))
  "Returns md5sum for a message object of type '<TrafficLightState>"
  "89e44dcab627a2c43a70ae1100695caa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrafficLightState)))
  "Returns md5sum for a message object of type 'TrafficLightState"
  "89e44dcab627a2c43a70ae1100695caa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrafficLightState>)))
  "Returns full string definition for message of type '<TrafficLightState>"
  (cl:format cl:nil "string color~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrafficLightState)))
  "Returns full string definition for message of type 'TrafficLightState"
  (cl:format cl:nil "string color~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrafficLightState>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'color))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrafficLightState>))
  "Converts a ROS message object to a list"
  (cl:list 'TrafficLightState
    (cl:cons ':color (color msg))
))
