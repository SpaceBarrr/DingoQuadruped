; Auto-generated. Do not edit!


(cl:in-package kelpie-msg)


;//! \htmlinclude imu.msg.html

(cl:defclass <imu> (roslisp-msg-protocol:ros-message)
  ((att
    :reader att
    :initarg :att
    :type kelpie-msg:att
    :initform (cl:make-instance 'kelpie-msg:att))
   (acc
    :reader acc
    :initarg :acc
    :type kelpie-msg:xyz_float32
    :initform (cl:make-instance 'kelpie-msg:xyz_float32))
   (gyro
    :reader gyro
    :initarg :gyro
    :type kelpie-msg:xyz_float32
    :initform (cl:make-instance 'kelpie-msg:xyz_float32)))
)

(cl:defclass imu (<imu>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <imu>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'imu)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kelpie-msg:<imu> is deprecated: use kelpie-msg:imu instead.")))

(cl:ensure-generic-function 'att-val :lambda-list '(m))
(cl:defmethod att-val ((m <imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kelpie-msg:att-val is deprecated.  Use kelpie-msg:att instead.")
  (att m))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kelpie-msg:acc-val is deprecated.  Use kelpie-msg:acc instead.")
  (acc m))

(cl:ensure-generic-function 'gyro-val :lambda-list '(m))
(cl:defmethod gyro-val ((m <imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kelpie-msg:gyro-val is deprecated.  Use kelpie-msg:gyro instead.")
  (gyro m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <imu>) ostream)
  "Serializes a message object of type '<imu>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'att) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acc) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gyro) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <imu>) istream)
  "Deserializes a message object of type '<imu>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'att) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acc) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gyro) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<imu>)))
  "Returns string type for a message object of type '<imu>"
  "kelpie/imu")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'imu)))
  "Returns string type for a message object of type 'imu"
  "kelpie/imu")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<imu>)))
  "Returns md5sum for a message object of type '<imu>"
  "a3e3586bb29c60a67ccc92ece7869236")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'imu)))
  "Returns md5sum for a message object of type 'imu"
  "a3e3586bb29c60a67ccc92ece7869236")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<imu>)))
  "Returns full string definition for message of type '<imu>"
  (cl:format cl:nil "att att~%xyz_float32 acc~%xyz_float32 gyro~%~%================================================================================~%MSG: kelpie/att~%float32 roll~%float32 pitch~%float32 yaw~%~%================================================================================~%MSG: kelpie/xyz_float32~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'imu)))
  "Returns full string definition for message of type 'imu"
  (cl:format cl:nil "att att~%xyz_float32 acc~%xyz_float32 gyro~%~%================================================================================~%MSG: kelpie/att~%float32 roll~%float32 pitch~%float32 yaw~%~%================================================================================~%MSG: kelpie/xyz_float32~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <imu>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'att))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acc))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gyro))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <imu>))
  "Converts a ROS message object to a list"
  (cl:list 'imu
    (cl:cons ':att (att msg))
    (cl:cons ':acc (acc msg))
    (cl:cons ':gyro (gyro msg))
))
