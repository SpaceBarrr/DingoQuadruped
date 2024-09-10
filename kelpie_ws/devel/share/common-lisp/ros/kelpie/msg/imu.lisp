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
    :initform (cl:make-instance 'kelpie-msg:xyz_float32))
   (gbias
    :reader gbias
    :initarg :gbias
    :type kelpie-msg:xyz_float32
    :initform (cl:make-instance 'kelpie-msg:xyz_float32))
   (grav
    :reader grav
    :initarg :grav
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

(cl:ensure-generic-function 'gbias-val :lambda-list '(m))
(cl:defmethod gbias-val ((m <imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kelpie-msg:gbias-val is deprecated.  Use kelpie-msg:gbias instead.")
  (gbias m))

(cl:ensure-generic-function 'grav-val :lambda-list '(m))
(cl:defmethod grav-val ((m <imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kelpie-msg:grav-val is deprecated.  Use kelpie-msg:grav instead.")
  (grav m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <imu>) ostream)
  "Serializes a message object of type '<imu>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'att) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acc) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gyro) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gbias) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'grav) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <imu>) istream)
  "Deserializes a message object of type '<imu>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'att) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acc) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gyro) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gbias) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'grav) istream)
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
  "ce43921e7b399d3ef9d38833a187cc81")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'imu)))
  "Returns md5sum for a message object of type 'imu"
  "ce43921e7b399d3ef9d38833a187cc81")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<imu>)))
  "Returns full string definition for message of type '<imu>"
  (cl:format cl:nil "att att~%xyz_float32 acc~%xyz_float32 gyro~%xyz_float32 gbias~%xyz_float32 grav~%~%================================================================================~%MSG: kelpie/att~%float32 roll~%float32 pitch~%float32 yaw~%~%================================================================================~%MSG: kelpie/xyz_float32~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'imu)))
  "Returns full string definition for message of type 'imu"
  (cl:format cl:nil "att att~%xyz_float32 acc~%xyz_float32 gyro~%xyz_float32 gbias~%xyz_float32 grav~%~%================================================================================~%MSG: kelpie/att~%float32 roll~%float32 pitch~%float32 yaw~%~%================================================================================~%MSG: kelpie/xyz_float32~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <imu>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'att))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acc))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gyro))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gbias))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'grav))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <imu>))
  "Converts a ROS message object to a list"
  (cl:list 'imu
    (cl:cons ':att (att msg))
    (cl:cons ':acc (acc msg))
    (cl:cons ':gyro (gyro msg))
    (cl:cons ':gbias (gbias msg))
    (cl:cons ':grav (grav msg))
))
