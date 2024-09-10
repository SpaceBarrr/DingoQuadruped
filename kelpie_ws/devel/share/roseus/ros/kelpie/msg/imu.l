;; Auto-generated. Do not edit!


(when (boundp 'kelpie::imu)
  (if (not (find-package "KELPIE"))
    (make-package "KELPIE"))
  (shadow 'imu (find-package "KELPIE")))
(unless (find-package "KELPIE::IMU")
  (make-package "KELPIE::IMU"))

(in-package "ROS")
;;//! \htmlinclude imu.msg.html


(defclass kelpie::imu
  :super ros::object
  :slots (_att _acc _gyro _gbias _grav ))

(defmethod kelpie::imu
  (:init
   (&key
    ((:att __att) (instance kelpie::att :init))
    ((:acc __acc) (instance kelpie::xyz_float32 :init))
    ((:gyro __gyro) (instance kelpie::xyz_float32 :init))
    ((:gbias __gbias) (instance kelpie::xyz_float32 :init))
    ((:grav __grav) (instance kelpie::xyz_float32 :init))
    )
   (send-super :init)
   (setq _att __att)
   (setq _acc __acc)
   (setq _gyro __gyro)
   (setq _gbias __gbias)
   (setq _grav __grav)
   self)
  (:att
   (&rest __att)
   (if (keywordp (car __att))
       (send* _att __att)
     (progn
       (if __att (setq _att (car __att)))
       _att)))
  (:acc
   (&rest __acc)
   (if (keywordp (car __acc))
       (send* _acc __acc)
     (progn
       (if __acc (setq _acc (car __acc)))
       _acc)))
  (:gyro
   (&rest __gyro)
   (if (keywordp (car __gyro))
       (send* _gyro __gyro)
     (progn
       (if __gyro (setq _gyro (car __gyro)))
       _gyro)))
  (:gbias
   (&rest __gbias)
   (if (keywordp (car __gbias))
       (send* _gbias __gbias)
     (progn
       (if __gbias (setq _gbias (car __gbias)))
       _gbias)))
  (:grav
   (&rest __grav)
   (if (keywordp (car __grav))
       (send* _grav __grav)
     (progn
       (if __grav (setq _grav (car __grav)))
       _grav)))
  (:serialization-length
   ()
   (+
    ;; kelpie/att _att
    (send _att :serialization-length)
    ;; kelpie/xyz_float32 _acc
    (send _acc :serialization-length)
    ;; kelpie/xyz_float32 _gyro
    (send _gyro :serialization-length)
    ;; kelpie/xyz_float32 _gbias
    (send _gbias :serialization-length)
    ;; kelpie/xyz_float32 _grav
    (send _grav :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kelpie/att _att
       (send _att :serialize s)
     ;; kelpie/xyz_float32 _acc
       (send _acc :serialize s)
     ;; kelpie/xyz_float32 _gyro
       (send _gyro :serialize s)
     ;; kelpie/xyz_float32 _gbias
       (send _gbias :serialize s)
     ;; kelpie/xyz_float32 _grav
       (send _grav :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kelpie/att _att
     (send _att :deserialize buf ptr-) (incf ptr- (send _att :serialization-length))
   ;; kelpie/xyz_float32 _acc
     (send _acc :deserialize buf ptr-) (incf ptr- (send _acc :serialization-length))
   ;; kelpie/xyz_float32 _gyro
     (send _gyro :deserialize buf ptr-) (incf ptr- (send _gyro :serialization-length))
   ;; kelpie/xyz_float32 _gbias
     (send _gbias :deserialize buf ptr-) (incf ptr- (send _gbias :serialization-length))
   ;; kelpie/xyz_float32 _grav
     (send _grav :deserialize buf ptr-) (incf ptr- (send _grav :serialization-length))
   ;;
   self)
  )

(setf (get kelpie::imu :md5sum-) "ce43921e7b399d3ef9d38833a187cc81")
(setf (get kelpie::imu :datatype-) "kelpie/imu")
(setf (get kelpie::imu :definition-)
      "att att
xyz_float32 acc
xyz_float32 gyro
xyz_float32 gbias
xyz_float32 grav

================================================================================
MSG: kelpie/att
float32 roll
float32 pitch
float32 yaw

================================================================================
MSG: kelpie/xyz_float32
float32 x
float32 y
float32 z

")



(provide :kelpie/imu "ce43921e7b399d3ef9d38833a187cc81")


