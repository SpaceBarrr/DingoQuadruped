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
  :slots (_att _acc _gyro ))

(defmethod kelpie::imu
  (:init
   (&key
    ((:att __att) (instance kelpie::att :init))
    ((:acc __acc) (instance kelpie::xyz_float32 :init))
    ((:gyro __gyro) (instance kelpie::xyz_float32 :init))
    )
   (send-super :init)
   (setq _att __att)
   (setq _acc __acc)
   (setq _gyro __gyro)
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
  (:serialization-length
   ()
   (+
    ;; kelpie/att _att
    (send _att :serialization-length)
    ;; kelpie/xyz_float32 _acc
    (send _acc :serialization-length)
    ;; kelpie/xyz_float32 _gyro
    (send _gyro :serialization-length)
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
   ;;
   self)
  )

(setf (get kelpie::imu :md5sum-) "a3e3586bb29c60a67ccc92ece7869236")
(setf (get kelpie::imu :datatype-) "kelpie/imu")
(setf (get kelpie::imu :definition-)
      "att att
xyz_float32 acc
xyz_float32 gyro

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



(provide :kelpie/imu "a3e3586bb29c60a67ccc92ece7869236")


