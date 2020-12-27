;; Auto-generated. Do not edit!


(when (boundp 'gps_simulator::gps_measurement)
  (if (not (find-package "GPS_SIMULATOR"))
    (make-package "GPS_SIMULATOR"))
  (shadow 'gps_measurement (find-package "GPS_SIMULATOR")))
(unless (find-package "GPS_SIMULATOR::GPS_MEASUREMENT")
  (make-package "GPS_SIMULATOR::GPS_MEASUREMENT"))

(in-package "ROS")
;;//! \htmlinclude gps_measurement.msg.html


(defclass gps_simulator::gps_measurement
  :super ros::object
  :slots (_x _z ))

(defmethod gps_simulator::gps_measurement
  (:init
   (&key
    ((:x __x) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _z (float __z))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get gps_simulator::gps_measurement :md5sum-) "3d990ebeae1ee5ba6990ba82cc74e4c5")
(setf (get gps_simulator::gps_measurement :datatype-) "gps_simulator/gps_measurement")
(setf (get gps_simulator::gps_measurement :definition-)
      "float32 x
float32 z

")



(provide :gps_simulator/gps_measurement "3d990ebeae1ee5ba6990ba82cc74e4c5")


