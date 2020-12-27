
(cl:in-package :asdf)

(defsystem "gps_simulator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "gps_measurement" :depends-on ("_package_gps_measurement"))
    (:file "_package_gps_measurement" :depends-on ("_package"))
  ))