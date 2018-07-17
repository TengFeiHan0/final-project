
(cl:in-package :asdf)

(defsystem "SLAM-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SFM_msg" :depends-on ("_package_SFM_msg"))
    (:file "_package_SFM_msg" :depends-on ("_package"))
  ))