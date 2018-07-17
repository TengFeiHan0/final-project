
(cl:in-package :asdf)

(defsystem "rec_gui-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Cam_POS" :depends-on ("_package_Cam_POS"))
    (:file "_package_Cam_POS" :depends-on ("_package"))
  ))