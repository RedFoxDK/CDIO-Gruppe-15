
(cl:in-package :asdf)

(defsystem "CDIO-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "send_qr" :depends-on ("_package_send_qr"))
    (:file "_package_send_qr" :depends-on ("_package"))
    (:file "send_circle" :depends-on ("_package_send_circle"))
    (:file "_package_send_circle" :depends-on ("_package"))
    (:file "circle_msg" :depends-on ("_package_circle_msg"))
    (:file "_package_circle_msg" :depends-on ("_package"))
    (:file "chapter2_msg1" :depends-on ("_package_chapter2_msg1"))
    (:file "_package_chapter2_msg1" :depends-on ("_package"))
  ))