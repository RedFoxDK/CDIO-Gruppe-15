
(cl:in-package :asdf)

(defsystem "CDIO-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "chapter2_msg1" :depends-on ("_package_chapter2_msg1"))
    (:file "_package_chapter2_msg1" :depends-on ("_package"))
  ))