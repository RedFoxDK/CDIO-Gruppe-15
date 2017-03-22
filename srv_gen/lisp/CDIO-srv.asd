
(cl:in-package :asdf)

(defsystem "CDIO-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "chapter2_srv1" :depends-on ("_package_chapter2_srv1"))
    (:file "_package_chapter2_srv1" :depends-on ("_package"))
    (:file "Get_battery_srv" :depends-on ("_package_Get_battery_srv"))
    (:file "_package_Get_battery_srv" :depends-on ("_package"))
  ))