
(cl:in-package :asdf)

(defsystem "aa274_s2-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MyMessage" :depends-on ("_package_MyMessage"))
    (:file "_package_MyMessage" :depends-on ("_package"))
    (:file "Section6msg" :depends-on ("_package_Section6msg"))
    (:file "_package_Section6msg" :depends-on ("_package"))
  ))