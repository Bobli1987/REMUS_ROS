
(cl:in-package :asdf)

(defsystem "imm-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "euler" :depends-on ("_package_euler"))
    (:file "_package_euler" :depends-on ("_package"))
  ))