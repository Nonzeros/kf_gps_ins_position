
(cl:in-package :asdf)

(defsystem "server_client-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "addInts" :depends-on ("_package_addInts"))
    (:file "_package_addInts" :depends-on ("_package"))
  ))