
(cl:in-package :asdf)

(defsystem "reflex_one_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :reflex_one_msgs-msg
)
  :components ((:file "_package")
    (:file "SetSpeed" :depends-on ("_package_SetSpeed"))
    (:file "_package_SetSpeed" :depends-on ("_package"))
    (:file "SetTactileThreshold" :depends-on ("_package_SetTactileThreshold"))
    (:file "_package_SetTactileThreshold" :depends-on ("_package"))
  ))