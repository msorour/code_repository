
(cl:in-package :asdf)

(defsystem "reflex_msgs2-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :reflex_msgs2-msg
)
  :components ((:file "_package")
    (:file "DistalRotation" :depends-on ("_package_DistalRotation"))
    (:file "_package_DistalRotation" :depends-on ("_package"))
    (:file "SetSpeed" :depends-on ("_package_SetSpeed"))
    (:file "_package_SetSpeed" :depends-on ("_package"))
    (:file "SetTactileThreshold" :depends-on ("_package_SetTactileThreshold"))
    (:file "_package_SetTactileThreshold" :depends-on ("_package"))
  ))