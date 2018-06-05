
(cl:in-package :asdf)

(defsystem "ros_gazebo_plugin_comm-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Vector7d" :depends-on ("_package_Vector7d"))
    (:file "_package_Vector7d" :depends-on ("_package"))
  ))