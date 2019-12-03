
(cl:in-package :asdf)

(defsystem "project_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Music" :depends-on ("_package_Music"))
    (:file "_package_Music" :depends-on ("_package"))
    (:file "Note" :depends-on ("_package_Note"))
    (:file "_package_Note" :depends-on ("_package"))
  ))