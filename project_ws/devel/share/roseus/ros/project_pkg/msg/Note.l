;; Auto-generated. Do not edit!


(when (boundp 'project_pkg::Note)
  (if (not (find-package "PROJECT_PKG"))
    (make-package "PROJECT_PKG"))
  (shadow 'Note (find-package "PROJECT_PKG")))
(unless (find-package "PROJECT_PKG::NOTE")
  (make-package "PROJECT_PKG::NOTE"))

(in-package "ROS")
;;//! \htmlinclude Note.msg.html


(defclass project_pkg::Note
  :super ros::object
  :slots (_key _duration _rest_before ))

(defmethod project_pkg::Note
  (:init
   (&key
    ((:key __key) 0)
    ((:duration __duration) 0.0)
    ((:rest_before __rest_before) 0.0)
    )
   (send-super :init)
   (setq _key (round __key))
   (setq _duration (float __duration))
   (setq _rest_before (float __rest_before))
   self)
  (:key
   (&optional __key)
   (if __key (setq _key __key)) _key)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:rest_before
   (&optional __rest_before)
   (if __rest_before (setq _rest_before __rest_before)) _rest_before)
  (:serialization-length
   ()
   (+
    ;; int32 _key
    4
    ;; float32 _duration
    4
    ;; float32 _rest_before
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _key
       (write-long _key s)
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rest_before
       (sys::poke _rest_before (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _key
     (setq _key (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rest_before
     (setq _rest_before (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get project_pkg::Note :md5sum-) "ff5d283ef6af3a9b602a9758c35d7198")
(setf (get project_pkg::Note :datatype-) "project_pkg/Note")
(setf (get project_pkg::Note :definition-)
      "int32 key
float32 duration
float32 rest_before

")



(provide :project_pkg/Note "ff5d283ef6af3a9b602a9758c35d7198")


