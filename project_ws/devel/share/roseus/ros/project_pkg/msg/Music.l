;; Auto-generated. Do not edit!


(when (boundp 'project_pkg::Music)
  (if (not (find-package "PROJECT_PKG"))
    (make-package "PROJECT_PKG"))
  (shadow 'Music (find-package "PROJECT_PKG")))
(unless (find-package "PROJECT_PKG::MUSIC")
  (make-package "PROJECT_PKG::MUSIC"))

(in-package "ROS")
;;//! \htmlinclude Music.msg.html


(defclass project_pkg::Music
  :super ros::object
  :slots (_notes ))

(defmethod project_pkg::Music
  (:init
   (&key
    ((:notes __notes) (let (r) (dotimes (i 0) (push (instance project_pkg::Note :init) r)) r))
    )
   (send-super :init)
   (setq _notes __notes)
   self)
  (:notes
   (&rest __notes)
   (if (keywordp (car __notes))
       (send* _notes __notes)
     (progn
       (if __notes (setq _notes (car __notes)))
       _notes)))
  (:serialization-length
   ()
   (+
    ;; project_pkg/Note[] _notes
    (apply #'+ (send-all _notes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; project_pkg/Note[] _notes
     (write-long (length _notes) s)
     (dolist (elem _notes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; project_pkg/Note[] _notes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _notes (let (r) (dotimes (i n) (push (instance project_pkg::Note :init) r)) r))
     (dolist (elem- _notes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get project_pkg::Music :md5sum-) "336014d282bfb921a1299cf4a485af06")
(setf (get project_pkg::Music :datatype-) "project_pkg/Music")
(setf (get project_pkg::Music :definition-)
      "project_pkg/Note[] notes

================================================================================
MSG: project_pkg/Note
int32 key
float32 duration
float32 rest_before

")



(provide :project_pkg/Music "336014d282bfb921a1299cf4a485af06")


