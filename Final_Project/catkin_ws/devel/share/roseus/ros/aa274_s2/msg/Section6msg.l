;; Auto-generated. Do not edit!


(when (boundp 'aa274_s2::Section6msg)
  (if (not (find-package "AA274_S2"))
    (make-package "AA274_S2"))
  (shadow 'Section6msg (find-package "AA274_S2")))
(unless (find-package "AA274_S2::SECTION6MSG")
  (make-package "AA274_S2::SECTION6MSG"))

(in-package "ROS")
;;//! \htmlinclude Section6msg.msg.html


(defclass aa274_s2::Section6msg
  :super ros::object
  :slots (_rho _delta _alpha ))

(defmethod aa274_s2::Section6msg
  (:init
   (&key
    ((:rho __rho) 0.0)
    ((:delta __delta) 0.0)
    ((:alpha __alpha) 0.0)
    )
   (send-super :init)
   (setq _rho (float __rho))
   (setq _delta (float __delta))
   (setq _alpha (float __alpha))
   self)
  (:rho
   (&optional __rho)
   (if __rho (setq _rho __rho)) _rho)
  (:delta
   (&optional __delta)
   (if __delta (setq _delta __delta)) _delta)
  (:alpha
   (&optional __alpha)
   (if __alpha (setq _alpha __alpha)) _alpha)
  (:serialization-length
   ()
   (+
    ;; float64 _rho
    8
    ;; float64 _delta
    8
    ;; float64 _alpha
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _rho
       (sys::poke _rho (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta
       (sys::poke _delta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _alpha
       (sys::poke _alpha (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _rho
     (setq _rho (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta
     (setq _delta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _alpha
     (setq _alpha (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get aa274_s2::Section6msg :md5sum-) "745d32393bd07b7f1c1cd05adaf03d1e")
(setf (get aa274_s2::Section6msg :datatype-) "aa274_s2/Section6msg")
(setf (get aa274_s2::Section6msg :definition-)
      "float64 rho
float64 delta
float64 alpha


")



(provide :aa274_s2/Section6msg "745d32393bd07b7f1c1cd05adaf03d1e")


