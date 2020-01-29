;; Auto-generated. Do not edit!


(when (boundp 'aa274_s2::MyMessage)
  (if (not (find-package "AA274_S2"))
    (make-package "AA274_S2"))
  (shadow 'MyMessage (find-package "AA274_S2")))
(unless (find-package "AA274_S2::MYMESSAGE")
  (make-package "AA274_S2::MYMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude MyMessage.msg.html


(defclass aa274_s2::MyMessage
  :super ros::object
  :slots (_text _flag _symbol _ratio _clock _period _para ))

(defmethod aa274_s2::MyMessage
  (:init
   (&key
    ((:text __text) "")
    ((:flag __flag) nil)
    ((:symbol __symbol) 0)
    ((:ratio __ratio) 0.0)
    ((:clock __clock) (instance ros::time :init))
    ((:period __period) (instance ros::time :init))
    ((:para __para) 0)
    )
   (send-super :init)
   (setq _text (string __text))
   (setq _flag __flag)
   (setq _symbol (round __symbol))
   (setq _ratio (float __ratio))
   (setq _clock __clock)
   (setq _period __period)
   (setq _para (round __para))
   self)
  (:text
   (&optional __text)
   (if __text (setq _text __text)) _text)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:symbol
   (&optional __symbol)
   (if __symbol (setq _symbol __symbol)) _symbol)
  (:ratio
   (&optional __ratio)
   (if __ratio (setq _ratio __ratio)) _ratio)
  (:clock
   (&optional __clock)
   (if __clock (setq _clock __clock)) _clock)
  (:period
   (&optional __period)
   (if __period (setq _period __period)) _period)
  (:para
   (&optional __para)
   (if __para (setq _para __para)) _para)
  (:serialization-length
   ()
   (+
    ;; string _text
    4 (length _text)
    ;; bool _flag
    1
    ;; char _symbol
    1
    ;; float64 _ratio
    8
    ;; time _clock
    8
    ;; duration _period
    8
    ;; int64 _para
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _text
       (write-long (length _text) s) (princ _text s)
     ;; bool _flag
       (if _flag (write-byte -1 s) (write-byte 0 s))
     ;; char _symbol
       (write-byte _symbol s)
     ;; float64 _ratio
       (sys::poke _ratio (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; time _clock
       (write-long (send _clock :sec) s) (write-long (send _clock :nsec) s)
     ;; duration _period
       (write-long (send _period :sec) s) (write-long (send _period :nsec) s)
     ;; int64 _para
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _para (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _para) (= (length (_para . bv)) 2)) ;; bignum
              (write-long (ash (elt (_para . bv) 0) 0) s)
              (write-long (ash (elt (_para . bv) 1) -1) s))
             ((and (class _para) (= (length (_para . bv)) 1)) ;; big1
              (write-long (elt (_para . bv) 0) s)
              (write-long (if (>= _para 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _para s)(write-long (if (>= _para 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _flag
     (setq _flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; char _symbol
     (setq _symbol (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _ratio
     (setq _ratio (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; time _clock
     (send _clock :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _clock :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; duration _period
     (send _period :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _period :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int64 _para
#+(or :alpha :irix6 :x86_64)
      (setf _para (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _para (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get aa274_s2::MyMessage :md5sum-) "239c79299df89155b13fc3feae022d2f")
(setf (get aa274_s2::MyMessage :datatype-) "aa274_s2/MyMessage")
(setf (get aa274_s2::MyMessage :definition-)
      "string text
bool flag
char symbol
float64 ratio
time clock
duration period
int64 para

")



(provide :aa274_s2/MyMessage "239c79299df89155b13fc3feae022d2f")


