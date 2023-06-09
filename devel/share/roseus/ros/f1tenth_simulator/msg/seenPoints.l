;; Auto-generated. Do not edit!


(when (boundp 'f1tenth_simulator::seenPoints)
  (if (not (find-package "F1TENTH_SIMULATOR"))
    (make-package "F1TENTH_SIMULATOR"))
  (shadow 'seenPoints (find-package "F1TENTH_SIMULATOR")))
(unless (find-package "F1TENTH_SIMULATOR::SEENPOINTS")
  (make-package "F1TENTH_SIMULATOR::SEENPOINTS"))

(in-package "ROS")
;;//! \htmlinclude seenPoints.msg.html


(defclass f1tenth_simulator::seenPoints
  :super ros::object
  :slots (_indices _confidences ))

(defmethod f1tenth_simulator::seenPoints
  (:init
   (&key
    ((:indices __indices) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:confidences __confidences) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _indices __indices)
   (setq _confidences __confidences)
   self)
  (:indices
   (&optional __indices)
   (if __indices (setq _indices __indices)) _indices)
  (:confidences
   (&optional __confidences)
   (if __confidences (setq _confidences __confidences)) _confidences)
  (:serialization-length
   ()
   (+
    ;; float64[] _indices
    (* 8    (length _indices)) 4
    ;; float64[] _confidences
    (* 8    (length _confidences)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _indices
     (write-long (length _indices) s)
     (dotimes (i (length _indices))
       (sys::poke (elt _indices i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _confidences
     (write-long (length _confidences) s)
     (dotimes (i (length _confidences))
       (sys::poke (elt _confidences i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _indices
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _indices (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _indices i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _confidences
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _confidences (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _confidences i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get f1tenth_simulator::seenPoints :md5sum-) "a1650ba81512ff9c6b2cf12c61879bc5")
(setf (get f1tenth_simulator::seenPoints :datatype-) "f1tenth_simulator/seenPoints")
(setf (get f1tenth_simulator::seenPoints :definition-)
      "float64[] indices
float64[] confidences

")



(provide :f1tenth_simulator/seenPoints "a1650ba81512ff9c6b2cf12c61879bc5")


