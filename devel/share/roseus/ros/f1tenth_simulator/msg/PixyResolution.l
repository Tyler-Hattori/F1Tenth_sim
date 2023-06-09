;; Auto-generated. Do not edit!


(when (boundp 'f1tenth_simulator::PixyResolution)
  (if (not (find-package "F1TENTH_SIMULATOR"))
    (make-package "F1TENTH_SIMULATOR"))
  (shadow 'PixyResolution (find-package "F1TENTH_SIMULATOR")))
(unless (find-package "F1TENTH_SIMULATOR::PIXYRESOLUTION")
  (make-package "F1TENTH_SIMULATOR::PIXYRESOLUTION"))

(in-package "ROS")
;;//! \htmlinclude PixyResolution.msg.html


(defclass f1tenth_simulator::PixyResolution
  :super ros::object
  :slots (_width _height ))

(defmethod f1tenth_simulator::PixyResolution
  (:init
   (&key
    ((:width __width) 0)
    ((:height __height) 0)
    )
   (send-super :init)
   (setq _width (round __width))
   (setq _height (round __height))
   self)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:serialization-length
   ()
   (+
    ;; uint16 _width
    2
    ;; uint16 _height
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _width
       (write-word _width s)
     ;; uint16 _height
       (write-word _height s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _width
     (setq _width (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _height
     (setq _height (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get f1tenth_simulator::PixyResolution :md5sum-) "20cde1cc3b01e7f015e45dc31f8ca17d")
(setf (get f1tenth_simulator::PixyResolution :datatype-) "f1tenth_simulator/PixyResolution")
(setf (get f1tenth_simulator::PixyResolution :definition-)
      "uint16 width
uint16 height

")



(provide :f1tenth_simulator/PixyResolution "20cde1cc3b01e7f015e45dc31f8ca17d")


