;; Auto-generated. Do not edit!


(when (boundp 'f1tenth_simulator::gpiowrite)
  (if (not (find-package "F1TENTH_SIMULATOR"))
    (make-package "F1TENTH_SIMULATOR"))
  (shadow 'gpiowrite (find-package "F1TENTH_SIMULATOR")))
(unless (find-package "F1TENTH_SIMULATOR::GPIOWRITE")
  (make-package "F1TENTH_SIMULATOR::GPIOWRITE"))

(in-package "ROS")
;;//! \htmlinclude gpiowrite.msg.html


(defclass f1tenth_simulator::gpiowrite
  :super ros::object
  :slots (_state _duration ))

(defmethod f1tenth_simulator::gpiowrite
  (:init
   (&key
    ((:state __state) nil)
    ((:duration __duration) 0)
    )
   (send-super :init)
   (setq _state __state)
   (setq _duration (round __duration))
   self)
  (:state
   (&optional (__state :null))
   (if (not (eq __state :null)) (setq _state __state)) _state)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; bool _state
    1
    ;; uint8 _duration
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _state
       (if _state (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _duration
       (write-byte _duration s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _state
     (setq _state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _duration
     (setq _duration (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get f1tenth_simulator::gpiowrite :md5sum-) "e48107ec9af92b61adf575e5d66914d0")
(setf (get f1tenth_simulator::gpiowrite :datatype-) "f1tenth_simulator/gpiowrite")
(setf (get f1tenth_simulator::gpiowrite :definition-)
      "# set the pin to high or low via true/false
bool state

# Optionally, provide duration for state. <= 0 will leave in state indefinitely.
uint8 duration

")



(provide :f1tenth_simulator/gpiowrite "e48107ec9af92b61adf575e5d66914d0")


