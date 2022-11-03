;; Auto-generated. Do not edit!


(when (boundp 'july_msgs::JulyIntMsg)
  (if (not (find-package "JULY_MSGS"))
    (make-package "JULY_MSGS"))
  (shadow 'JulyIntMsg (find-package "JULY_MSGS")))
(unless (find-package "JULY_MSGS::JULYINTMSG")
  (make-package "JULY_MSGS::JULYINTMSG"))

(in-package "ROS")
;;//! \htmlinclude JulyIntMsg.msg.html


(defclass july_msgs::JulyIntMsg
  :super ros::object
  :slots (_msg_a _msg_b ))

(defmethod july_msgs::JulyIntMsg
  (:init
   (&key
    ((:msg_a __msg_a) 0)
    ((:msg_b __msg_b) 0)
    )
   (send-super :init)
   (setq _msg_a (round __msg_a))
   (setq _msg_b (round __msg_b))
   self)
  (:msg_a
   (&optional __msg_a)
   (if __msg_a (setq _msg_a __msg_a)) _msg_a)
  (:msg_b
   (&optional __msg_b)
   (if __msg_b (setq _msg_b __msg_b)) _msg_b)
  (:serialization-length
   ()
   (+
    ;; int32 _msg_a
    4
    ;; int32 _msg_b
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _msg_a
       (write-long _msg_a s)
     ;; int32 _msg_b
       (write-long _msg_b s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _msg_a
     (setq _msg_a (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _msg_b
     (setq _msg_b (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get july_msgs::JulyIntMsg :md5sum-) "d09a3f7a5a94e2671033e5fccab47e31")
(setf (get july_msgs::JulyIntMsg :datatype-) "july_msgs/JulyIntMsg")
(setf (get july_msgs::JulyIntMsg :definition-)
      "int32 msg_a
int32 msg_b
")



(provide :july_msgs/JulyIntMsg "d09a3f7a5a94e2671033e5fccab47e31")


