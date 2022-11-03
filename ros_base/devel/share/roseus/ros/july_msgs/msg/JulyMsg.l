;; Auto-generated. Do not edit!


(when (boundp 'july_msgs::JulyMsg)
  (if (not (find-package "JULY_MSGS"))
    (make-package "JULY_MSGS"))
  (shadow 'JulyMsg (find-package "JULY_MSGS")))
(unless (find-package "JULY_MSGS::JULYMSG")
  (make-package "JULY_MSGS::JULYMSG"))

(in-package "ROS")
;;//! \htmlinclude JulyMsg.msg.html


(defclass july_msgs::JulyMsg
  :super ros::object
  :slots (_detail _id ))

(defmethod july_msgs::JulyMsg
  (:init
   (&key
    ((:detail __detail) "")
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _detail (string __detail))
   (setq _id (round __id))
   self)
  (:detail
   (&optional __detail)
   (if __detail (setq _detail __detail)) _detail)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; string _detail
    4 (length _detail)
    ;; int32 _id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _detail
       (write-long (length _detail) s) (princ _detail s)
     ;; int32 _id
       (write-long _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _detail
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _detail (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get july_msgs::JulyMsg :md5sum-) "7f5528a9204f8b31c9206fc97a4a07bb")
(setf (get july_msgs::JulyMsg :datatype-) "july_msgs/JulyMsg")
(setf (get july_msgs::JulyMsg :definition-)
      "string detail
int32 id
")



(provide :july_msgs/JulyMsg "7f5528a9204f8b31c9206fc97a4a07bb")


