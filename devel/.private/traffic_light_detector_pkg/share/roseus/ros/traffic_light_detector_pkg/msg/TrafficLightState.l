;; Auto-generated. Do not edit!


(when (boundp 'traffic_light_detector_pkg::TrafficLightState)
  (if (not (find-package "TRAFFIC_LIGHT_DETECTOR_PKG"))
    (make-package "TRAFFIC_LIGHT_DETECTOR_PKG"))
  (shadow 'TrafficLightState (find-package "TRAFFIC_LIGHT_DETECTOR_PKG")))
(unless (find-package "TRAFFIC_LIGHT_DETECTOR_PKG::TRAFFICLIGHTSTATE")
  (make-package "TRAFFIC_LIGHT_DETECTOR_PKG::TRAFFICLIGHTSTATE"))

(in-package "ROS")
;;//! \htmlinclude TrafficLightState.msg.html


(defclass traffic_light_detector_pkg::TrafficLightState
  :super ros::object
  :slots (_color ))

(defmethod traffic_light_detector_pkg::TrafficLightState
  (:init
   (&key
    ((:color __color) "")
    )
   (send-super :init)
   (setq _color (string __color))
   self)
  (:color
   (&optional __color)
   (if __color (setq _color __color)) _color)
  (:serialization-length
   ()
   (+
    ;; string _color
    4 (length _color)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _color
       (write-long (length _color) s) (princ _color s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _color
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _color (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get traffic_light_detector_pkg::TrafficLightState :md5sum-) "89e44dcab627a2c43a70ae1100695caa")
(setf (get traffic_light_detector_pkg::TrafficLightState :datatype-) "traffic_light_detector_pkg/TrafficLightState")
(setf (get traffic_light_detector_pkg::TrafficLightState :definition-)
      "string color

")



(provide :traffic_light_detector_pkg/TrafficLightState "89e44dcab627a2c43a70ae1100695caa")


