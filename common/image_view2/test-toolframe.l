#!/usr/bin/env roseus
;;
;; FOR PR2 USER ONLY
;; for non-pr2 users, use test-imageview2.l
;;
;; rosrun image_view2 image_view2 image:=/wide_stereo/left/image_raw camera_info:=/wide_stereo/left/camera_info
;;
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")
;;;
;;;
(ros::roseus "imageview2-client")

(ros::advertise "image_marker" image_view2::ImageMarker2 1)

(ros::rate 10)
(while (ros::ok)
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :header :stamp (ros::time-now))
    (send mrk :type image_view2::ImageMarker2::*FRAMES*)
    (send mrk :frames (list "/r_gripper_tool_frame" "/l_gripper_tool_frame"))
    (ros::publish "image_marker" mrk)
    (print (list (send mrk :header :stamp) (send mrk :frames)))
    (ros::spin-once)
    (ros::sleep)))


