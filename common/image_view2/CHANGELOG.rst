^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package image_view2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.10 (2018-11-03)
-------------------

2.2.9 (2018-11-02)
------------------

2.2.8 (2018-11-01)
------------------
* Add comment about installation (`#1604 <https://github.com/jsk-ros-pkg/jsk_common/issues/1604>`_)
* [image_view2/image_view2.cpp] Correct grammer. 'could not found' -> could not find' (`#1606 <https://github.com/jsk-ros-pkg/jsk_common/issues/1606>`_)
* Contributors: Yuto Uchimi, Iori Yanokura

2.2.7 (2018-06-27)
------------------

2.2.6 (2018-01-05)
------------------
* image_view2: support kinetic (`#1573 <https://github.com/jsk-ros-pkg/jsk_common/issues/1573>`_)
* image_view2: fix publish_mouse_event (`#1564 <https://github.com/jsk-ros-pkg/jsk_common/issues/1564>`_)
* need to find pcl_ros for image_view2 (`#1541 <https://github.com/jsk-ros-pkg/jsk_common/issues/1541>`_)
* Contributors: Eisoku Kuroiwa, Kei Okada, Yuki Furuta

2.2.5 (2017-06-19)
------------------

2.2.4 (2017-06-14)
------------------

2.2.3 (2017-03-23)
------------------

2.2.2 (2016-12-30)
------------------
* test/publish_lena.py: lena() is not included in scipy from 0.17
* src/image_view2.cpp: add cv::waitKey for opencv3 installed from source to fix freezing issue
* Contributors: Kei Okada

2.2.1 (2016-12-13)
------------------

2.2.0 (2016-10-28)
------------------

2.1.2 (2016-09-14)
------------------

2.1.1 (2016-09-07)
------------------

2.1.0 (2016-09-06)
------------------

2.0.17 (2016-07-21)
-------------------

2.0.16 (2016-06-19)
-------------------

2.0.15 (2016-06-13)
-------------------

2.0.14 (2016-05-14)
-------------------
* fix for error when using opencv3
* Contributors: Krishneel Chaudhary

2.0.13 (2016-04-29)
-------------------
* Support OpenCV3
* Contributors: Kentaro Wada

2.0.12 (2016-04-18)
-------------------

2.0.11 (2016-03-20)
-------------------
* Fix header of screenrectangle topic to include frame_id
  Modified:
  - jsk_ros_patch/image_view2/image_view2.cpp
* remove dynamic_reconfigure.parameter_generator, which only used for rosbuild
* [image_view2] Keep publishing test data
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda

2.0.10 (2016-02-13)
-------------------
* [image_view2] retry 3 to avoid 'failed to launch' error
* Contributors: Kentaro Wada

2.0.9 (2015-12-14)
------------------
* [image_view2] Not test on hydro (jsk_tools/test_topic_published.py does not work on travis/jenkins)
  Modified:
  jsk_ros_patch/image_view2/CMakeLists.txt
* [image_view2] Use ccache if installed to make it fast to generate object files
* [image_view2] Install test dir for rostest after installed
  Modified:
  jsk_ros_patch/image_view2/CMakeLists.txt
* [image_view2] Refactor package.xml (sort and remove no need)
  Modified:
  jsk_ros_patch/image_view2/package.xml
* [image_view2] Test screenrectangle image with mouse event
  Closes https://github.com/jsk-ros-pkg/jsk_common/issues/1247
  Modified:
  jsk_ros_patch/image_view2/package.xml
  Added:
  jsk_ros_patch/image_view2/test/publish_lena.py
  jsk_ros_patch/image_view2/test/publish_mouse_event.py
  jsk_ros_patch/image_view2/test/rectangle_mouse_event.test
* [image_view2] avoid segfo caused by minus width for rectangle
* Contributors: Kentaro Wada, Ryohei Ueda, Yu Ohara

2.0.8 (2015-12-07)
------------------

2.0.7 (2015-12-05)
------------------

2.0.6 (2015-12-02)
------------------

2.0.5 (2015-11-30)
------------------

2.0.4 (2015-11-25)
------------------
* [image_view2] Visualize depth image
* [image_view2] Describe about msg types with topics
* [image_view2] Describe about some publising topics
* [image_view2] Publish cropped image with roi Currently this is only work with rectangle mode.
* [image_view2] Describe ~interactive_mode actual value
* [image_view2] Add document about grabcut rect interaction
* [image_view2] Add document about grabcut interaction
* [image_view2] Add document about poly interaction
* [image_view2] Add document about line interaction
* [image_view2] Add documentation about rectangle mode interaction
* [image_view2] Add document about subscribing topics and advertising services
* [image_view2] Fix typo in document: ImageMarker -> ImageMarker2
* [image_view2] Add README.md
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda

2.0.3 (2015-07-24)
------------------

2.0.2 (2015-07-07)
------------------

2.0.1 (2015-06-28)
------------------

2.0.0 (2015-06-19)
------------------

1.0.72 (2015-06-07)
-------------------
* [image_view2] Fix drawing of rectangle when interaction_mode is grabcut_rect
* Contributors: Ryohei Ueda

1.0.71 (2015-05-17)
-------------------

1.0.70 (2015-05-08)
-------------------

1.0.69 (2015-05-05)
-------------------
* [CMakeLists.txt] add depends to gen_cfg
* Contributors: Kei Okada

1.0.68 (2015-05-05)
-------------------

1.0.67 (2015-05-03)
-------------------
* [image_view2] add Fisheye Grid Line option
* [jsk_perception] add dynamic reconf for image_view2
* [image_view2] add grid option
* Contributors: Yuto Inagaki

1.0.66 (2015-04-03)
-------------------

1.0.65 (2015-04-02)
-------------------
* [image_view2] Use loner queue for event callback in order not to miss event topics
* Contributors: Ryohei Ueda

1.0.64 (2015-03-29)
-------------------
* [image_view2] Clear poly mode caches when image_view2 is resetted
* [image_view2] Support poly mode to select polygonal region on image
* [image_view2] Check if input image is valid and skip if the input is invalid
* [image_view2] Do not show warning message when scale is 0
* Contributors: Ryohei Ueda

1.0.63 (2015-02-19)
-------------------
* [image_view2] Publish marked image in local namespace
* [image_view2] Ignore scale=0 data in scale_interaction.py
* Contributors: Ryohei Ueda

1.0.62 (2015-02-17)
-------------------
* [image_view2] Add utility script to scale mouse event from image_view2
  for resized image
* [image_view2] Initialize window_selection_ and font_ variable even in
  no-window mode
* [image_view2] Publish rectangular region infromation even in grabcut_rect mode
* [image_view2] Reset rectangle region when changing mode
* [image_view2] Add none mode to ignore any interaction with the user
* [image_view2] Add new flag: ratio_scale to pecify size of text by ratio
  to the size of image
* [image_view2] Add left_up_origin flag to ImageMarker2 to draw text from left up origin
* Contributors: Ryohei Ueda

1.0.61 (2015-02-11)
-------------------
* [image_view2] Add topic interface to emulate mouse event
* [image_view2] Separate main function to another cpp file
* [image_view2] Add std_srvs/Empty interface to change interaction mode
* Contributors: Ryohei Ueda

1.0.60 (2015-02-03)
-------------------

1.0.59 (2015-02-03)
-------------------
* Remove rosbuild files
* [image_view2] Add service to change interaction mode
* [image_view2] Support continuous publishing in line selection mode
* [image_view2] Fix timing to publish points selected in line mode
* [image_view2] Add new interaction mode to select line
* [image_view2] Do not publish region outside of the image
* [image_view2] Add ~region_continuous_publish parameter and if it's true,
  image_view2 will keep publishing region selected by user
* [image_view2] Do not show image if no image is available
* [image_view2] Do not use time difference to detect point or rectangle
* Contributors: Ryohei Ueda

1.0.58 (2015-01-07)
-------------------
* [image_view2] Call GUI functions from main thread
* [image_view2] Add new interaction mode to image_view 2 to select
  foreground and background by rectangular region
* [image_view2] add mode to select foreground and background
  for grabcut
* [image_view2] Use opencv2 c++ function to handle window
* [image_view2] add utility function to resolve tf
* [image_view2] refactor to se smaller function
* [image_view2] Use parameter to change mode to select rectangle or
  freeform trajectory instad of "SHIFT KEY"
* [image_view2] Use camel case for methods and functions
* [image_view2] Separate header and cpp file for maintainance
* [image_view2] fix variable name with _ suffix and untabify indents
* [image_view2] Optimize image_view2 to decrease CPU load.
  1) add ~skip_draw_rate to throttle redrawing.
  2) use ros::spin if possible
* Redraw image even though no new message is available
* Add tab-width to image_view2.cpp
* Contributors: Ryohei Ueda

1.0.57 (2014-12-23)
-------------------

1.0.56 (2014-12-17)
-------------------

1.0.55 (2014-12-09)
-------------------

1.0.54 (2014-11-15)
-------------------

1.0.53 (2014-11-01)
-------------------

1.0.52 (2014-10-23)
-------------------

1.0.51 (2014-10-20)
-------------------

1.0.50 (2014-10-20)
-------------------

1.0.49 (2014-10-13)
-------------------

1.0.48 (2014-10-12)
-------------------
* remove depends to opencv2, since indigo depends on libopencv-dev, so we depends on cv_bridge whcih both hydro/indigo depends on it
* Contributors: Kei Okada

1.0.47 (2014-10-08)
-------------------

1.0.46 (2014-10-03)
-------------------

1.0.45 (2014-09-29)
-------------------

1.0.44 (2014-09-26)
-------------------

1.0.43 (2014-09-26)
-------------------

1.0.42 (2014-09-25)
-------------------

1.0.41 (2014-09-23)
-------------------

1.0.40 (2014-09-19)
-------------------

1.0.39 (2014-09-17)
-------------------

1.0.38 (2014-09-13)
-------------------

1.0.36 (2014-09-01)
-------------------

1.0.35 (2014-08-16)
-------------------

1.0.34 (2014-08-14)
-------------------

1.0.33 (2014-07-28)
-------------------

1.0.32 (2014-07-26)
-------------------

1.0.31 (2014-07-23)
-------------------

1.0.30 (2014-07-15)
-------------------

1.0.29 (2014-07-02)
-------------------

1.0.28 (2014-06-24)
-------------------

1.0.27 (2014-06-10)
-------------------
* publish the mouse position to movepoint topic during mouse move event
* Contributors: Ryohei Ueda

1.0.26 (2014-05-30)
-------------------

1.0.25 (2014-05-26)
-------------------

1.0.24 (2014-05-24)
-------------------

1.0.23 (2014-05-23)
-------------------

1.0.22 (2014-05-22)
-------------------

1.0.21 (2014-05-20)
-------------------
* does not check 0.5sec test if the image_view2 is in series mode.
* not use ros::Rate's sleep, use cvWaitKey to captuere
  keys to be pressed
* Contributors: Ryohei Ueda

1.0.20 (2014-05-09)
-------------------

1.0.19 (2014-05-06)
-------------------

1.0.18 (2014-05-04)
-------------------

1.0.17 (2014-04-20)
-------------------

1.0.16 (2014-04-19)
-------------------

1.0.15 (2014-04-19)
-------------------

1.0.14 (2014-04-19)
-------------------

1.0.13 (2014-04-19)
-------------------

1.0.12 (2014-04-18)
-------------------

1.0.11 (2014-04-18)
-------------------

1.0.10 (2014-04-17)
-------------------

1.0.9 (2014-04-12)
------------------

1.0.8 (2014-04-11)
------------------

1.0.4 (2014-03-27)
------------------
* image_View2:add message_generation, message_runtime to package.xml
* in order to avoid empty catkin_LIBRARIES problem, call generate_messaegs after target_link_libraries
* fix typo CATKIN-DEPENDS -> CATKIN_DEPENDS
* Contributors: Ryohei Ueda, Kei Okada

1.0.2 (2014-03-12)
------------------
* `#299 <https://github.com/jsk-ros-pkg/jsk_common/issues/299>`_: add dependency image_view2 to image_view
* fix image_view2 dependency for rosbuild environment
* Contributors: Ryohei Ueda, nozawa

1.0.1 (2014-03-07)
------------------
* added CIRCLE3D type marker sample
* add CIRCLE3D type marker
* Contributors: Kei Okada, HiroyukiMikita

1.0.0 (2014-03-05)
------------------
* set all package to 1.0.0
* install image_view2
* use rosdep instead of depend package
* add find_package PCL for catkin
* supporting series selection in addition to rectangle selection
* use image_transport parameter, it is the same as image_view
* change for updating drawing while not image comming
* adding dependency to generation_message
* add show_info parameter to display curret frame rate, see Issue 247
* catkinize image_view2
* fix all the indent and add the function to fill in the polygon
* add function to draw in the circle
* new parameter: tf_timeout
* support to set the width of a line
* add ~resize_scale_x, ~resize_scale_y parameters for using resized image
* add subscribing point click
* add points_rectangle_extractor.cpp
* changed text msg visualizationo, bigger textsize and color
* add 3d strip/list/polygon/points/text  `#850 <https://github.com/jsk-ros-pkg/jsk_common/issues/850>`_
* fix typo
* add use_window param
* fix for fuerte
* fix deprecated functions
* update comment for TEXT
* use scale for size of the font
* add text example
* fix putText
* check lastCommonTime
* add comments
* added a flag for action==REMOVE&&id==-1, for clear all the markers
* namespace std is needed in image_view2.cpp
* add blurry mode
* set points size to 10
* fix out_msg.encoding from TYPE_32FC1 to bgr8
* update deprecated funcitons to current function api for cam_model
* change fond and use ROS_DEBUG to display tf exception
* send TF exception error at fist 5 times
* changed debug messages for markers from ROS_INFO to ROS_DEBUG
* update to new roseus msg format
* remove deprecated codes
* update to support bayer image and move to cv2
* draw selecting rectangle every time
* add TEXT type marker, only simple outputs yet
* enable ADD/REMOVE action, lifetime, marker colors partially
* change marker_sub buffer from 1 to 10
* remove /reset_time
* publish screenpoint and screenrectangle on namespace + imagetopic_name
* add example to see gripper_tool_frame in image_view2
* remove unused function cmvision-cb
* back to previous version, which is not using subscribeCamera, becouse of slow connection of pr2-network
* rewrite using subscribeCamera
* add image_view2/
* Contributors: Manabu Saito, kazuto, Kei Okada, youhei, Xiangyu Chen, Ryohei Ueda, mikita
