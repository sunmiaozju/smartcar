^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package razor_imu_9dof
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (08-03-2015)
------------------
* Resolving bug in exiting display_3D_visualization (`#15 <https://github.com/KristofRobot/razor_imu_9dof/issues/15>`_)
* Adding dynamic reconfigure for yaw calibration (Paul Bouchier)
* Moving calibration values from firmware to ROS yaml file (`#13 <https://github.com/KristofRobot/razor_imu_9dof/issues/13>`_)

    * Note: this is a BREAKING CHANGE - requires firmware update (updated firmware provided)
    
* Refactoring code: moved scripts to nodes, renamed node.py to imu_node.py (Paul Bouchier)
* Adding diagnostic status reporting (Paul Bouchier)

1.0.5 (15-11-2014)
------------------
* Moving scripts from nodes to scripts dir
* Installing files in ``src`` and ``magnetometer_calibration``
* Major cleanup of package.xml and CMakeLists.txt

1.0.4 (15-11-2014)
------------------
* Adding press 'a' to align feature
* Moving magnetometer calibration sketches under dedicated ``magnetometer_calibration`` directory
* Adding magnetometer calibration sketches for Processing and Matlab (Paul Bouchier)
* Setting default USB port to /dev/ttyUSB0 in all files
* Adding graceful exit in case USB port not found
* Adding queue_size=1
* Fixing x linear accelerations sign

1.0.3 (02-11-2014)
------------------
* Moving all file one directory up
* Changing axis orientation in 3D visualization to be in line with REP 103
* Additional output of linear accelerationa and angular velocity in 3D visualization 
* Adding units of measurement to 3D visualization
* Major graphical improvements to the 3D visualization
* Adding explanation on different launch files

1.0.2 (31-10-2014)
------------------
* Adding valid covariances (Paul Bouchier)
* Fixing incorrect direction of yaw & pitch orientation (Paul Bouchier)
* Fix Readme references & instructions (Paul Bouchier)
* Converting acceleration to m/s^2 (Paul Bouchier)
* Adapting to new output message YPRAG instead of YPRAMG (Paul Bouchier)
* Updating package.xml links to ahrs site (Paul Bouchier)
* Upgrading Arduino package to 1.4.2 from Peter Bartz' site (Paul Bouchier)
* Fixing link to Peter Bartz' site (Paul Bouchier)
* Documenting #ox output mode (Paul Bouchier)
* Renaming new mode #define to better reflect what it does (Paul Bouchier) 
* Adding #ox output mode back into Peter's code (Paul Bouchier)

1.0.1 (15-03-2014)
------------------
* Cleaning up code based on catkin_lint report
* Creating additional launch files for display/publishing
* Implementing flush of first IMU results
* Changing default port, and adding output showing which port was selected
* Removing obsolete ``roslib`` import and ``roslib.load_manifest``

1.0.0 (29-12-2013)
------------------
* First catkinized version
