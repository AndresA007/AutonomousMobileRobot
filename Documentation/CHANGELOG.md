# Change Log
### VVA Version 0.6.1 (07-nov-2020).
* All the ROS packages and nodes that were previously intended to run on the Raspberry Pi or the Laptop, were migrated to run on the Nvidia Jetson Nano (J-Nano). Except by rviz, rtabmapviz and the teleop.
* When running on hardware, the state publishers are now started from “vva_jnano_consolidated/vva_jnano_lowlevel_consolidated.launch”. When running on simulation, they are started from “vva_description/rviz_rtabmap_simulation.launch”.
* The version of rtabmap used is v0.20, which comes in the apt repos. Is no longer required to compile it. The "rgbd_relay" node is not used, because all the nodes run on the same computer (J-Nano).
* Two new launch files were made to decouple the rtabmap (in the J-Nano) and rtabmapviz (in the Laptop): "vva_rtabmap_hw.launch" and "vva_rtabmapviz_hw.launch". The simulation will be managed in a separate launch file: "vva_rtabmap_simulation.launch".
* DeepSpeech was decoupled from the package "vva_voice_interact_server" (previously called "vva_voice_interact_laptop"), now it runs as an independent process under Python 3.7 (ROS Melodic runs on Python 2.7). The communication between DeepSpeech and ROS is done using sockets and a shared file. DeepSpeech version used: v0.8.2.
* The package "vva_voice_interact_server" now notifies the user through the Kinect's LED when a voice command is received and if it was recognized as a valid command or not.
* The URDF model was updated to match the new vehicle, called VAA.
* A new teleop node is available. This can move the vehicle in curves combining translation and rotation. To invoke use: "roslaunch vva_nav_test vva_teleop3.launch".
* An Arduino Mega2560 is now used to manage all the low level tasks, and to avoid the usage of the GPIO ports of the J-Nano, to prevent damages in the J-Nano due to over-voltages.
* The package "vva_base_controller" was modified to split the functionality with the Arduino. The communication with the Arduino is implemented using "rosserial_arduino".
* The wheels calibration is now automatically done when "vva_base_controller.launch" is launched and it detects that the calibration parameters file is not present. There is no need for a separate launch file for calibration.
* The wheels PID controller implemented in the package "vva_base_controller" was modified, it now has two sets of coefficients: one for in place rotation and other for translation. This because the conditions of friction in these two situations are very different.
* The node "vva_topicAnalyzerForAccelLimits.py" now calculates the acceleration limits and prints them in the screen. To invoke: "rosrun vva_nav_test vva_topicAnalyzerForAccelLimits.py"
* The bash script "RunAsBackgroundScript/runVVA_ROSNodesOnBackground.sh" is now available to start all the nodes in the background. It has 4 options: "start_mapping", "start_navigation", "status" and "stop".
* The node “vva_navigation/vva_navigation_correction.py” was modified to stop the RP-Lidar when there is no any goal in progress.
* A watchdog crontab script was done to monitor the status of the WiFi connection and restart the WiFi interface if the connectivity fails. Name of the script: "EmailNotificationAndWatchdogScripts/WiFiWatchdog.sh".
* The package "vva_robot_healthcheck" was added to receive the status of the WiFi from the watchdog script and notify the user through the Kinect's LED and tilt.
* The node "vva_user_intents/vva_navigation_intent.py" was modified to transform all the goals to the "map" frame.

## Previous versions
### VVA Version 0.6 (08-aug-2020)
* Supports the use of the microphones array of the Kinect by detecting a wake-word after which the user can record a 4 seconds audio clip with a voice command.
* Introduces a Mobile Client App that supports the use of the microphone of the mobile phone to record voice commands.
* The voice commands are processed by the Speech Recognition node in the package vva_voice_interact_laptop. This node is based on Mozilla DeepSpeech v0.6.1 and currently supports only american english language.
* All the audio clips captured throught the microphones array of the Kinect are saved and preserved for future use as datasets for training DeepSpeech. The audio clips captured through the Mobile App are not preserved.
* Introduces an architecture to implement the Intent Recognition, to interpret the Voice Commands. Several Intent Recognition nodes can be deployed in the package vva_user_intents, each node specialized in different tasks. Currently, there is only one node in charge of the navigation related tasks.
* The Intent Recognition node in charge of the navigation tasks (vva_navigation_intent.py) supports the following commands: "navigate to", "start patrolling" and "stop navigation".
* The known issue of v0.5 regading vva_navigation_correction not working on simulation is now fixed.
* The launch files "vva_odom_correction.launch", "vva_move_base.launch" and "vva_navigation_correction.launch" were consolidated in one launch file called "vva_consolidated_nav.launch".

### VVA Version 0.5 (18-jun-2020)
* This version has important improvements in the autonomous navigation, besides, it incorporates the detection of low obstacles and cliffs.
* The configuration of the navigation stack is changed to implement the use of layer based costmaps through the use of plugins.
* The plugins "voxel_grid" and "spatio_temporal_voxel_layer" are tested to implement 2.5D costmaps. These can be optionally enabled to use the Kinect to detect obstacles that are below the line of sight of the LIDAR.
* The node "costmap_2d/costmap_2d_cloud" is tested to show in rviz the "voxel_grid" as a PointCloud. "spatio_temporal_voxel_layer" doesn’t require this plugin to show the voxel grid in rviz.
* The detection of obstacles that are below the line of sight of the LIDAR was added by using the Kinect and the node "depth_nav_tools/laserscan_kinect", invoked from "vva_navigation/vva_move_base.launch".
* The detection of cliffs and stairs with the Kinect was added by using the node "vva_cliff_detector/vva_cliff_detector", invoked from "vva_navigation/vva_move_base.launch". The plugin "vva_cliff_detector_layer::VVACliffDetectorLayer" was used too.
* "vva_kinect_aux" was modified to correct the jump or flapping shown in rviz when the tilt of the Kinect was changed. The joint of the base was fixed at 0 degrees with respect to “y” axis.
* The optional module "rtabmap_ros/point_cloud_xyz" was tested to generate PointClouds from the depth-image. To test it use: "roslaunch vva_navigation vva_depthimage_2_pointcloud.launch simulation:=false".
* A customizable script was created for optional statistics generation from the topics. To test it use: "rosrun vva_nav_test vva_topic_analyzer.py"
* A node was developed to correct the LIDAR based odometry "vva_lidar_odom/vva_odom_correction.py", it makes periodic restarts of the odometry.
* A node was developed to correct the navigation "vva_navigation/vva_navigation_correction.py", it sets a temporary closer goal to unstuck the robot.

### VVA Version 0.4 (18-may-2020)
* Encoders and PID control is implemented for the wheels.
* These files are added in vva_base_controller/src/: vva_base_encoders_publisher.py and vva_wheels_calibration.py.
* The file vva_base_controller/src/vva_base_controller.py is completely rewritten.
* Automatic calibration was added to find the maximum and minimum velocity of the wheels’ motors.
* The capacity to change the tilt of the Kinect was added by using the topic "vva_kinect/tilt_angle".
*  The capacity to change the LED modes of the Kinect was added by using the topic "vva_kinect/led_option".
* The detection of the tilt of the vehicle with respect to “x” and “y” axis was added by using the internal accelerometers of the Kinect.
