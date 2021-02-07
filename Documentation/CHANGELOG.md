# Change Log
### VVA Version 0.6.1 (05-nov-2020).
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
