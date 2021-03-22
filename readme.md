Demo of a robot arm in gazebo

Adapted from chapter 5 of ROS robotics by example

Updated several things to work in Melodic, then added and extra joint, a camera, connected to gazebo and detect april tags.

Instructions:
* Clone package into your catkin workspace
* catkin_make to build the packages && source devel/setup.bash

Test 1: basic function in Rviz:
* run command: roslaunch rrbot rrbot_rviz.launch model:=rrbot2.xacro
    * you should see the robot arm with a joint state window, drag around to move the joints

Test 2: inertial values and color for links:
* roslaunch rrbot rrbot_rviz.launch model:=rrbot2.xacro
    * you should see different links now in Rviz

Test 3: with gripper
* roslaunch rrbot rrbot_rviz.launch model:=rrbot3.xacro
    * you should now see and control the gripper (2 joints)
    * note: "TIFFFieldWithTag: Internal error, unknown tag 0x829a" error is normal and can be ignored

Test 4: launching arm in Gazebo with no controllers:
* roslaunch rrbot rrbot_gazebo.launch
    * you should see the gazebo window open and the arm fall under gravity

Test 5: load gazebo controllers an command joints.
* Useful link for dataflows with gazebo: http://gazebosim.org/tutorials/?tut=ros_control
* first run same command above: roslaunch rrbot rrbot_gazebo.launch model:=rrbot4.xacro
* then: roslaunch rrbot rrbot_control.launch controller:=rrbot_control.yaml
* launch rqt and play with joint commands
* try "rosrun rrbot wave.py" to have the robot do the wave

Test 6: add a rotational joint to the robot:
* edit rrbot_control.launch, at line 12, delete "joint_base_mid_position_controller" add: "joint_rot_main_position_controller joint_main_mid_position_controller"
* first run same command above: roslaunch rrbot rrbot_gazebo.launch model:=rrbot5.xacro
* then: roslaunch rrbot rrbot_control.launch controller:=rrbot_control2.yaml
* then run the wave again: rosrun rrbot wave2.py

Test 7: add a camera to the last joint:
* http://gazebosim.org/tutorials?tut=ros_gzplugins#Camera
* camera link was added ro rrbot6.xacro
* added camera plugin to rrbot.gazebo from link above
* first run same command above: roslaunch rrbot rrbot_gazebo.launch model:=rrbot6.xacro (has camera added now)
* then: roslaunch rrbot rrbot_control.launch controller:=rrbot_control2.yaml
* to see the image: rosrun image_view image_view image:=/rrbot/camera1/image_raw

Test 8: add april tag to gazebo and detect it:
* resource: https://optitag.io/blogs/news/using-your-apriltag-with-ros, apriltag models, follow and install: https://github.com/koide3/gazebo_apriltag
* in src: git clone https://github.com/AprilRobotics/apriltag.git && git clone https://github.com/AprilRobotics/apriltag_ros.git
* catkin build in catkin_ws (remove old build files from catkin_make if you were using those)
* configre files in apriltag_ros/config folder
    * be sure to change camera topic names in apriltag_ros launch file:
       * need to also fix the camera_optical bug to remap frame btw gazebo and rviz. make sure to use camera_link_optical as the subscribed topic
* launch the robot in gazebo: roslaunch rrbot rrbot_gazebo.launch model:=rrbot6.xacro
* launch the  controllers: roslaunch rrbot rrbot_control.launch controller:=rrbot_control2.yaml
* spawn the cube with tags on it: roslaunch rrbot spawn_apriltag.launch
* view camera node as above
* using rqt, command the arm till cube is in the camera
* launch the apriltag node: roslaunch apriltag_ros continuous_detection.launch
* view tagged image: rosrun image_view image_view image:=/tag_detections_image
* check it in rviz by adding robot_model and TIFFFieldWithTag
* can also see the world coordinates of the tag via: rosrun tf tf_echo /world /tag_0