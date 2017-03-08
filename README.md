[Build}:

1. Clone this repo and place the 3 folders in the src directory of your catkin workspace.
2. Change the focal length and principal focal point in apriltag_ros/src/april_detector.cpp . It has been hard-coded for the time being.
3. catkin_make from the root of the workspace.
4. Cross fingers and pray.

[Run]:

1. First run : roslaunch usb_cam usb_cam-test.launch
2. Then in another tab run : roslaunch apriltags_ros example.launch
3. To view detections : rostopic echo -c /tag_detections.

[For developers]
1. The apriltag_ros node publishes pose data in the format defined in apriltag_ros/msg, under the topic /tag_detections.

For further info:
Email me.
