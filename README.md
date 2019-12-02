# SheetMusicRecognition
EECS106a/206a final project.

## ROS

### Run Instructions
In 3 separate terminals:\
`roscore`\
`rosrun rosserial_python serial_node.py /dev/ttyACM2` (argument name may be different depending on what the port is called)\
`python arduino_interface.py` (in `project_ws/src/project_pkg/src`)

### Messages
Music (Note array) \
Note (key to press, duration of press, duration of rest before pressing)

### Useful Tutorials
http://wiki.ros.org/rosserial_arduino/Tutorials \
https://www.servomagazine.com/magazine/article/november2016_ros-arduino-interfacing-for-robotics-projects \
https://www.theconstructsim.com/solve-error-importerror-no-module-named-xxxx-msg-2/ \
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
