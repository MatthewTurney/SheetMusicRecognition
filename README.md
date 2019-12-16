# SheetMusicRecognition
EECS106a/206a final project.

## ROS

### Run Instructions
In 4 separate terminals:\
`roscore`\
`rosrun rosserial_python serial_node.py /dev/ttyACM2` (argument name may be different depending on what the port is called)\
`python arduino_interface_node.py` (in `project_ws/src/project_pkg/src`) \
`python music_sender_node.py alphabet_song` (in `project_ws/src/project_pkg/scripts`, command line arg is song to play)

### Framework
<img src="https://github.com/MatthewTurney/SheetMusicRecognition/blob/master/project_ws/rosgraph.png"/>

#### Nodes
music_sender_node: processes sheet music \
arduino_interface_node: sends angle values to servos according to music \
serial_node: moves servos

#### Messages
Music (Note array) \
Note (key to press, duration of press, duration of rest before pressing)

### Useful Tutorials
http://wiki.ros.org/rosserial_arduino/Tutorials \
https://www.servomagazine.com/magazine/article/november2016_ros-arduino-interfacing-for-robotics-projects \
https://www.theconstructsim.com/solve-error-importerror-no-module-named-xxxx-msg-2/ \
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
