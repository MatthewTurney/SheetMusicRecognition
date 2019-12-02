#!/usr/bin/env python
# license removed for brevity
import rospy
import pickle
import time
import os
import sys
from std_msgs.msg import UInt16
from project_pkg.msg import Music, Note

def main():

    music = get_music()

    pub = rospy.Publisher('music', Music, queue_size=10)

    rospy.init_node('mock_music_sender', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    pub.publish(music)

def get_music():
    with open(os.path.dirname(os.path.dirname(os.path.realpath(sys.argv[0]))) + '/scripts/music/' + sys.argv[1] + '.music', 'rb') as music_file:
        return pickle.load(music_file)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass