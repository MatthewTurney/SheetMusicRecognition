#!/usr/bin/env python
# license removed for brevity
import rospy
import pickle
import time
import os
import sys
from sheet_music import sheet_music_recognition

from std_msgs.msg import UInt16
from project_pkg.msg import Music, Note


def main():

    music = get_music()

    pub = rospy.Publisher('music', Music, queue_size=10)

    rospy.init_node('mock_music_sender', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    pub.publish(music)

def get_music():
    string_to_duration = {'quarter': 1.0 / 4, 'half': 1.0 / 2, 'whole': 1}
    msg = process_sheet_music('sheet_music/images/alphabet_song.jpg', False)
    lst = []
    for n in msg:
        lst.append(Note(key_to_int[n[0]], string_to_duration[n[1]], 0))

    music = Music(lst)

    return music


def get_music2():
    with open(os.path.dirname(os.path.dirname(os.path.realpath(sys.argv[0]))) + '/scripts/music/' + sys.argv[1] + '.music', 'rb') as music_file:
        return pickle.load(music_file)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass