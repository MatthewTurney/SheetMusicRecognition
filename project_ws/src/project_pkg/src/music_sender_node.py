#!/usr/bin/env python
# license removed for brevity
import rospy
import pickle
import time
import os
import sys
from sheet_music.sheet_music_recognition import process_sheet_music
from project_pkg.msg import Music, Note
from std_msgs.msg import UInt16, Float32MultiArray, MultiArrayLayout, MultiArrayDimension

def main():

    music = get_music2()
    print(music)

    pub = rospy.Publisher('music', Music, queue_size=10)

    rospy.init_node('music_sender', anonymous=True)
    rate = rospy.Rate(100) # 10hz

    pub.publish(music)

def get_music():

    string_to_duration = {'eighth': 1.0/8, 'quarter': 1.0 / 4, 'half': 1.0 / 2, 'whole': 1.0}
    msg = process_sheet_music('sheet_music/images/alphabet_song.jpg')

    lst = []
    for n in msg:
        lst.append(Note(n[0], string_to_duration[n[1]], 0))
    """
    music = Float32MultiArray()
    music.layout.dim.append(MultiArrayDimension())
    music.layout.dim.append(MultiArrayDimension())
    music.layout.dim[0].label = "notes"
    music.layout.dim[1].label = "note_data"
    music.layout.dim[0].size = len(lst)
    music.layout.dim[1].size = 3
    music.layout.dim[0].stride = 3*len(lst)
    music.layout.dim[1].stride = 3
    music.layout.data_offset = 0
    music.data = lst
    """

    music = Music(lst)

    with open('alphabet_song.music', 'wb') as music_file:
    	pickle.dump(music, music_file)

    return music


def get_music2():
    with open('alphabet_song.music', 'rb') as music_file:
        return pickle.load(music_file)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass