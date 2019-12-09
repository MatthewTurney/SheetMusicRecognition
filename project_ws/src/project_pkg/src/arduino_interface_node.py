#!/usr/bin/env python
# license removed for brevity
import rospy
import pickle
import time
import os
import sys
from std_msgs.msg import UInt16
from project_pkg.msg import Music, Note

key_vals = [113, 105, 98, 90, 84, 76, 68] #C, D, E, F, G, A, B
down_val = 73
up_val = 110
bpm = 60
move_delay = 0.15
up_delay = 0.0
initial_delay = 1.0

def callback(data):

    music = data
    print(music)

    pub1 = rospy.Publisher('servo1', UInt16, queue_size=10)
    pub2 = rospy.Publisher('servo2', UInt16, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    pub1.publish(key_vals[music.notes[0].key])
    print("pub1: " + str(key_vals[music.notes[0].key]))
    pub2.publish(up_val)

    start_time = time.time()

    rate.sleep()

    while time.time() - start_time < initial_delay:
        rate.sleep()

    for note in music.notes:

        note.rest_before += move_delay + up_delay
        note.duration -= (move_delay + up_delay)

        start_time = time.time()

        rate.sleep()

        while time.time() - up_delay < note.rest_before:
            rate.sleep()

        pub1.publish(key_vals[note.key])
        print("pub1: " + str(key_vals[note.key]))

        rate.sleep()

        while time.time() - start_time < note.rest_before:
            rate.sleep()

        pub2.publish(down_val)
        #print("pub2: down")

        start_time = time.time()

        rate.sleep()

        while time.time() - start_time < note.duration * 240 / bpm:
            rate.sleep()

        pub2.publish(up_val)
        #print("pub2: up")

def main():

    rospy.init_node('arduino_interface', anonymous=True)
    rospy.Subscriber('music', Music, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass