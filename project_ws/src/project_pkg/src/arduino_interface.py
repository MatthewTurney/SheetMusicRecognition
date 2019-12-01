#!/usr/bin/env python
# license removed for brevity
import rospy
import pickle
import time
from std_msgs.msg import UInt16
from project_pkg.msg import Music, Note
 
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def main():

    music = get_music()

    pub1 = rospy.Publisher('servo1', UInt16, queue_size=10)
    pub2 = rospy.Publisher('servo2', UInt16, queue_size=10)

    key_vals = [120, 110, 100, 90, 80, 70, 60, 50] #C, D, E, F, G, A, B, C
    down_val = 80;
    up_val = 90;

    rospy.init_node('arduino_interface', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    pub2.publish(up_val)

    for note in music.notes:

        start_time = time.time()

        pub1.publish(key_vals[note.key])
        print("pub1: " + str(key_vals[note.key]))

        rate.sleep()

        while time.time() - start_time < note.rest_before:
            rate.sleep()

        pub2.publish(down_val)
        #print("pub2: down")

        start_time = time.time()

        rate.sleep()

        while time.time() - start_time < note.duration:
            rate.sleep()

        pub2.publish(up_val)
        #print("pub2: up")

def get_music():
    with open('twinkle_twinkle.music', 'rb') as music_file:
        return pickle.load(music_file)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass