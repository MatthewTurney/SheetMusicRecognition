#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import UInt16
from project_nodes.msg import Music, Note
 
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def main():
    pub1 = rospy.Publisher('servo1', UInt16, queue_size=10)
    pub2 = rospy.Publisher('servo2', UInt16, queue_size=10)

    key_vals = [120, 110, 100, 90, 80, 70, 60, 50] #C, D, E, F, G, A, B, C
    down_val = 80;
    up_val = 90;

    rospy.init_node('arduino_interface', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass