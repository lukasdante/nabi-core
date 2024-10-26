#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    # Callback function for receiving messages
    rospy.loginfo("I heard: %s", data.data)

def listener():
    # Initialize the listener node
    rospy.init_node('listener', anonymous=True)
    # Subscribe to the 'chatter' topic
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()  # Keeps the node running until it's stopped

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
