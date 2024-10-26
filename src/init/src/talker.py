#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    # Initialize the talker node
    rospy.init_node('talker', anonymous=True)
    # Create a publisher that publishes to the 'chatter' topic
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(1)  # Publish rate in Hz

    while not rospy.is_shutdown():
        hello_str = "Hello, ROS! Time: %s" % rospy.get_time()
        rospy.loginfo(hello_str)  # Log the message
        pub.publish(hello_str)  # Publish the message
        rate.sleep()  # Sleep to maintain the publish rate

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
