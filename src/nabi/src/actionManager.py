#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class ActionManager:
    def __init__(self):
        try:
            # Subscribe to the `conversation/parameters` topic
            rospy.Subscriber('conversation/parameters', String, self.callback)
            rospy.loginfo('Action Manager node is initalized.')
        except Exception as e:
            rospy.logerr(f'Failed to initialize action manager node: {e}')
    
    def callback(self, msg):
        """ Creates a callback for the ActionManager node. """
        rospy.loginfo(f'Parameters: {msg.data}')

        # Deserialize the JSON data from the received message
        parameters = json.loads(msg.data)

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('action_manager', anonymous=True)

    # Instantiate the ActionManager
    manager = ActionManager()

    # Keep the node running and wait for callbacks
    rospy.spin()