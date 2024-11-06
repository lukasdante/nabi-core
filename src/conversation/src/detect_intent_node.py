import yaml
from dotenv import load_dotenv
from std_msgs.msg import Bool, String
import json

class DetectIntentNode:
    def __init__(self):
        rospy.init_node('detect_intent_node')

        # load configurations
        self.load_config()

        # publish the parameters and the response
        self.params_pub = rospy.Publisher('intent/parameters', String, queue_size=10)
        self.response_pub = rospy.Publisher('intent/response', String, queue_size=10)

    def load_config(self):
        """ Load configuration from YAML file. """

        