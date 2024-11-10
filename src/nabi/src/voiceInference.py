#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
import yaml
import argparse
from pathlib import Path
from voiceModules import Recorder, Writer, Parser, Talker
from dotenv import load_dotenv

class ConversationalNode:
    def __init__(self, recorder: Recorder, writer: Writer, parser: Parser, talker: Talker):
        self.recorder = recorder
        self.writer = writer
        self.parser = parser
        self.talker = talker

        self.parameters_pub = rospy.Publisher('conversation/parameters', String, queue_size=10)

    def publish(self, msg):
        self.parameters_pub.publish(msg)
        rospy.loginfo("Parameters have been published.")

    def start_conversation(self):
        try:
            rospy.loginfo("Starting conversation loop.")

            while not rospy.is_shutdown():
                # Record audio stream until silence is detected
                recording = self.recorder.record()

                if not self.recorder.is_silent:
                    # Transcribe audio to text
                    transcription = self.writer.write(self.recorder.output_file)
                    
                    # Obtain AI agent response and parameters extracted
                    response, parameters = self.parser.detect_intent(transcription)

                    # Publish the parameters to the conversation/parameters topic
                    self.publish(parameters)

                    # Convert response to audio and play the audio
                    self.talker.talk(response)
        except Exception as e:
            rospy.logerr(f"Failed to start conversation: {e}")

if __name__ == '__main__':
    # Load environment variables
    load_dotenv()

    # Argument passer for scripting
    parser = argparse.ArgumentParser()

    # Argument for configuration
    parser.add_argument(
        "--config",
        type=str,
        required=False,
        default='config.yaml',
        help="Configuration file to instantiate the classes, should be found in nabi package, defaults to config.yaml."
    )
    args = parser.parse_args()

    # Get config.yaml directory
    cwd = Path.cwd()
    config_path = (cwd.parent / args.config).resolve()

    # Initialize the node
    rospy.init_node('conversational_node')

    # Using config.yaml, initialize ConversationalNode()
    with open(config_path) as file:
        try: 
            config = yaml.safe_load(file)

            recorder = Recorder(config)
            writer = Writer(config)
            parser = Parser(config)
            talker = Talker(config)
            node = ConversationalNode(recorder, writer, parser, talker)
            
            rospy.loginfo('Successfully initialized conversational node.')

            node.start_conversation()
        except Exception as e:
            rospy.logerr('Failed to initialize conversational node.')