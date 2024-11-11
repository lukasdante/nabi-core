#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
import yaml
import json
import argparse
from pathlib import Path
from voiceModules import Recorder, Writer, Parser, Talker
from dotenv import load_dotenv
from ctypes import *

class ConversationalNode:
    def __init__(self, recorder: Recorder, writer: Writer, parser: Parser, talker: Talker):
        self.recorder = recorder
        self.writer = writer
        self.parser = parser
        self.talker = talker

        self.parameters_pub = rospy.Publisher('conversation/parameters', String, queue_size=10)

    def publish(self, msg):
        msg = json.dumps(msg)
        self.parameters_pub.publish(msg)
        rospy.loginfo("Parameters have been published.")

    def start_conversation(self):
        try:

            while not rospy.is_shutdown():
                # Record audio stream until silence is detected
                self.recorder.record()

                if self.recorder.has_speech:
                    # Transcribe audio to text
                    transcription = self.writer.write()

                    if transcription:

                        if 'terminate' in transcription:
                            self.talker.talk('Terminating conversation.')
                            break
                        
                        # Obtain AI agent response and parameters extracted
                        intent = self.parser.detect_intent(transcription)

                        # Publish the parameters to the conversation/parameters topic
                        self.publish(intent['parameters'])
                        
                        # Terminate conversation


                        # Convert response to audio and play the audio
                        self.talker.talk(intent['response'])

                rospy.sleep(0.2)

        except Exception as e:
            rospy.logerr(f"Failed to start conversation: {e}")

if __name__ == '__main__':

    # Handle ASLA errors for cleaner output.
    ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
    def py_error_handler(filename, line, function, err, fmt):
        pass
    c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
    asound = cdll.LoadLibrary('libasound.so')
    asound.snd_lib_error_set_handler(c_error_handler)
    
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

            
        except Exception as e:
            rospy.logerr('Failed to initialize conversational node.')

    node.start_conversation()       