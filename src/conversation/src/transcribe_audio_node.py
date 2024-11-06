import os
import requests
import base64
import json
import rospy
import yaml
from dotenv import load_dotenv
from std_msgs.msg import Bool, String

class TranscribeAudioNode:
    def __init__(self):
        rospy.init_node('transcribe_audio_node')

        # load configurations
        self.load_config()

        self.audio_ready = False

        # subscribe to microphone/audio_ready topic
        rospy.Subscriber("microphone/audio_ready", Bool, self.audio_ready_callback)
        self.transcription_pub = rospy.Publisher('transcription/transcription_text', String, queue_size=10)
        self.reset_pub = rospy.Publisher('microphone/audio_ready', Bool, queue_size=10)
    def load_config(self):
        """ Load configuration from YAML file. """
        # load environment variables
        load_dontenv()
        API_KEY = os.getenv('STT_API_KEY')

        try:
            with open("../config.yaml", "r") as file:
                config = yaml.safe_load(file)
                self.api_endpoint = f'https://speech.googleapis.com/v1/speech:recognize?key={API_KEY}'

                if not self.api_endpoint:
                    rospy.logerr('API endpoint not found from the given key.')
                    rospy.signal_shutdown("API key missing.")
                self.encoding = config.get('ENCODING', 'LINEAR16')
                self.sample_rate = config.get('SAMPLE_RATE', 16000)
                self.language = config.get('LANG', 'en-US')
                self.input_audio = config.get('OUTPUT_FILE', 'output.wav')
        
        except Exception as e:
            rospy.logerr(f'Failed to load configuration: {e}')
            raise

    def audio_ready_callback(self, msg):
        self.audio_ready = msg.data

        if self.audio_ready:
            rospy.loginfo("Audio is ready. Transcribing audio...")
            transcription = self.transcribe_audio()
            msg = String()
            msg.data = transcription
            self.transcription_pub.publish(msg)
            rospy.loginfo("Transcription published.")
            self.reset_pub.publish(False)
            self.audio_ready = False
            rospy.loginfo('Resetting audio ready status.')
    
    def encode_audio(self):
        with open(audio_file, 'rb') as f:
            audio_content = f.read()
        return base64.base64encode(audio_content).decode('utf-8')

    def transcribe_audio(self):
        encoded_audio = self.encode_audio(self.input_audio)

        # configure request
        headers = {'Content-Type': 'application/json'}
        body = {
            "config": {
                "encoding": self.encoding,
                "sampleRateHertz": self.sample_rate,
                "languageCode": self.language
            },
            "audio": {
                "content": encoded_audio
            }
        }
        
        # Send request to API endpoint
        response = requests.post(self.api_endpoint, headers=headers, json=body)

        if response.status_code == 200:
            result = response.json()
            if 'results' in result:
                for res in result['results']:
                    rospy.loginfo(f"Transcript: {res['alternatives'][0]['transcript']}")
                    os.remove(self.input_audio)
                    rospy.loginfo(f"Finished transcribing, {self.input_audio} is removed.")
                    return res['alternatives'][0]['transcript']
            else:
                rospy.logwarn("No transcription found.")
        else:
            rospy.logerr(f"Error: {response.status_code}, {response.text}")
            raise

    def run(self):
        rospy.loginfo("Transcribe audio is running...")
        rospy.spin()

node = TranscribeAudioNode()
node.run()