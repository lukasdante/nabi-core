#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
import pyaudio
import numpy as np
import wave
import yaml

class MicrophoneNode:
    def __init__(self):

        # Load configurations
        self.load_config()

        # Publisher to signal that audio is ready
        self.audio_pub = rospy.Publisher('microphone/audio_ready', Bool, queue_size=10)
        rospy.Subscriber('conversation/ready', Bool, self.ready_callback)

        self.start_recording = rospy.get_param('~start_recording', False)
        
        rospy.loginfo("Microphone node successfully launched.")
        rospy.loginfo(self.start_recording)
    def load_config(self):
        """ Load configuration from YAML file. """

        self.ready = True
        self.audio_format = pyaudio.paInt16

        try:
            with open("../config.yaml", "r") as file:
                config = yaml.safe_load(file)
                self.chunk_size = config.get('CHUNK_SIZE', 1024)
                self.channels = config.get('CHANNELS', 1)
                self.sample_rate = config.get('SAMPLE_RATE', 16000)
                self.threshold = config.get('THRESHOLD', 1000)
                self.silence_limit = config.get('SILENCE_LIMIT', 1.2)
                self.output_file = config.get('OUTPUT_FILE', 'output.wav')
        except Exception as e:
            rospy.logerr('Failed to load configuration.')
            raise

    def ready_callback(self, msg):
        # check if the speaker node has finished
        self.ready = msg.data

    def get_volume(self, data):
        """Calculate the volume (mean absolute amplitude) of the audio data."""
        audio_data = np.frombuffer(data, dtype=np.int16)
        return np.abs(audio_data).mean()

    def record_audio(self):
        """Record audio until silence is detected."""

        audio = pyaudio.PyAudio()

        try:
            stream = audio.open(format=self.audio_format,
                            channels=self.channels,
                            rate=self.sample_rate,
                            input=True,
                            frames_per_buffer=self.chunk_size)
            
            rospy.loginfo("Recording started. Speak into the microphone...")
            
        except Exception as e:
            rospy.logerr("Failed to open audio stream:")
            return

        frames = []
        silent_chunks = 0
        
        while True:
            try:
                data = stream.read(self.chunk_size)
            except Exception as e:
                rospy.logerr("Error reading audio data:")

            frames.append(data)
            
            # Calculate the volume and print it for calibration
            volume = self.get_volume(data)
            rospy.loginfo("Current volume:")
            
            if volume < self.threshold:
                silent_chunks += 1
            else:
                silent_chunks = 0
            
            if silent_chunks >= int(self.silence_limit * self.sample_rate / self.chunk_size):
                rospy.loginfo("Silence detected. Stopping recording...")
                break

        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        audio.terminate()

        # Save the recorded audio to a file
        wf = wave.open(self.output_file, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(audio.get_sample_size(self.audio_format))
        wf.setframerate(self.sample_rate)
        wf.writeframes(b''.join(frames))
        wf.close()
    
        rospy.loginfo("Recording saved as")
        self.start_recording = False

        # publish to topic TODO: add publish to speaker node
        self.audio_pub.publish(True)

    def run(self):
        rate = rospy.Rate(10)
        while not ros.is_shutdown():
            rospy.loginfo("Microphone node is successfully running.")
            rospy.spin()

if __name__ == '__main__':
    rospy.init_node('microphone_node')
    rospy.loginfo('Microphone node initialized.')
    rospy.run()
    # node = MicrophoneNode()
    # node.run()