import os
import rospy
import pyaudio
import wave
import yaml
import requests
import base64
import json
import time
import uuid
import numpy as np
from dotenv import load_dotenv
from collections import namedtuple
from std_msgs.msg import String, Bool
from google.cloud.dialogflowcx_v3beta1.services.agents import AgentsClient
from google.cloud.dialogflowcx_v3beta1.services.sessions import SessionsClient
from google.cloud.dialogflowcx_v3beta1.types import session
from google.api_core.client_options import ClientOptions
from google.protobuf.json_format import MessageToDict
from google.auth import jwt as google_jwt
from google.auth import crypt

class Recorder:
    def __init__(self, config):
        try:
            self.chunk_size = config.get('CHUNK_SIZE', 1024)
            self.channels = config.get('CHANNELS', 1)
            self.sample_rate = config.get('SAMPLE_RATE', 16000)
            self.threshold = config.get('THRESHOLD', 1000)
            self.silence_limit = config.get('SILENCE_LIMIT', 1.2)
            self.input_file = config.get('INPUT_AUDIO_FILE', 'input.wav')
            self.audio_format = pyaudio.paInt16
            self.has_speech = True
            rospy.loginfo("Recorder initialized.")
        except Exception as e:
            rospy.logerr(f"Unable to initialize recorder: {e}")

    def record(self):
        """ Record audio until silence is detected. """

        audio = pyaudio.PyAudio()

        # Open audio stream for recording
        try:
            stream = audio.open(format=self.audio_format,
                            channels=self.channels,
                            rate=self.sample_rate,
                            input=True,
                            frames_per_buffer=self.chunk_size)
            
            rospy.loginfo("Recording started. Speak into the microphone...")
        except Exception as e:
            rospy.logerr(f"Failed to open audio stream: {e}.")
            return

        # Read and store the audio stream until silence is detected
        frames = []
        silent_chunks = 0
        last_log_time = rospy.get_time()
        initial_time = rospy.get_time()
        while True:
            try:
                data = stream.read(self.chunk_size)
            except Exception as e:
                rospy.logerr(f"Error reading audio data: {e}")
                break

            frames.append(data)
            
            # Calculate the volume
            volume = self.get_volume(data)

            # For every 200ms display the volume
            if rospy.get_time() - last_log_time >= 0.2:
                rospy.loginfo(f"Current volume: {volume}")
                last_log_time = rospy.get_time()
            
            # Record silent chunks and stop recording once limit is reached
            if volume < self.threshold:
                silent_chunks += 1
            else:
                silent_chunks = 0
            
            if silent_chunks >= int(self.silence_limit * self.sample_rate / self.chunk_size):
                rospy.loginfo("Silence detected. Stopping recording...")

                # If audio is entirely silent, set status to silent
                if rospy.get_time() - initial_time > (self.silence_limit + 0.1):
                    self.has_speech = True
                else:
                    self.has_speech = False

                break

        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        audio.terminate()

        # Save the recorded audio to a file
        wf = wave.open(self.input_file, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(audio.get_sample_size(self.audio_format))
        wf.setframerate(self.sample_rate)
        wf.writeframes(b''.join(frames))
        wf.close()
    
        rospy.loginfo(f"Recording has been saved as {self.input_file}.")

    def get_volume(self, data):
        """ Obtains the mean absolute value of the current audio. """

        if not data:
            return 0
        audio_data = np.frombuffer(data, dtype=np.int16)
        return np.abs(audio_data).mean()

class Writer:
    def __init__(self, config):
        try:
            self.api_key = os.getenv('STT_API_KEY')
            self.api_endpoint = f'https://speech.googleapis.com/v1/speech:recognize?key={self.api_key}'
            self.language = config.get('LANG', 'en-US')
            self.sample_rate = config.get('SAMPLE_RATE', 16000)
            self.encoding_format = config.get('ENCODING', 'LINEAR16')
            self.input_file = config.get('INPUT_AUDIO_FILE', 'input.wav')

            rospy.loginfo("Writer initialized.")
        except Exception as e:
            rospy.logerr(f"Unable to initialize writer: {e}")

    def write(self):
        """ Encodes audio to text. """

        # Load and encode the audio file
        encoded_audio = self.encode_audio()

        # Configure the request
        headers = {'Content-Type': 'application/json'}
        body = {
            'config': {
                'encoding': self.encoding_format,
                'sampleRateHertz': self.sample_rate,
                'languageCode': self.language
            },
            'audio': {
                'content': encoded_audio
            }
        }

        # Send the request to the API endpoint
        try:
            response = requests.post(self.api_endpoint, headers=headers, json=body)
        except Exception as e:
            rospy.logerr(f"Writer request error: {e}")

        # Check if the request is successful
        if response.status_code == 200:
            result = response.json()

            # Get the transcription from results
            if 'results' in result:
                for res in result['results']:
                    transcription = res['alternatives'][0]['transcript']
                    rospy.loginfo(f"Transcript: {transcription}")

                    # Remove the audio file
                    if os.path.exists(self.input_file):
                        os.remove(self.input_file)

                    return transcription
            else:
                rospy.loginfo("No transcription found.")
                return None
        else:
            rospy.logerr(f"Writer request error {response.status_code}.")
            return None
    
    def encode_audio(self):
        """ Encode audio to base64 and decode to utf-8. """

        with open(self.input_file, 'rb') as f:
            audio_content = f.read()
        return base64.b64encode(audio_content).decode('utf-8')

class Parser:
    def __init__(self, config):
        try:
            self.project_id = os.getenv("DFCX_PROJECT_ID")
            self.location_id = os.getenv("DFCX_LOCATION_ID")
            self.agent_id = os.getenv("DFCX_AGENT_ID")
            self.service_account = os.getenv("DFCX_SERVICE_ACCOUNT")
            os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.service_account
            self.agent = f'projects/{self.project_id}/locations/{self.location_id}/agents/{self.agent_id}'
            self.session_id = None
            self.language = config.get('LANG', 'en-US')

            rospy.loginfo("Parser initialized.")
        except Exception as e:
            rospy.logerr(f"Unable to initialize parser: {e}")

    def detect_intent(self, text):
        """ Detect intent, extract parameters, and output response. """

        if not self.session_id:
            self.session_id = uuid.uuid4()
        
        session_path = f'{self.agent}/sessions/{self.session_id}'
        api_endpoint = f'{AgentsClient.parse_agent_path(self.agent)["location"]}-dialogflow.googleapis.com:443'
        client_options = ClientOptions(api_endpoint=api_endpoint)
        session_client = SessionsClient(client_options=client_options)

        # Configure the request
        text_input = session.TextInput(text=text)
        query_input = session.QueryInput(text=text_input, language_code=self.language)
        
        # Send the request
        request = session.DetectIntentRequest(
            session=session_path, query_input=query_input
        )

        # Obtain the response
        response = session_client.detect_intent(request=request)

        # 
        response_messages = [
            " ".join(msg.text.text) for msg in response.query_result.response_messages
        ]

        # Convert the parameters to a dictionary and prepare response
        parameters = MessageToDict(response._pb)
        rospy.loginfo(f"Parameters: {parameters['queryResult']['parameters']}")

        response_text = ' '.join(response_messages)
        rospy.loginfo(f"Response text: {response_text}")
        
        # Return the Intent
        return {'response': response_text, 'parameters': parameters['queryResult']['parameters']}

class Talker:
    def __init__(self, config):
        try:
            self.service_account = os.getenv("TTS_SERVICE_ACCOUNT")

            with open(self.service_account) as f:
                self.service_account_info = json.load(f)

            self.language = config.get('LANG', 'en-US')
            self.gender = config.get('GENDER', 'FEMALE')
            self.accent = config.get('ACCENT', 'en-US-Neural2-C')
            self.encoding_format = config.get('ENCODING', 'LINEAR16')
            self.output_file = config.get('OUTPUT_AUDIO_FILE', 'output.wav')
            self.token_validity = config.get('TOKEN_LIFE', 3600)
            self.token_url = "https://oauth2.googleapis.com/token"
            self.scopes = "https://www.googleapis.com/auth/cloud-platform"
            self.api_endpoint = "https://texttospeech.googleapis.com/v1/text:synthesize"
            


            rospy.loginfo("Talker initialized.")
        except Exception as e:
            rospy.logerr(f"Unable to initialize talker: {e}")
    def write_json(self, text):
        """ Writes JSON payload for Text-to-Speech Inference """
        
        data = {
            "input": {
                "text": f"{text}"
            },
            "voice": {
                "languageCode": f"{self.language}",
                "name": f"{self.accent}",
                "ssmlGender": f"{self.gender}"
            },
            "audioConfig": {
                "audioEncoding": f"{self.encoding_format}"
            }
        }

        return json.dumps(data)

    def vocalize(self, data):
        """ Returns speech of a given API response. """

        now = int(time.time())
        payload = {
            "iss": self.service_account_info["client_email"],
            "scope": self.scopes,
            "aud": self.token_url,
            "iat": now,
            "exp": now + self.token_validity
        }

        # Sign the JWT using the private key from the service account
        signed_jwt = google_jwt.encode(crypt.RSASigner.from_service_account_info(self.service_account_info), payload)

        # Request access token
        response = requests.post(self.token_url, data={
            "grant_type": "urn:ietf:params:oauth:grant-type:jwt-bearer",
            "assertion": signed_jwt
        })

        # Check response
        if response.status_code == 200:
            access_token = response.json()["access_token"]

            # Use the access token to call the Text-to-Speech API
            api_headers = {
                "Authorization": f"Bearer {access_token}",
                "Content-Type": "application/json; charset=utf-8"
            }

            # Make the POST request
            api_response = requests.post(self.api_endpoint, headers=api_headers, data=data)

            api_response_text = api_response.text

            # Print a message indicating completion
            if api_response.status_code == 200:
                rospy.loginfo(f"Response saved.")
            else:
                rospy.logerr(f"API Error: {api_response.status_code} {api_response.text}")
        else:
            rospy.logerr(f"Error: {response.status_code} {response.text}")
        return api_response_text

    def save_audio(self, speech):
        response = json.loads(speech)
        audio_data = response['audioContent']

        # Decode base64 to binary data and save as .wav
        with open(self.output_file, "wb") as file:
            file.write(base64.b64decode(audio_data))

    def talk(self, response):
        """ Talks given a response in text. """

        response = self.write_json(response)
        speech = self.vocalize(response)
        self.save_audio(speech)

        try:
            # Open the wav file
            with wave.open(self.output_file, 'rb') as wav_file:
                # Set up the PyAudio stream
                audio = pyaudio.PyAudio()
                stream = audio.open(
                    format=audio.get_format_from_width(wav_file.getsampwidth()),
                    channels=wav_file.getnchannels(),
                    rate=wav_file.getframerate(),
                    output=True
                )

                # Read and play audio data
                data = wav_file.readframes(1024)
                while data:
                    stream.write(data)
                    data = wav_file.readframes(1024)

                # Stop and close the stream
                stream.stop_stream()
                stream.close()
                audio.terminate()

            rospy.loginfo("Playback finished.")
            os.remove(self.output_file)
        except Exception as e:
            rospy.logerr(f"An error occurred: {e}")

