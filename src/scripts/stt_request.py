import base64
import requests
import json
import os
from dotenv import load_dotenv

# Function to encode the WAV file in base64
def encode_audio(audio_file):
    with open(audio_file, 'rb') as f:
        audio_content = f.read()
    return base64.b64encode(audio_content).decode('utf-8')

# Function to perform API call
def transcribe_audio(api_key, audio_file, language="en-US"):
    # URL for Google Speech-to-Text API
    url = f"https://speech.googleapis.com/v1/speech:recognize?key={api_key}"

    # Load and encode the audio file in base64
    encoded_audio = encode_audio(audio_file)

    # Configure the request
    headers = {'Content-Type': 'application/json'}
    body = {
        "config": {
            "encoding": "LINEAR16",  # WAV format after conversion
            "sampleRateHertz": 16000,
            "languageCode": language
        },
        "audio": {
            "content": encoded_audio
        }
    }

    # Send the request to the API
    response = requests.post(url, headers=headers, json=body)

    # Check if the request was successful
    if response.status_code == 200:
        result = response.json()
        if 'results' in result:
            for res in result['results']:
                print(f"Transcript: {res['alternatives'][0]['transcript']}")
                os.remove(audio_file)
                return res['alternatives'][0]['transcript']
        else:
            print("No transcription found")
    else:
        print(f"Error: {response.status_code}, {response.text}")

if __name__ == "__main__":
    # Replace with your actual API key
    load_dotenv()
    STT_API_KEY = os.getenv('STT_API_KEY')

    # Temporary WAV file path
    wav_file = "output.wav"

    # Transcribe the converted WAV audio
    transcribe_audio(STT_API_KEY, wav_file)
    