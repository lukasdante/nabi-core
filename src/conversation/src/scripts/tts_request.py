import json
import time
import requests
from google.auth import jwt as google_jwt
from google.auth import crypt
from base64 import b64decode
import os
from dotenv import load_dotenv

def json_write(text, 
               languageCode="en-US",
               languageName="en-US-Neural2-C",
               gender="FEMALE",
               encoding="LINEAR16"):  # Change encoding to LINEAR16 for WAV
    data = {
        "input": {
            "text": f"{text}"
        },
        "voice": {
            "languageCode": f"{languageCode}",
            "name": f"{languageName}",
            "ssmlGender": f"{gender}"
        },
        "audioConfig": {
            "audioEncoding": f"{encoding}"
        }
    }

    # Return JSON as a string
    return json.dumps(data)

def init_tts(data):
    # Path to your Service Account JSON key
    SERVICE_ACCOUNT_FILE = os.getenv("TTS_SERVICE_ACCOUNT")
    
    # Google OAuth 2.0 token endpoint
    TOKEN_URL = "https://oauth2.googleapis.com/token"

    # Scopes for the access token
    SCOPES = "https://www.googleapis.com/auth/cloud-platform"

    # Load service account key
    with open(SERVICE_ACCOUNT_FILE) as f:
        service_account_info = json.load(f)

    now = int(time.time())
    payload = {
        "iss": service_account_info["client_email"],
        "scope": SCOPES,
        "aud": TOKEN_URL,
        "iat": now,
        "exp": now + 3600  # Token valid for 1 hour
    }

    # Sign the JWT using the private key from the service account
    signed_jwt = google_jwt.encode(crypt.RSASigner.from_service_account_info(service_account_info), payload)

    # Request access token
    response = requests.post(TOKEN_URL, data={
        "grant_type": "urn:ietf:params:oauth:grant-type:jwt-bearer",
        "assertion": signed_jwt
    })

    # Check response
    if response.status_code == 200:
        access_token = response.json()["access_token"]

        # Use the access token to call the Text-to-Speech API
        api_url = "https://texttospeech.googleapis.com/v1/text:synthesize"
        api_headers = {
            "Authorization": f"Bearer {access_token}",
            "Content-Type": "application/json; charset=utf-8"
        }

        # Make the POST request
        api_response = requests.post(api_url, headers=api_headers, data=data)

        api_response_text = api_response.text

        # Print a message indicating completion
        if api_response.status_code == 200:
            print(f"Response saved.")
        else:
            print(f"API Error: {api_response.status_code} {api_response.text}")
    else:
        print(f"Error: {response.status_code} {response.text}")
    return api_response_text

def decode_tts_output(input, output_file):
    response = json.loads(input)
    audio_data = response['audioContent']

    # Decode base64 to binary data and save as .wav
    with open(output_file, "wb") as new_file:
        new_file.write(b64decode(audio_data))

if __name__ == "__main__":
    load_dotenv()

    sample_text = "I am Nabi, your favorite collaborative robot."
    
    # Get the data with LINEAR16 encoding
    data = json_write(sample_text)
    api_response = init_tts(data)
    decode_tts_output(api_response, "synthesize-text-audio.wav")