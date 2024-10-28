from recorder import record_audio
from stt_request import transcribe_audio
from detect_intent import detect_intent_text
from tts_request import init_tts, decode_tts_output, json_write
from play_sound import play_wav
from dotenv import load_dotenv
import os
import uuid
import json
import datetime

def record_and_transcribe():
    try:
        # record audio
        record_audio(threshold=1500)

        # transcribe the input audio
        transcribed_audio = transcribe_audio(STT_API_KEY, "output.wav")

        # detect intent from the transcribed audio
        response, parameters = detect_intent_text(transcribed_audio, PROJECT_ID, LOCATION_ID, AGENT_ID, SESSION_ID)
        
        # decode the response to an audio output
        data = json_write(response)
        api_response = init_tts(data)
        decode_tts_output(api_response, "response.wav")
        
        # play the audio output
        play_wav("response.wav")

        print(parameters)

        return parameters

    except Exception as e:
        print(e)


def generate_session_id():
    return uuid.uuid4()

if __name__ == "__main__":
    SESSION_ID = generate_session_id()
    load_dotenv(override=True)
    STT_API_KEY = os.getenv('STT_API_KEY')
    TTS_API_KEY = os.getenv('TTS_API_KEY')
    PROJECT_ID = os.getenv("DFCX_PROJECT_ID")
    LOCATION_ID = os.getenv("DFCX_LOCATION_ID")
    AGENT_ID = os.getenv("DFCX_AGENT_ID")
    service_account_path = os.getenv("DFCX_SERVICE_ACCOUNT")
    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = service_account_path

    terminate = False
    while not terminate:
        parameters = record_and_transcribe()

        # parameters['session'] = str(SESSION_ID)
        # parameters['timestamp'] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        with open("parameters.txt", "a") as f:
            json.dump(parameters, f)
            f.write('\n')

        # if parameters['terminate'] == "True":
        #     terminate = True

            # locate 