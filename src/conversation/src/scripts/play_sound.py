import wave
import pyaudio
import os

def play_wav(file_path):
    try:
        # Open the wav file
        with wave.open(file_path, 'rb') as wav_file:
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

        print("Playback finished.")
        os.remove(file_path)
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # Replace 'file_example_WAV_700KB.wav' with your actual file path
    wav_file = 'synthesize-text-audio.wav'
    play_wav(wav_file)