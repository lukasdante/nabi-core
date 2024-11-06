import pyaudio
import numpy as np
import wave

# Audio parameters
CHUNK = 1024  # Number of audio frames per buffer
FORMAT = pyaudio.paInt16  # Audio format (16-bit samples)
CHANNELS = 1  # Mono audio
RATE = 16000  # Sampling rate (16kHz)
THRESHOLD = 1000  # Volume threshold for silence detection (calibrate this)
SILENCE_LIMIT = 1.2  # Number of seconds of silence before stopping the recording

def get_volume(data):
    """Calculate the volume (mean absolute amplitude) of the audio data."""
    audio_data = np.frombuffer(data, dtype=np.int16)
    return np.abs(audio_data).mean()

def is_silent(data, threshold):
    """Returns 'True' if the audio is silent below the threshold."""
    return get_volume(data) < threshold

def record_audio(output_filename="output.wav", threshold=THRESHOLD):
    """Record audio until silence is detected."""
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)
    
    print("Recording started. Speak into the microphone...")
    
    frames = []
    silent_chunks = 0
    
    while True:
        data = stream.read(CHUNK)
        frames.append(data)
        
        # Calculate the volume and print it for calibration
        volume = get_volume(data)
        print(f"Current volume: {volume}")
        
        if is_silent(data, THRESHOLD):
            silent_chunks += 1
        else:
            silent_chunks = 0
        
        if silent_chunks >= int(SILENCE_LIMIT * RATE / CHUNK):
            print("Silence detected. Stopping recording...")
            break

    # Stop and close the stream
    stream.stop_stream()
    stream.close()
    p.terminate()

    # Save the recorded audio to a file
    wf = wave.open(output_filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()
    
    print(f"Recording saved as {output_filename}")

if __name__ == "__main__":
    record_audio()
    
