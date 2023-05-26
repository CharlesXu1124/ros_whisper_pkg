#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from whisper_interfaces.srv import WhisperResponse
import os
import torch
import whisper
import pyaudio
import wave
from pydub import AudioSegment
from pydub.playback import play
import threading
import datetime
import time

class RosWhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        self.openai_service = self.create_service(
            WhisperResponse, 'whisper_server', self.llm_callback
        )

        DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
        
        # Global variables
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 2
        self.RATE = 44100
        self.RECORD_SECONDS = 5  # Change this value to adjust the duration of each recording
        self.OUTPUT_FILENAME = '{}.mp3'
        self.STOP_RECORDING = False

        self.model = whisper.load_model("small", device=DEVICE)
        self.get_logger().info(
            f"Model is {'multilingual' if self.model.is_multilingual else 'English-only'} "
            f"and has {sum(np.prod(p.shape) for p in self.model.parameters()):,} parameters."
        )

    def remove_temp_file(self, filename):
        import os
        try:
            os.remove(filename)
        except OSError as e:
            print(f"Error deleting temporary file: {e}")

    def record_audio(self):

        # Create an instance of PyAudio
        audio = pyaudio.PyAudio()

        # Open the stream
        stream = audio.open(format=self.FORMAT, channels=self.CHANNELS, rate=self.RATE, input=True, frames_per_buffer=self.CHUNK)

        print("Recording started...")

        frames = []

        # Record audio until the flag is set to True
        while not self.STOP_RECORDING:
            data = stream.read(self.CHUNK)
            frames.append(data)
        
        print("Recording stopped.")
        
        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        
        # Terminate the PyAudio instance
        audio.terminate()
        
        # Save the recorded audio as a WAV file
        now = datetime.datetime.now()
        filename = "record"
        wav_filename = "record.mp3"

        wf = wave.open(wav_filename, 'wb')
        wf.setnchannels(self.CHANNELS)
        wf.setsampwidth(audio.get_sample_size(self.FORMAT))
        wf.setframerate(self.RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
        
        # Convert the WAV file to MP3
        audio = AudioSegment.from_wav(wav_filename)
        audio.export(filename, format='mp3')
        
        # Delete the temporary WAV file
        self.remove_temp_file(wav_filename)
        
        print(f"Recording saved as {filename}")
        
    

    def start_recording(self):
        
        # Create a new thread for recording audio
        record_thread = threading.Thread(target=self.record_audio)
        
        # Set the flag to False to start recording
        self.STOP_RECORDING = False
        
        # Start the recording thread
        record_thread.start()

    def stop_recording(self):
        
        # Set the flag to True to stop recording
        self.STOP_RECORDING = True


    def llm_callback(self, request, response): 
        self.get_logger().info("call received")
        
        self.start_recording()
        time.sleep(5)
        self.stop_recording()
        
        
        audio = whisper.load_audio("record.mp3")
        audio = whisper.pad_or_trim(audio)
        mel = whisper.log_mel_spectrogram(audio).to(self.model.device)

        # _, probs = self.model.detect_language(mel)
        # print(f"Detected language: {max(probs, key=probs.get)}")

        options = whisper.DecodingOptions(language="en", without_timestamps=True, fp16 = False)
        result = whisper.decode(self.model, mel, options)
        self.get_logger().info(result.text)

        response.result = result.text
        return response


def main(args=None):
    rclpy.init(args=args)
    whisperNode = RosWhisperNode()
    rclpy.spin(whisperNode)
    whisperNode.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()