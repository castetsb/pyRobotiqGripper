import numpy as np
import sounddevice as sd
import time
import threading

class Bipper:
    def __init__(self, sample_rate=44100, volume=0.3, audio_pitch=600.0, min_beep_rate=1.5, max_beep_rate=12.0):
        # Audio configuration constants
        self.sample_rate = sample_rate
        
        # Protected variables altered dynamically by main thread
        self._volume = volume
        self._audio_pitch = audio_pitch
        self._min_beep_rate = min_beep_rate
        self._max_beep_rate = max_beep_rate
        self._input_signal = 0.0
        
        # State tracking for smooth audio phase continuity
        self._audio_phase = 0.0
        self._beep_phase = 0.0
        
        # Lock to protect shared variables across threads
        self._lock = threading.Lock()
        self._stream = None

    # --- Thread-Safe Getters and Setters ---

    @property
    def input_signal(self):
        with self._lock: return self._input_signal
    @input_signal.setter
    def input_signal(self, value):
        with self._lock: self._input_signal = max(0.0, min(1.0, float(value)))

    @property
    def audio_pitch(self):
        with self._lock: return self._audio_pitch
    @audio_pitch.setter
    def audio_pitch(self, value):
        with self._lock: self._audio_pitch = max(20.0, float(value))  # Human hearing floor

    @property
    def min_beep_rate(self):
        with self._lock: return self._min_beep_rate
    @min_beep_rate.setter
    def min_beep_rate(self, value):
        with self._lock: self._min_beep_rate = max(0.1, float(value))

    @property
    def max_beep_rate(self):
        with self._lock: return self._max_beep_rate
    @max_beep_rate.setter
    def max_beep_rate(self, value):
        with self._lock: self._max_beep_rate = max(0.1, float(value))

    @property
    def volume(self):
        with self._lock: return self._volume
    @volume.setter
    def volume(self, value):
        with self._lock: self._volume = max(0.0, min(1.0, float(value)))

    # --- Core Audio Engine ---

    def _audio_callback(self, outdata, frames, time_info, status):
        """Runs continuously in a dedicated background audio thread."""
        # Thread-safe snapshot of parameters for this buffer slice
        with self._lock:
            signal = self._input_signal
            pitch = self._audio_pitch
            min_rate = self._min_beep_rate
            max_rate = self._max_beep_rate
            vol = self._volume

        # 1. Generate continuous pitch wave
        audio_phase_inc = 2 * np.pi * pitch / self.sample_rate
        audio_phases = self._audio_phase + np.arange(1, frames + 1) * audio_phase_inc
        wave = np.sin(audio_phases)
        
        # 2. Calculate current dynamic beep rate
        beep_rate = min_rate + (signal * (max_rate - min_rate))
        beep_phase_inc = 2 * np.pi * beep_rate / self.sample_rate
        beep_phases = self._beep_phase + np.arange(1, frames + 1) * beep_phase_inc
        
        # 3. Create envelope
        envelope = (np.sin(beep_phases) > 0).astype(float)
        
        # Process and write to output
        audio = wave * envelope * vol
        outdata[:] = audio.reshape(-1, 1)

        # Retain phase state across blocks
        self._audio_phase = audio_phases[-1] % (2 * np.pi)
        self._beep_phase = beep_phases[-1] % (2 * np.pi)

    def start(self):
        """Starts background audio playback thread."""
        if self._stream is None:
            self._stream = sd.OutputStream(
                samplerate=self.sample_rate, 
                channels=1, 
                callback=self._audio_callback
            )
            self._stream.start()

    def stop(self):
        """Stops background audio playback thread."""
        if self._stream is not None:
            self._stream.stop()
            self._stream.close()
            self._stream = None

'''
# ---------------------------------------------------------
# Main Application Test Run
# ---------------------------------------------------------
if __name__ == "__main__":
    bipper = Bipper()
    bipper.start()

    try:
        print("Testing dynamic updates from main thread. Press Ctrl+C to stop.\n")
        
        # Step 1: Modulate Input Signal (0.0 -> 1.0)
        print("--> Modulating signal (speeding up beeps)...")
        for val in np.linspace(0.0, 1.0, 30):
            bipper.input_signal = val
            time.sleep(0.1)

        # Step 2: Dynamically shift base pitch mid-stream
        print("--> Shifting base pitch higher...")
        bipper.audio_pitch = 900.0
        time.sleep(1.5)

        # Step 3: Change the beep scaling frequencies entirely 
        print("--> Scaling up maximum possible beep speed...")
        bipper.max_beep_rate = 25.0
        time.sleep(2.0)

        # Step 4: Drop pitch and slow everything back down
        print("--> Resetting to low pitch and slow speeds...")
        bipper.audio_pitch = 400.0
        bipper.max_beep_rate = 12.0
        bipper.input_signal = 0.1
        time.sleep(2.0)
            
    except KeyboardInterrupt:
        pass
    finally:
        bipper.stop()
        print("Stopped.")
'''