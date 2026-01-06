#!/usr/bin/env python3
"""
EEG Recorder with Event Markers

Records EEG from LSL and markers from Unity, saves to EDF+ format.
No OpenVibe required!

Usage:
    python eeg_recorder.py
    
Output:
    eeg_recording_YYYYMMDD_HHMMSS.edf (EEG + annotations)
"""

import sys
import time
import threading
import numpy as np
from datetime import datetime
from pathlib import Path

try:
    from pylsl import StreamInlet, resolve_streams
except ImportError:
    print("Error: pip install pylsl")
    sys.exit(1)

try:
    import pyedflib
except ImportError:
    print("Error: pip install pyedflib")
    sys.exit(1)


# Configuration
EEG_STREAM_NAME = "UnicornEEG"
MARKER_STREAM_NAME = "UnityMarkers"
SFREQ = 250  # Hz
NUM_CHANNELS = 8
CHANNEL_NAMES = ['Fz', 'C3', 'Cz', 'C4', 'Pz', 'PO7', 'Oz', 'PO8']

# Data buffers
eeg_data = []
eeg_timestamps = []
markers = []
data_lock = threading.Lock()
recording = True


def eeg_receiver_thread(inlet):
    """Receive EEG data from LSL"""
    global recording
    
    print("[EEG] Recording started...")
    
    while recording:
        try:
            samples, timestamps = inlet.pull_chunk(timeout=0.1, max_samples=25)
            if samples:
                with data_lock:
                    eeg_data.extend(samples)
                    eeg_timestamps.extend(timestamps)
        except:
            break


def marker_receiver_thread():
    """Receive markers from Unity LSL"""
    global recording
    
    print(f"[Markers] Searching for '{MARKER_STREAM_NAME}'...")
    
    while recording:
        streams = resolve_streams()
        matching = [s for s in streams if s.name() == MARKER_STREAM_NAME]
        
        if matching:
            inlet = StreamInlet(matching[0])
            print("[Markers] Connected!")
            
            while recording:
                try:
                    sample, timestamp = inlet.pull_sample(timeout=0.1)
                    if sample and sample[0]:
                        marker = sample[0]
                        time_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        
                        with data_lock:
                            markers.append((timestamp, marker))
                        
                        print(f"[MARKER] {time_str} | {marker}")
                except:
                    break
        else:
            time.sleep(2.0)


def save_edf(filename):
    """Save EEG data and markers to EDF+ format"""
    
    if not eeg_data:
        print("No EEG data to save!")
        return False
    
    # Convert to numpy array
    data = np.array(eeg_data).T  # Shape: (channels, samples)
    
    # Create EDF file
    print(f"\n[Save] Creating EDF file: {filename}")
    print(f"       Duration: {data.shape[1] / SFREQ:.1f} seconds")
    print(f"       Samples: {data.shape[1]}")
    print(f"       Markers: {len(markers)}")
    
    f = pyedflib.EdfWriter(str(filename), NUM_CHANNELS, file_type=pyedflib.FILETYPE_EDFPLUS)
    
    try:
        # Set file header
        f.setTechnician("EEG Recorder")
        f.setRecordingAdditional("HRI Experiment")
        
        # Set channel headers
        for i, ch_name in enumerate(CHANNEL_NAMES):
            f.setSignalHeader(i, {
                'label': ch_name,
                'dimension': 'uV',
                'sample_frequency': SFREQ,
                'physical_min': -500.0,
                'physical_max': 500.0,
                'digital_min': -32768,
                'digital_max': 32767,
                'transducer': 'AgAgCl electrode',
                'prefilter': 'HP:0.1Hz LP:100Hz',
            })
        
        # Write EEG data
        f.writeSamples(data)
        
        # Write annotations (markers)
        if eeg_timestamps:
            start_time = eeg_timestamps[0]
            
            for ts, marker in markers:
                # Calculate relative time from recording start
                onset = ts - start_time
                if onset >= 0:
                    f.writeAnnotation(onset, 0.1, marker)
                    
        print("[Save] EDF saved successfully!")
        return True
        
    finally:
        f.close()


def main():
    global recording
    
    print("\n" + "="*60)
    print("EEG Recorder (Python + pyedflib)")
    print("="*60)
    print("\nPress Ctrl+C to stop and save.\n")
    
    # Find EEG stream
    print(f"[EEG] Looking for '{EEG_STREAM_NAME}'...")
    streams = resolve_streams()
    matching = [s for s in streams if s.name() == EEG_STREAM_NAME]
    
    if not matching:
        print("[EEG] Stream not found! Run unicorn_lsl_stream.py first.")
        return
    
    eeg_inlet = StreamInlet(matching[0])
    print(f"[EEG] Connected!")
    
    # Start threads
    eeg_thread = threading.Thread(target=eeg_receiver_thread, args=(eeg_inlet,), daemon=True)
    marker_thread = threading.Thread(target=marker_receiver_thread, daemon=True)
    
    eeg_thread.start()
    marker_thread.start()
    
    # Recording loop
    start_time = time.time()
    try:
        while True:
            time.sleep(1.0)
            elapsed = time.time() - start_time
            with data_lock:
                n_samples = len(eeg_data)
                n_markers = len(markers)
            
            rate = n_samples / elapsed if elapsed > 0 else 0
            print(f"[Recording] {elapsed:.0f}s | Samples: {n_samples} | Rate: {rate:.1f} Hz | Markers: {n_markers}")
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
        recording = False
        time.sleep(0.5)
    
    # Save file
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = Path(f"eeg_recording_{timestamp}.edf")
    save_edf(filename)
    
    print(f"\n✅ Recording complete: {filename}")


if __name__ == "__main__":
    main()
