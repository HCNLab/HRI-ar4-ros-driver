#!/usr/bin/env python3
"""
Real-time EEG Visualizer (Visualization Only)

Shows EEG waveforms with markers in real-time.
For recording, use LSL Lab Recorder.

Keyboard Controls:
  ↑/↓ : Adjust amplitude scale
  ←/→ : Adjust channel spacing
  +/- : Adjust time window
  A   : Auto-scale to fit signals
  R   : Reset to defaults
  P   : Pause/Resume display
  H   : Show/Hide help
"""

import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import numpy as np
import sys
import threading
import time
from collections import deque
from datetime import datetime
from scipy import signal

try:
    from pylsl import StreamInlet, resolve_streams
except ImportError:
    print("Error: pip install pylsl")
    sys.exit(1)


# Configuration
EEG_STREAM = "UnicornEEG"
MARKER_STREAM = "UnityMarkers"
SFREQ = 250

CHANNELS = ['Fz', 'C3', 'Cz', 'C4', 'Pz', 'PO7', 'Oz', 'PO8']
COLORS = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3', '#ff7f00', '#a65628', '#f781bf', '#999999']


class DisplaySettings:
    def __init__(self):
        self.time_window = 8
        self.scale = 50.0
        self.channel_spacing = 1.0
        self.paused = False
        self.show_help = True
        
    @property
    def buffer_size(self):
        return int(SFREQ * self.time_window)


settings = DisplaySettings()

# Display buffers
MAX_BUFFER = SFREQ * 30
eeg_buffer = [deque([0]*MAX_BUFFER, maxlen=MAX_BUFFER) for _ in range(8)]
marker_times = deque(maxlen=50)
marker_texts = deque(maxlen=50)

data_lock = threading.Lock()
running = True
sample_count = 0
marker_count = 0


def design_filters():
    b_bp, a_bp = signal.butter(4, [1, 40], btype='bandpass', fs=SFREQ)
    b_notch, a_notch = signal.iirnotch(60, 30, fs=SFREQ)
    return b_bp, a_bp, b_notch, a_notch


def eeg_thread(inlet, filters):
    global running, sample_count
    b_bp, a_bp, b_notch, a_notch = filters
    
    zi_bp = [signal.lfilter_zi(b_bp, a_bp) * 0 for _ in range(8)]
    zi_notch = [signal.lfilter_zi(b_notch, a_notch) * 0 for _ in range(8)]
    
    while running:
        try:
            samples, _ = inlet.pull_chunk(timeout=0.05, max_samples=25)
            if samples:
                for sample in samples:
                    with data_lock:
                        for i in range(8):
                            val = sample[i]
                            val, zi_notch[i] = signal.lfilter(b_notch, a_notch, [val], zi=zi_notch[i])
                            val, zi_bp[i] = signal.lfilter(b_bp, a_bp, val, zi=zi_bp[i])
                            eeg_buffer[i].append(val[0])
                        sample_count += 1
        except:
            break


def marker_thread():
    global running, marker_count
    
    print(f"[Markers] Searching for '{MARKER_STREAM}'...")
    
    while running:
        streams = resolve_streams()
        matching = [s for s in streams if s.name() == MARKER_STREAM]
        
        if matching:
            inlet = StreamInlet(matching[0])
            print("[Markers] Connected!")
            
            while running:
                try:
                    sample, _ = inlet.pull_sample(timeout=0.1)
                    if sample and sample[0]:
                        marker = sample[0]
                        with data_lock:
                            marker_times.append(time.time())
                            parts = marker.split('|')
                            short = parts[1] if len(parts) > 1 else marker
                            marker_texts.append(short[:12])
                            marker_count += 1
                        print(f"[MARKER] {datetime.now().strftime('%H:%M:%S')} | {marker}")
                except:
                    break
        else:
            time.sleep(1.0)


def auto_scale():
    with data_lock:
        all_data = []
        for i in range(8):
            data = list(eeg_buffer[i])[-settings.buffer_size:]
            if data:
                all_data.extend(data)
        
        if all_data:
            std = np.std(all_data)
            if std > 0:
                settings.scale = max(5, std * 4)
                print(f"[Auto-scale] Scale: {settings.scale:.1f} µV/div")


def on_key(event):
    if event.key == 'up':
        settings.scale *= 0.8
        print(f"Scale: {settings.scale:.1f} µV/div")
    elif event.key == 'down':
        settings.scale *= 1.25
        print(f"Scale: {settings.scale:.1f} µV/div")
    elif event.key == 'left':
        settings.channel_spacing = max(0.3, settings.channel_spacing * 0.9)
    elif event.key == 'right':
        settings.channel_spacing = min(3.0, settings.channel_spacing * 1.1)
    elif event.key == '+' or event.key == '=':
        settings.time_window = min(30, settings.time_window + 2)
        print(f"Time window: {settings.time_window}s")
    elif event.key == '-':
        settings.time_window = max(2, settings.time_window - 2)
        print(f"Time window: {settings.time_window}s")
    elif event.key == 'a':
        auto_scale()
    elif event.key == 'r':
        settings.scale = 50.0
        settings.channel_spacing = 1.0
        settings.time_window = 8
        print("Reset to defaults")
    elif event.key == 'p':
        settings.paused = not settings.paused
        print("PAUSED" if settings.paused else "RESUMED")
    elif event.key == 'h':
        settings.show_help = not settings.show_help


def main():
    global running
    
    print("\n" + "="*60)
    print("EEG Visualizer (Visualization Only)")
    print("="*60)
    print("For recording, use LSL Lab Recorder!")
    print()
    print("Keyboard Controls:")
    print("  ↑/↓  : Amplitude scale")
    print("  ←/→  : Channel spacing")
    print("  +/-  : Time window (2-30s)")
    print("  A    : Auto-scale")
    print("  R    : Reset defaults")
    print("  P    : Pause/Resume")
    print("="*60 + "\n")
    
    print(f"[EEG] Looking for '{EEG_STREAM}'...")
    streams = resolve_streams()
    matching = [s for s in streams if s.name() == EEG_STREAM]
    
    if not matching:
        print("[EEG] Stream not found!")
        return
    
    eeg_inlet = StreamInlet(matching[0])
    print("[EEG] Connected!")
    
    filters = design_filters()
    
    t_eeg = threading.Thread(target=eeg_thread, args=(eeg_inlet, filters), daemon=True)
    t_marker = threading.Thread(target=marker_thread, daemon=True)
    t_eeg.start()
    t_marker.start()
    
    plt.ion()
    fig, ax = plt.subplots(figsize=(14, 8))
    fig.canvas.mpl_connect('key_press_event', on_key)
    
    lines = []
    for i in range(8):
        line, = ax.plot([], [], color=COLORS[i], linewidth=0.8, label=CHANNELS[i])
        lines.append(line)
    
    marker_objs = []
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right', fontsize=8, ncol=4)
    ax.grid(True, alpha=0.3)
    
    help_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, fontsize=8,
                        verticalalignment='top', fontfamily='monospace',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    scale_text = ax.text(0.98, 0.02, "", transform=ax.transAxes, fontsize=10,
                         horizontalalignment='right', fontfamily='monospace')
    
    plt.tight_layout()
    start_time = time.time()
    
    try:
        while plt.fignum_exists(fig.number):
            if not settings.paused:
                with data_lock:
                    n_samples = sample_count
                    n_markers = marker_count
                    
                    buffer_size = settings.buffer_size
                    x = np.arange(buffer_size) / SFREQ
                    
                    for i in range(8):
                        data = list(eeg_buffer[i])[-buffer_size:]
                        if len(data) < buffer_size:
                            data = [0] * (buffer_size - len(data)) + data
                        data = np.array(data)
                        lines[i].set_data(x, data / settings.scale + (7 - i) * settings.channel_spacing)
                    
                    ax.set_xlim(0, settings.time_window)
                    ax.set_ylim(-settings.channel_spacing, 8 * settings.channel_spacing)
                    ax.set_yticks([(7 - i) * settings.channel_spacing for i in range(8)])
                    ax.set_yticklabels(CHANNELS)
                    
                    for obj in marker_objs:
                        obj.remove()
                    marker_objs = []
                    
                    now = time.time()
                    for mt, txt in zip(marker_times, marker_texts):
                        age = now - mt
                        if 0 < age < settings.time_window:
                            x_pos = settings.time_window - age
                            marker_objs.append(ax.axvline(x=x_pos, color='red', linewidth=2, alpha=0.7))
                            marker_objs.append(ax.text(x_pos + 0.05, 8 * settings.channel_spacing - 0.2, txt,
                                                       fontsize=9, color='red', fontweight='bold'))
                
                elapsed = time.time() - start_time
                status = "PAUSED" if settings.paused else "Viewing"
                ax.set_title(f'{status}: {elapsed:.0f}s | Samples: {n_samples} | Markers: {n_markers}')
                
                scale_text.set_text(f"Scale: {settings.scale:.0f} µV/div | Window: {settings.time_window}s")
                
                if settings.show_help:
                    help_text.set_text("↑↓:Scale  ←→:Spacing  +/-:Time  A:Auto  R:Reset  P:Pause  H:Hide")
                else:
                    help_text.set_text("")
            
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.04)
            
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        time.sleep(0.2)
        plt.close('all')
        print("\nDone. Use LSL Lab Recorder for data recording!")


if __name__ == "__main__":
    main()
