#!/usr/bin/env python3
"""
LSL to OpenVibe Bridge with TCP Tagging

EEG: TCP port 12345 (Generic Raw Telnet Reader)
Markers: TCP port 15361 (OpenVibe TCP Tagging / Stimulation Listener)

OpenVibe will receive both EEG data and stimulations for combined recording.
"""

import socket
import struct
import sys
import time
import threading
from datetime import datetime

try:
    from pylsl import StreamInlet, resolve_streams
except ImportError:
    print("Error: pip install pylsl")
    sys.exit(1)


# Marker log
marker_log = []
marker_lock = threading.Lock()

# TCP Tagging socket
tagging_socket = None
tagging_connected = False

# Marker to OpenVibe stimulation code mapping
# OpenVibe OVTK_StimulationId codes start from 0x8001
STIM_CODES = {
    "SEQ_START": 0x8001,
    "SEQ_END": 0x8002,
    "MOVE_START": 0x8010,
    "MOVE_END": 0x8011,
    "PRESS_START": 0x8020,
    "PRESS_END": 0x8021,
    "HOLD_START": 0x8030,
    "HOLD_END": 0x8031,
    "RETURN_HOME": 0x8040,
}


def connect_tagging():
    """Connect to OpenVibe TCP Tagging port"""
    global tagging_socket, tagging_connected
    
    try:
        tagging_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tagging_socket.connect(('localhost', 15361))
        tagging_connected = True
        print("[Tagging] Connected to OpenVibe TCP Tagging (port 15361)")
        return True
    except Exception as e:
        print(f"[Tagging] Could not connect to port 15361: {e}")
        print("[Tagging] Start OpenVibe Acquisition Server first!")
        tagging_connected = False
        return False


def send_stimulation(stim_code):
    """Send stimulation to OpenVibe via TCP Tagging"""
    global tagging_socket, tagging_connected
    
    if not tagging_connected:
        return
    
    try:
        # OpenVibe TCP Tagging format: 8 bytes
        # 4 bytes: padding (zeros)
        # 4 bytes: stimulation ID (little-endian uint32)
        data = struct.pack('<II', 0, stim_code)
        tagging_socket.sendall(data)
    except:
        tagging_connected = False


def marker_to_stim_code(marker):
    """Convert marker string to OpenVibe stimulation code"""
    parts = marker.split("|")
    if len(parts) >= 2:
        event_type = parts[1]
        return STIM_CODES.get(event_type, 0x8099)
    return 0x8099


def marker_receiver_thread():
    """Receive Unity markers and send to OpenVibe"""
    print("[Markers] Searching for 'UnityMarkers' stream...")
    
    while True:
        streams = resolve_streams()
        matching = [s for s in streams if s.name() == "UnityMarkers"]
        
        if matching:
            inlet = StreamInlet(matching[0])
            print("[Markers] Connected to UnityMarkers stream!")
            
            while True:
                try:
                    sample, timestamp = inlet.pull_sample(timeout=0.1)
                    if sample and sample[0]:
                        marker = sample[0]
                        stim_code = marker_to_stim_code(marker)
                        time_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        
                        # Send to OpenVibe TCP Tagging
                        send_stimulation(stim_code)
                        
                        with marker_lock:
                            marker_log.append((timestamp, marker, stim_code))
                        
                        print(f"[STIM 0x{stim_code:04X}] {time_str} | {marker}")
                except:
                    break
        else:
            time.sleep(2.0)


def main():
    print("\n" + "="*60)
    print("LSL → OpenVibe Bridge (EEG + TCP Tagging)")
    print("="*60 + "\n")
    
    stream_name = "UnicornEEG"
    tcp_port = 12345
    num_channels = 8
    
    # Start marker receiver
    marker_thread = threading.Thread(target=marker_receiver_thread, daemon=True)
    marker_thread.start()
    
    # Find EEG LSL stream
    print(f"[EEG] Looking for '{stream_name}'...")
    streams = resolve_streams()
    matching = [s for s in streams if s.name() == stream_name]
    
    if not matching:
        print("[EEG] Stream not found!")
        return
    
    inlet = StreamInlet(matching[0])
    print(f"[EEG] Connected!")
    
    # TCP server for EEG
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('localhost', tcp_port))
    server.listen(1)
    
    print(f"\n>>> OpenVibe Settings <<<")
    print(f"1. Acquisition Server:")
    print(f"   - Driver: Generic Raw Telnet Reader")
    print(f"   - Channels: {num_channels}")
    print(f"   - Port: {tcp_port}")
    print(f"2. Designer: Add 'Stimulation Listener' box")
    print(f"   - Port: 15361 (default)")
    print(f"\n[TCP] Waiting for OpenVibe connection...")
    
    client, addr = server.accept()
    print(f"[TCP] OpenVibe connected!")
    
    # Connect to TCP Tagging
    connect_tagging()
    
    print("-"*60)
    
    sample_count = 0
    start_time = time.time()
    
    try:
        while True:
            samples, _ = inlet.pull_chunk(timeout=0.01, max_samples=10)
            
            if samples:
                for sample in samples:
                    data = struct.pack(f'{num_channels}f', *sample[:num_channels])
                    try:
                        client.sendall(data)
                        sample_count += 1
                    except:
                        print("[TCP] Connection lost!")
                        return
            
            if sample_count > 0 and sample_count % 1250 == 0:
                elapsed = time.time() - start_time
                rate = sample_count / elapsed
                with marker_lock:
                    mc = len(marker_log)
                tag_status = "✓" if tagging_connected else "✗"
                print(f"[Status] Samples: {sample_count} | Rate: {rate:.1f} Hz | Markers: {mc} | Tagging: {tag_status}")
                
    except KeyboardInterrupt:
        print("\n" + "-"*60)
    finally:
        client.close()
        server.close()
        if tagging_socket:
            tagging_socket.close()
        
        if marker_log:
            filename = f"markers_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            with open(filename, 'w') as f:
                f.write("timestamp,marker,stim_code\n")
                for ts, mk, sc in marker_log:
                    f.write(f"{ts},{mk},0x{sc:04X}\n")
            print(f"Markers saved: {filename}")
        
        print("Done.")


if __name__ == "__main__":
    main()
