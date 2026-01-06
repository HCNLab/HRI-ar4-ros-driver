#!/usr/bin/env python3
"""
Unicorn BCI Core-8 LSL Streaming (NO GUI)
Uses g.Pype SDK

Run from VS Code (F5)
"""

import gpype as gp
import time

if __name__ == "__main__":
    print("\n" + "="*60)
    print("Unicorn BCI Core-8 LSL Streamer")
    print("="*60)
    print("\nMake sure Unicorn is powered ON.\n")
    
    # Create pipeline
    p = gp.Pipeline()
    
    # Create BCI Core-8 source
    source = gp.BCICore8(serial=None, channel_count=8)
    
    # Create LSL sender
    lsl = gp.LSLSender(stream_name="UnicornEEG")
    
    # Connect
    p.connect(source, lsl)
    
    print("Starting LSL stream: 'UnicornEEG'")
    print("8 channels @ 250 Hz")
    print("Press Ctrl+C to stop\n")
    
    try:
        p.start()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        p.stop()
        print("Done.")
