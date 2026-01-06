# %%
"""
XDF EEG Analysis Script for HRI Experiment

Loads XDF files recorded with LSL Lab Recorder, applies filters,
and provides event-based analysis for ErrP detection.
"""

import numpy as np
from pathlib import Path
import pyxdf
import mne

mne.set_log_level('WARNING')

# Channel configuration for Unicorn EEG
CHANNELS = ['Fz', 'C3', 'Cz', 'C4', 'Pz', 'PO7', 'Oz', 'PO8']
MONTAGE = 'standard_1020'

# %%
# Marker code definitions (matching Unity/Python)
MARKER_CODES = {
    '1': 'TRIAL_START',
    '9': 'TRIAL_END',
    '10': 'ASSEMBLY_COMPLETE',
    '11': 'BUTTON_PRESS',
    '20': 'ROBOT_START',
    '21': 'ROBOT_PRESS_CYL',
    '22': 'ROBOT_PRESS_CUBE',
    '23': 'ROBOT_COMPLETE',
    '30': 'FEEDBACK_CORRECT',
    '31': 'FEEDBACK_WRONG',
    '32': 'FEEDBACK_ERROR',
    '120': 'ROBOT_START_ERR',
    '121': 'ROBOT_PRESS_WRONG',  # ErrP target
    '123': 'ROBOT_COMPLETE_ERR',
}

def parse_marker(marker_text):
    """Parse marker text and return code and description"""
    # Format: "S 21" or "S  1|T1"
    text = marker_text.strip()
    if text.startswith('S'):
        parts = text[1:].split('|')
        code = parts[0].strip()
        extra = parts[1] if len(parts) > 1 else ''
        name = MARKER_CODES.get(code, f'UNKNOWN_{code}')
        return code, name, extra
    return text, text, ''

# %% [markdown]
# ## 1. Load XDF File

# %%
filepath = Path("recording.xdf")  # <- Set your file path here

# %%
streams, header = pyxdf.load_xdf(str(filepath))

print(f"Found {len(streams)} streams:")
for i, s in enumerate(streams):
    name = s['info']['name'][0]
    stype = s['info']['type'][0]
    n_samples = len(s['time_series']) if 'time_series' in s else 0
    print(f"  [{i}] {name} ({stype}) - {n_samples} samples")

# %% [markdown]
# ## 2. Extract EEG Stream

# %%
def find_stream(streams, keyword):
    for s in streams:
        if keyword.lower() in s['info']['name'][0].lower():
            return s
    return None

eeg_stream = find_stream(streams, 'EEG') or find_stream(streams, 'Unicorn')

eeg_data = np.array(eeg_stream['time_series']).T
eeg_times = np.array(eeg_stream['time_stamps'])
sfreq = float(eeg_stream['info']['nominal_srate'][0])

print(f"EEG: {eeg_data.shape[0]} ch, {eeg_data.shape[1]} samples, {sfreq} Hz")
print(f"Duration: {eeg_data.shape[1] / sfreq:.1f}s")

# %% [markdown]
# ## 3. Extract and Parse Markers

# %%
marker_stream = find_stream(streams, 'Marker') or find_stream(streams, 'Unity')

markers = []
if marker_stream:
    marker_data = marker_stream['time_series']
    marker_times = marker_stream['time_stamps']
    eeg_start = eeg_times[0]
    
    for data, ts in zip(marker_data, marker_times):
        text = str(data[0]) if isinstance(data, (list, np.ndarray)) else str(data)
        onset = ts - eeg_start
        if onset >= 0:
            code, name, extra = parse_marker(text)
            markers.append((onset, code, name, extra))
    
    print(f"\nFound {len(markers)} markers:")
    for onset, code, name, extra in markers[:15]:
        extra_str = f" ({extra})" if extra else ""
        print(f"  {onset:8.2f}s : S{code:>3} = {name}{extra_str}")
    if len(markers) > 15:
        print(f"  ... and {len(markers) - 15} more")

# %% [markdown]
# ## 4. Create MNE Raw Object

# %%
n_channels = min(8, eeg_data.shape[0])
eeg_data = eeg_data[:n_channels]
ch_names = CHANNELS[:n_channels]

eeg_data_v = eeg_data * 1e-6  # uV -> V

info = mne.create_info(ch_names=ch_names, sfreq=sfreq, ch_types='eeg')
raw = mne.io.RawArray(eeg_data_v, info)

montage = mne.channels.make_standard_montage(MONTAGE)
raw.set_montage(montage, on_missing='ignore')

print(raw)

# %%
# Add annotations with readable names
if markers:
    annotations = mne.Annotations(
        onset=[m[0] for m in markers],
        duration=[0.0] * len(markers),
        description=[m[2] for m in markers]  # Use parsed name
    )
    raw.set_annotations(annotations)
    print(f"Added {len(markers)} annotations")

# %% [markdown]
# ## 5. Filtering

# %%
raw_notch = raw.copy().notch_filter(freqs=60)
raw_filtered = raw_notch.copy().filter(l_freq=1, h_freq=40)
print("Filtering complete: Notch 60Hz + Bandpass 1-40Hz")

# %% [markdown]
# ## 6. Plot EEG

# %%
raw_filtered.plot(scalings=dict(eeg=100e-6), duration=10, title='Filtered EEG')

# %% [markdown]
# ## 7. Event Analysis

# %%
# Get events from annotations
events, event_id = mne.events_from_annotations(raw_filtered)
print(f"Events: {len(events)}")
print(f"Event IDs: {event_id}")

# %% [markdown]
# ## 8. ErrP Epoch Analysis

# %%
# Create epochs around ErrP markers (ROBOT_PRESS_WRONG = 121)
errp_events = ['ROBOT_PRESS_WRONG']
normal_events = ['ROBOT_PRESS_CYL', 'ROBOT_PRESS_CUBE']

# Check if ErrP events exist
errp_ids = {k: v for k, v in event_id.items() if k in errp_events}
normal_ids = {k: v for k, v in event_id.items() if k in normal_events}

print(f"ErrP events: {errp_ids}")
print(f"Normal events: {normal_ids}")

# %%
# Create epochs for ErrP analysis
if errp_ids:
    epochs_errp = mne.Epochs(raw_filtered, events, event_id=errp_ids,
                              tmin=-0.2, tmax=0.8, baseline=(-0.2, 0), 
                              preload=True, reject=dict(eeg=150e-6))
    print(f"ErrP epochs: {len(epochs_errp)}")

# %%
if normal_ids:
    epochs_normal = mne.Epochs(raw_filtered, events, event_id=normal_ids,
                                tmin=-0.2, tmax=0.8, baseline=(-0.2, 0),
                                preload=True, reject=dict(eeg=150e-6))
    print(f"Normal epochs: {len(epochs_normal)}")

# %% [markdown]
# ## 9. ERP Comparison (ErrP vs Normal)

# %%
# Plot ERPs if both exist
if errp_ids and normal_ids and len(epochs_errp) > 0 and len(epochs_normal) > 0:
    evoked_errp = epochs_errp.average()
    evoked_normal = epochs_normal.average()
    
    # Compare at Fz (frontal - typical ErrP location)
    mne.viz.plot_compare_evokeds({'Error': evoked_errp, 'Normal': evoked_normal},
                                  picks='Fz', title='ErrP Comparison at Fz')
