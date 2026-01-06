# %%
"""
GDF EEG Analysis Script for HRI Experiment

Loads GDF files recorded with OpenViBE, applies filters,
and provides event-based analysis for ErrP detection.
"""

import numpy as np
from pathlib import Path
import mne

mne.set_log_level('WARNING')

# Channel configuration for Unicorn EEG (8 channels)
# NOTE: OpenViBE GDF usually saves channels in order. Rename if necessary.
CHANNELS = ['Fz', 'C3', 'Cz', 'C4', 'Pz', 'PO7', 'Oz', 'PO8']
MONTAGE = 'standard_1020'

# %%
# Marker code definitions (matching Unity/Python)
# In OpenViBE GDF, markers are typically saved as integer codes.
# Standard OVTK_StimulationId_Label_XX codes are mapped to integers.
# However, if you used 'LSL Marker Input', the codes might be passed directly.
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
    '122': 'ROBOT_PRESS_MISS',
    '123': 'ROBOT_COMPLETE_ERR',
    '124': 'ROBOT_PRESS_AIR',
    '125': 'ROBOT_PRESS_FREEZE',
    '126': 'ROBOT_PRESS_DOUBLE',
    '127': 'ROBOT_PRESS_ABORT',
    # OpenViBE specific mappings (if OVTK codes are used)
    # Example: 33025 = OVTK_StimulationId_Label_01 -> mapped to 1
    # You might need to check your GDF file to see actual codes
}

# %% [markdown]
# ## 1. Load GDF File

# %%
filepath = Path("record-[2025.12.30-13.17.08].edf")  # <- Set your file path here

# %%
# Load EDF using MNE
# eog=None, misc=None, stim_channel='auto' usually works
if not filepath.exists():
    print(f"File not found: {filepath}")
    # Create dummy data for demonstration if file missing
    print("Creating dummy data...")
    info = mne.create_info(CHANNELS, 250, 'eeg')
    raw = mne.io.RawArray(np.random.randn(8, 250*10)*1e-5, info)
else:
    # preload=True to load data into memory
    print(f"Loading {filepath}...")
    try:
        raw = mne.io.read_raw_edf(filepath, preload=True)
    except Exception as e:
        print(f"Error loading EDF with standard reader: {e}")
        # Sometimes EDF files from OpenViBE need specific handling
        print("Trying with different parameters...")
        raw = mne.io.read_raw_edf(filepath, preload=True, stim_channel='auto')

print(raw)
print(f"Channels: {raw.ch_names}")
print(f"Duration: {raw.times[-1]:.1f}s")

# %% [markdown]
# ## 2. Set Channel Names (if needed)

# %%
# Check if channel names match expected Unicorn channels
# OpenViBE might save them as 'Channel 1', 'Channel 2' etc.
if len(raw.ch_names) >= 8:
    # Rename first 8 channels to our standard names
    mapping = {old: new for old, new in zip(raw.ch_names[:8], CHANNELS)}
    print(f"Renaming channels: {mapping}")
    try:
        raw.rename_channels(mapping)
    except Exception as e:
        print(f"Channel renaming warning: {e}")

# Set montage
montage = mne.channels.make_standard_montage(MONTAGE)
raw.set_montage(montage, on_missing='ignore')

# %% [markdown]
# ## 3. Extract Events (Markers)

# %%
# MNE automatically reads GDF events/annotations
events, event_id = mne.events_from_annotations(raw)

print(f"\nFound {len(events)} events:")
print(f"Event ID mapping: {event_id}")

# %% [markdown]
# ## 4. Map Event Codes to Readable Names

# %%
# Inverse map event_id to find description for each code
# In EDF from OpenViBE, codes appear as "OVTK_StimulationId_Label_XX" where XX is hex

def parse_ovtk_label(desc):
    """
    Parse OpenViBE OVTK_StimulationId_Label_XX format
    The XX part is hexadecimal representing our original marker code
    Example: OVTK_StimulationId_Label_15 -> 0x15 = 21 (ROBOT_PRESS_CYL)
    """
    if 'OVTK_StimulationId_Label_' in desc:
        hex_part = desc.split('_')[-1]  # Get the XX part
        try:
            original_code = int(hex_part, 16)  # Convert hex to decimal
            return str(original_code)
        except ValueError:
            return None
    elif 'OVTK_StimulationId_Number_' in desc:
        # Number format uses decimal
        num_part = desc.split('_')[-1]
        try:
            return num_part
        except:
            return None
    return None

readable_events = []
for ev in events:
    onset = ev[0] / raw.info['sfreq']
    mne_id = str(ev[2])
    
    # Find key in event_id that has value == ev[2]
    name = "UNKNOWN"
    original_code = mne_id
    
    for desc, idx in event_id.items():
        if idx == ev[2]:
            # Try to parse OVTK format
            parsed = parse_ovtk_label(desc)
            if parsed:
                original_code = parsed
                if parsed in MARKER_CODES:
                    name = MARKER_CODES[parsed]
                else:
                    name = f"CODE_{parsed}"
            elif desc in MARKER_CODES:
                original_code = desc
                name = MARKER_CODES[desc]
            else:
                name = desc
            break
            
    readable_events.append((onset, original_code, name))

print("\nFirst 15 decoded events:")
for t, c, n in readable_events[:15]:
    print(f"  {t:8.2f}s : Code {c:>3} = {n}")

# %% [markdown]
# ## 5. Update Annotations for Plots

# %%
# Replace OVTK labels with readable names in Raw annotations
# This makes plots show readable marker names
if raw.annotations is not None and len(raw.annotations) > 0:
    new_descriptions = []
    for desc in raw.annotations.description:
        parsed = parse_ovtk_label(desc)
        if parsed:
            if parsed in MARKER_CODES:
                new_descriptions.append(MARKER_CODES[parsed])
            else:
                new_descriptions.append(f"CODE_{parsed}")
        elif desc in MARKER_CODES:
            new_descriptions.append(MARKER_CODES[desc])
        else:
            new_descriptions.append(desc)
    
    # Create new annotations with readable names
    new_annotations = mne.Annotations(
        onset=raw.annotations.onset,
        duration=raw.annotations.duration,
        description=new_descriptions
    )
    raw.set_annotations(new_annotations)
    print(f"\nUpdated {len(new_descriptions)} annotations with readable names")

# %% [markdown]
# ## 6. Filtering

# %%
# Convert to float if not already (edf might be int16/float32)
if raw.orig_format != 'double':
    # raw is already float64 after loading with MNE usually
    pass

# Apply filters
raw_notch = raw.copy().notch_filter(freqs=60)
raw_filtered = raw_notch.copy().filter(l_freq=1, h_freq=40)
print("Filtering complete: Notch 60Hz + Bandpass 1-40Hz")

# %% [markdown]
# ## 6. Plot EEG

# %%
raw_filtered.plot(scalings=dict(eeg=100e-6), duration=10, title='Filtered EEG (GDF)')

# %% [markdown]
# ## 7. ErrP Epoch Analysis

# %%
# Define event codes to analyze (our original codes, not OVTK)
errp_code = '121' # ROBOT_PRESS_WRONG
normal_codes = ['21', '22'] # CYL, CUBE

# Find corresponding MNE event IDs by parsing OVTK labels
target_event_ids = {}

for desc, idx in event_id.items():
    # Parse OVTK format to get original code
    parsed = parse_ovtk_label(desc)
    code_to_check = parsed if parsed else desc
    
    if code_to_check == errp_code:
        target_event_ids['ErrP/Wrong'] = idx
        print(f"  Matched ErrP: {desc} -> code {errp_code}")
    elif code_to_check in normal_codes:
        target_event_ids[f'Normal/{code_to_check}'] = idx
        print(f"  Matched Normal: {desc} -> code {code_to_check}")

print(f"\nTarget Event IDs for Epoching: {target_event_ids}")

if not target_event_ids:
    print("WARNING: No matching event codes found! Check 'Event ID mapping' above.")
    print("You may need to update MARKER_CODES or check how OpenViBE saved them.")

# %%
if target_event_ids:
    epochs = mne.Epochs(raw_filtered, events, event_id=target_event_ids,
                        tmin=-0.2, tmax=0.8, baseline=(-0.2, 0),
                        preload=True, reject=dict(eeg=150e-6))
    
    print("\nEpochs summary:")
    print(epochs)

    # %% [markdown]
    # ## 8. ERP Comparison

    # %%
    # Plot average (ERP)
    if len(epochs) > 0:
        evoked_dict = {label: epochs[label].average() for label in target_event_ids.keys() if label in epochs}
        
        if evoked_dict:
            # Plot comparison at Fz
            mne.viz.plot_compare_evokeds(evoked_dict, picks='Fz', title='ERP Comparison at Fz')
            
            # Plot image of all epochs for ErrP
            if 'ErrP/Wrong' in epochs:
                epochs['ErrP/Wrong'].plot_image(picks='Fz', title='ErrP Epochs (Fz)')
else:
    print("Skipping epoch analysis due to missing events.")