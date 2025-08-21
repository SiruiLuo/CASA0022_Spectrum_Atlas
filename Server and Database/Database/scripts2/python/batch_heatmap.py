#!/usr/bin/env python3
import subprocess

signal_types = [
    "RC Low Band",
    "RC Aircraft",
    "RC Ground",
    "TETRA / Emergency",
    "LoRa / ISM 433",
    "ISM 868",
    "GSM 900 UL",
    "GSM 900 DL",
    "FM Radio",
    "Airband (AM)",
    "AIS / Marine",
    "ADS-B 1090",
    "LTE 1800"
]


session_id = "MarshgateReal_20250714_172855"


script_path = "/home/pi/scripts2/python/heatmap.py"

for signal in signal_types:
    print(f"\n? Generating heatmap for: {signal}")
    try:
        subprocess.run(
            ["python3", script_path, signal, session_id],
            check=True
        )
    except subprocess.CalledProcessError as e:
        print(f" Failed to generate heatmap for: {signal}")
        print("   Error:", e)
