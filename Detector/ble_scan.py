#!/usr/bin/env python3
"""
ble_scan_top.py
Every 5 seconds scans nearby BLE devices and prints ONLY the one with the
strongest RSSI.  Run: python3 ble_scan_top.py
"""

import asyncio
import datetime
from bleak import BleakScanner

SCAN_INTERVAL = 5.0  # scan duration in seconds

async def scan_loop():
    while True:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"\n========== {timestamp} ==========")

        # discover() blocks for SCAN_INTERVAL seconds
        devices = await BleakScanner.discover(timeout=SCAN_INTERVAL)

        if not devices:
            print("No BLE devices detected")
            continue

        # Pick the device with the highest (least‑negative) RSSI
        strongest = max(devices, key=lambda d: d.rssi)
        name = strongest.name or "Unknown"
        print(f"Strongest → {strongest.address}  RSSI: {strongest.rssi:>4} dBm  "
              f"Name: {name}")

        # If you prefer the loop’s *total* period to stay at 5 s exactly,
        # add: await asyncio.sleep(max(0, SCAN_INTERVAL - processing_time))

if __name__ == "__main__":
    try:
        asyncio.run(scan_loop())
    except KeyboardInterrupt:
        print("\nScan stopped")
