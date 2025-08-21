#!/usr/bin/env python3
"""
ble_scan_top_influx.py

Every 5 s scans for nearby BLE advertisements, takes the strongest RSSI value
and writes *only* that value to InfluxDB 2.x.

Environment variables:
  INFLUX_TOKEN  – write‑permission token
  INFLUX_ORG    – organisation name   (default: casa0014)
  INFLUX_BUCKET – bucket name         (default: ble)
"""

import os
import asyncio
import datetime
import time
from bleak import BleakScanner
from influxdb_client import InfluxDBClient, Point, WriteOptions

# InfluxDB connection ---------------------------------------------------------
INFLUX_URL    = "http://100.82.123.4:8086"
INFLUX_TOKEN  = os.environ["INFLUX_TOKEN"]
INFLUX_ORG    = os.environ.get("INFLUX_ORG", "casa0014")
INFLUX_BUCKET = os.environ.get("INFLUX_BUCKET", "ble")

client    = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
write_api = client.write_api(
    write_options=WriteOptions(batch_size=1, flush_interval=1000)
)

# Settings --------------------------------------------------------------------
SCAN_INTERVAL = 5.0                     # seconds
PROBE_ID      = os.uname().nodename     # hostname tag

# Helper ----------------------------------------------------------------------
def make_point(rssi: int) -> Point:
    """Return an InfluxDB point containing the RSSI value only."""
    return (
        Point("ble_rssi")
        .tag("probe", PROBE_ID)
        .field("rssi", rssi)
        .time(time.time_ns())
    )

# Main loop -------------------------------------------------------------------
async def scan_loop() -> None:
    while True:
        print(f"\n========== {datetime.datetime.now():%Y-%m-%d %H:%M:%S} ==========")

        devices = await BleakScanner.discover(
            timeout=SCAN_INTERVAL, return_adv=True
        )

        if not devices:
            print("No BLE devices detected")
            continue

        # Strongest RSSI value only
        _, adv = max(devices.values(), key=lambda t: t[1].rssi)
        print(f"Strongest RSSI: {adv.rssi} dBm")

        write_api.write(
            bucket=INFLUX_BUCKET,
            org=INFLUX_ORG,
            record=make_point(adv.rssi),
        )

# Entry point -----------------------------------------------------------------
if __name__ == "__main__":
    try:
        asyncio.run(scan_loop())
    except KeyboardInterrupt:
        print("\nScan stopped")
    finally:
        write_api.close()
        client.close()
