#!/usr/bin/env python3
import os, yaml, time, signal, datetime as dt, sys
import psycopg2, psycopg2.extras
import numpy as np
from rtlsdr import RtlSdr
import rospy
from geometry_msgs.msg import PoseStamped

# ────────── 1. Load configuration ──────────
cfg_path = '/home/Steven/Spectrum_Atlas_GUI2/config.yaml'
cfg = yaml.safe_load(open(cfg_path)) if os.path.exists(cfg_path) else {}

DB   = cfg.get('db',   {})
SCAN = cfg.get('scan', {})
STEP_MHZ   = SCAN.get('step_mhz', 1)
NSAMPLES   = SCAN.get('samples', 256000)
BATCH_COMMIT = SCAN.get('batch_commit', 50)
raw_id = SCAN.get('session_id', 'default')
SESSION_ID = f"{raw_id}_{dt.datetime.now():%Y%m%d_%H%M%S}"

# ────────── 2. Connect to PostgreSQL ──────────
conn = psycopg2.connect(
    host=DB.get('host', '127.0.0.1'),
    dbname=DB.get('dbname', 'spectrum_atlas'),
    user=DB.get('user', 'atlas_user'),
    password=DB.get('password', 'changeme'))
cur  = conn.cursor()

# ────────── 3. Frequency band definition + sampling points ──────────
BANDS = [
    ("RC Low Band",       27,   28),    # Toy remote control low band
    ("RC Aircraft",       35,   36),    # UK RC aircraft band
    ("RC Ground",         40,   41),    # UK RC ground vehicles
    
    ("TETRA / Emergency", 380,  400),   # Police / Fire / Emergency comms
    
    ("LoRa / ISM 433",    433,  434),   # LoRa, RF remotes, door systems
    ("ISM 868",           868,  870),   # LoRaWAN, Sigfox, NB-IoT, metering

    ("GSM 900 UL",        880,  915),   # Uplink (Mobile → Base Station)
    ("GSM 900 DL",        925,  960),   # Downlink (Base Station → Mobile)

    ("FM Radio",          88,   108),   # FM radio broadcast
    ("Airband (AM)",      118,  137),   # Airband AM (pilot communication)
    ("AIS / Marine",      161,  163),   # AIS / Maritime communication

    ("ADS-B 1090",       1090, 1090),   # Aircraft broadcast position (ADS-B)

    ("LTE 1800",         1710, 1760),   # LTE uplink
]

FREQ_POINTS = []
for name, f_start, f_end in BANDS:
    if f_end - f_start < 5:
        freqs = [int((f_start + f_end) / 2)]
    else:
        step = (f_end - f_start) / 4  # 5 points
        freqs = [round(f_start + i * step) for i in range(5)]
    for f in freqs:
        FREQ_POINTS.append((f, name))  # (frequency, signal_type)

# ────────── 4. SLAM coordinate subscription ──────────
latest_pose = None
def pose_cb(msg: PoseStamped):
    global latest_pose
    latest_pose = (msg.pose.position.x, msg.pose.position.y)

rospy.init_node('sdr_logger', anonymous=True)
rospy.Subscriber('/slam_out_pose', PoseStamped, pose_cb)
rospy.loginfo("ROS subscriber ready. Starting SDR…")

# ────────── 5. Initialize RTL-SDR ──────────
sdr = RtlSdr()
sdr.sample_rate = 2.048e6
sdr.gain = 'auto'

# ────────── 6. Main sweeping loop ──────────
def power_db(samples):
    return 10*np.log10(np.mean(np.abs(samples)**2))

def print_progress(idx, total, freq):
    pct = idx / total * 100
    sys.stdout.write(f"\rScanning {freq:>6} MHz  "
                     f"[{idx}/{total}  {pct:5.1f}%]")
    sys.stdout.flush()

def graceful_exit(sig, frame):
    rospy.loginfo("Stopping…")
    sdr.close(); conn.close()
    print(f"\nTotal points written this run: {inserted}")
    sys.exit(0)
signal.signal(signal.SIGINT, graceful_exit)

inserted = 0
total_points = len(FREQ_POINTS)

while not rospy.is_shutdown():
    if latest_pose is None:
        rospy.sleep(0.1)
        continue

    x, y = latest_pose
    t_now = dt.datetime.utcnow()

    for idx, (f_mhz, sig_type) in enumerate(FREQ_POINTS, 1):
        try:
            sdr.center_freq = f_mhz * 1e6
            iq = sdr.read_samples(NSAMPLES)
            rssi = power_db(iq)
        except Exception as e:
            rospy.logwarn(f"IQ read fail @ {f_mhz} MHz: {e}")
            continue

        cur.execute(
            """
            INSERT INTO signal_samples
              (session_id, timestamp, signal_type, frequency,
               bandwidth, rssi, antenna_id, location)
            VALUES
              (%s,%s,%s,%s,%s,%s,%s,
               ST_SetSRID(ST_MakePoint(%s,%s),3857))
            """,
            (SESSION_ID, t_now, sig_type, f_mhz, STEP_MHZ,
             rssi, 'rtl-sdr-1', x, y))
        inserted += 1

        if inserted % BATCH_COMMIT == 0:
            conn.commit()

        if idx % 5 == 0 or idx == total_points:
            print_progress(idx, total_points, f_mhz)

    conn.commit()
    print("\nOne sweep done at", t_now.isoformat(),
          f"@ pose ({x:.2f},{y:.2f})")

    # ✚ Write status file for GUI reading
    with open("/home/Steven/Spectrum_Atlas_GUI2/logger_status.txt", "w") as fp:
        fp.write(f"{t_now:%Y-%m-%d %H:%M:%S} - {inserted} points completed")
