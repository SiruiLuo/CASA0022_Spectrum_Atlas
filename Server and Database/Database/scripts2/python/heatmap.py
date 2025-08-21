#!/usr/bin/env python3
import sys, yaml, cv2, psycopg2
import numpy as np, matplotlib.pyplot as plt
from scipy.interpolate import Rbf
from scipy.ndimage import gaussian_filter
from scipy.spatial import cKDTree
from collections import defaultdict

EPS_SCALE = 1.0
SMOOTH_SCALE = 0.2
SIGMA_SCALE = 0.4
THEO_MIN = -120

signal_type, session_id = sys.argv[1:3]
safe_signal_type = signal_type.replace(" ", "_").replace("/", "-")

pgm_path = '/home/pi/incoming_maps_2/map.pgm'
yaml_path = '/home/pi/incoming_maps_2/map.yaml'
meta = yaml.safe_load(open(yaml_path))
reso, (ox, oy, _) = float(meta['resolution']), meta['origin']

img = cv2.flip(cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE), 0)
h, w = img.shape
x0, x1, y0, y1 = ox, ox + w * reso, oy, oy + h * reso
extent = (x0, x1, y0, y1)

conn = psycopg2.connect(dbname="spectrum_atlas", user="atlas_user", password="Paic2013", host="127.0.0.1")
cur = conn.cursor()
cur.execute("SELECT ST_X(location), ST_Y(location), rssi FROM signal_samples WHERE signal_type=%s AND session_id=%s;",
            (signal_type, session_id))
rows = cur.fetchall()
cur.close(); conn.close()
if not rows:
    sys.exit("No signal points found.")

d = defaultdict(list)
for x, y, rssi in rows:
    d[(x, y)].append(rssi)
xs, ys, rssi_vals = zip(*[(x, y, np.mean(r)) for (x, y), r in d.items()])
xs, ys, rssi_vals = map(np.asarray, (xs, ys, rssi_vals))

tree = cKDTree(np.c_[xs, ys])
median_nn = np.median(tree.query(np.c_[xs, ys], k=2)[0][:, 1])
eps = EPS_SCALE * median_nn
smooth = SMOOTH_SCALE * median_nn

rbf = Rbf(xs, ys, rssi_vals, function='multiquadric', epsilon=eps, smooth=smooth)
gx, gy = np.meshgrid(np.linspace(x0, x1, w), np.linspace(y0, y1, h))
gz = rbf(gx, gy)
sigma_px = max(0.1, SIGMA_SCALE * median_nn / reso)
gz = gaussian_filter(gz, sigma=sigma_px)

lo = THEO_MIN
hi = rssi_vals.max()
gz = np.clip(gz, lo, hi)

mask = (img == 254) | (img == 255)
gm = np.where(mask, gz, np.nan)

fig, ax = plt.subplots(figsize=(10, 8))
ax.imshow(img, cmap='gray', extent=extent, origin='lower', zorder=1)
im = ax.imshow(gm, cmap='jet', extent=extent, origin='lower', alpha=0.75, vmin=lo, vmax=hi, zorder=2)
plt.colorbar(im, ax=ax, label='RSSI (dBm)')
ax.set_title(f"Signal Heatmap\n{signal_type}")
ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
out = f"/home/pi/raster_out_2/heatmap_{safe_signal_type}_{session_id}.png"
plt.savefig(out, dpi=300, bbox_inches='tight'); plt.close()
print("Heatmap saved:", out)
