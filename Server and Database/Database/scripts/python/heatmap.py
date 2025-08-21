#!/usr/bin/env python3

import sys, yaml, cv2, psycopg2
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import Rbf
from scipy.ndimage import gaussian_filter
from collections import defaultdict

signal_type = sys.argv[1]
session_id  = sys.argv[2]


safe_signal_type = signal_type.replace(" ", "_").replace("/", "-")

pgm_path = '/home/pi/incoming_maps/map.pgm'
yaml_path = '/home/pi/incoming_maps/map.yaml'
meta = yaml.safe_load(open(yaml_path))
reso = float(meta['resolution'])
ox, oy, _ = meta['origin']

img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
img = cv2.flip(img, 0)

h, w = img.shape
x0, x1 = ox, ox + w * reso
y0, y1 = oy, oy + h * reso
extent = (x0, x1, y0, y1)

conn = psycopg2.connect(
    dbname="spectrum_atlas",
    user="atlas_user",
    password="Paic2013",
    host="127.0.0.1"
)
cur = conn.cursor()
cur.execute("""
    SELECT ST_X(location), ST_Y(location), rssi
    FROM signal_samples
    WHERE signal_type = %s AND session_id = %s;
""", (signal_type, session_id))
rows = cur.fetchall()
cur.close()
conn.close()

if not rows:
    print("No signal points found.")
    sys.exit(1)

point_dict = defaultdict(list)
for x, y, rssi in rows:
    point_dict[(x, y)].append(rssi)

xs, ys, rssi_vals = [], [], []
for (x, y), rssis in point_dict.items():
    xs.append(x)
    ys.append(y)
    rssi_vals.append(np.mean(rssis))

xs, ys, rssi_vals = np.array(xs), np.array(ys), np.array(rssi_vals)

grid_x, grid_y = np.meshgrid(
    np.linspace(x0, x1, w),
    np.linspace(y0, y1, h)
)

try:
    rbf_func = Rbf(xs, ys, rssi_vals, function='multiquadric')
    grid_z = rbf_func(grid_x, grid_y)
    grid_z = gaussian_filter(grid_z, sigma=1)
except Exception as e:
    print("RBF interpolation failed:", str(e))
    sys.exit(1)

white_mask = (img == 254) | (img == 255)
grid_masked = np.where(white_mask, grid_z, np.nan)

fig, ax = plt.subplots(figsize=(10, 8))
ax.imshow(img, cmap='gray', extent=extent, origin='lower', zorder=1)

heat = ax.imshow(
    grid_masked, cmap='jet', extent=extent, origin='lower',
    alpha=0.75, zorder=2
)

plt.colorbar(heat, ax=ax, label='RSSI')
ax.set_title(f"Signal Heatmap\n{signal_type}")
ax.set_xlabel("X (meters)")
ax.set_ylabel("Y (meters)")

out_path = f"/home/pi/raster_out/heatmap_{safe_signal_type}_{session_id}.png"
plt.savefig(out_path, dpi=300, bbox_inches='tight')
plt.close()

print("Heatmap saved:", out_path)