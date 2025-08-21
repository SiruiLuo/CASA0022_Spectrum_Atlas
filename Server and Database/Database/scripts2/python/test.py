#!/usr/bin/env python3

import sys, yaml, cv2, psycopg2
import numpy as np
import matplotlib.pyplot as plt


signal_type = sys.argv[1]
session_id  = sys.argv[2]


pgm_path = '/home/pi/incoming_maps/map.pgm'
yaml_path = '/home/pi/incoming_maps/map.yaml'

meta = yaml.safe_load(open(yaml_path))
reso = float(meta['resolution'])          # meters/pixel
ox, oy, _ = meta['origin']                # origin of map in world coords

img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
img = cv2.flip(img, 0)  

h, w = img.shape
x0 = ox
x1 = ox + w * reso
y0 = oy
y1 = oy + h * reso
extent = (x0, x1, y0, y1)


conn = psycopg2.connect(
    dbname="spectrum_atlas",
    user="atlas_user",
    password="Paic2013",
    host="127.0.0.1"
)
cur = conn.cursor()
cur.execute("""
    SELECT ST_X(location), ST_Y(location)
    FROM signal_samples
    WHERE signal_type = %s AND session_id = %s;
""", (signal_type, session_id))
points = cur.fetchall()
cur.close()
conn.close()

if not points:
    print("No signal points found.")
    sys.exit(1)

xs, ys = zip(*points)


fig1, ax1 = plt.subplots(figsize=(10, 8))
ax1.imshow(img, cmap='gray', extent=extent, origin='lower')
ax1.set_title("Map Floorplan (from .pgm)")
ax1.set_xlabel("X (meters)")
ax1.set_ylabel("Y (meters)")

map_only_path = f"/home/pi/raster_out/map_only_{session_id}.png"
plt.savefig(map_only_path, dpi=300, bbox_inches='tight')
plt.close()
print("Map-only image saved:", map_only_path)


fig2, ax2 = plt.subplots(figsize=(10, 8))
ax2.imshow(img, cmap='gray', extent=extent, origin='lower', zorder=1)
ax2.scatter(xs, ys, c='red', s=15, label='Measurement Point', zorder=2)
ax2.set_title(f"Signal Measurement Points\n{signal_type}")
ax2.set_xlabel("X (meters)")
ax2.set_ylabel("Y (meters)")
ax2.legend()

points_path = f"/home/pi/raster_out/points_{session_id}.png"
plt.savefig(points_path, dpi=300, bbox_inches='tight')
plt.close()
print("Measurement point plot saved:", points_path)
