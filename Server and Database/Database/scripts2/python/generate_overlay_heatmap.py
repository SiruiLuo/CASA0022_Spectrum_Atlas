#!/usr/bin/env python3

import sys, os, yaml, cv2, psycopg2, numpy as np
from shapely.geometry import Polygon, Point
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize

signal_type = sys.argv[1]
session_id = sys.argv[2]

pgm_path = '/home/pi/incoming_maps/map.pgm'
yaml_path = '/home/pi/incoming_maps/map.yaml'
meta = yaml.safe_load(open(yaml_path))
reso = float(meta['resolution'])
ox, oy, _ = meta['origin']
img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
h, w = img.shape

_, bw = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY_INV)
contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnt = max(contours, key=cv2.contourArea).reshape(-1, 2)
coords = []
for px, py in cnt:
    mx = ox + px * reso
    my = oy + (h - py) * reso
    coords.append((mx, my))
coords.append(coords[0])
room_poly = Polygon(coords)
xmin, ymin, xmax, ymax = room_poly.bounds

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
    sys.exit("No signal samples found.")

pts = np.array([[x, y] for x, y, _ in rows])
vals = np.array([r for _, _, r in rows])

res = 0.1
nx = int(np.ceil((xmax - xmin) / res)) + 1
ny = int(np.ceil((ymax - ymin) / res)) + 1
grid_x, grid_y = np.mgrid[xmin:xmax:complex(nx), ymin:ymax:complex(ny)]
zi = griddata(pts, vals, (grid_x, grid_y), method='nearest')
zi = gaussian_filter(zi, sigma=1)

mask = np.full(zi.shape, False)
for i in range(zi.shape[0]):
    for j in range(zi.shape[1]):
        px = xmin + i * res
        py = ymin + j * res
        if room_poly.contains(Point(px, py)):
            mask[i, j] = True
zi_masked = np.where(mask, zi, np.nan)

fig, ax = plt.subplots(figsize=(10, 8))

ax.imshow(
    img,
    cmap='gray',
    extent=(ox, ox + w * reso, oy, oy + h * reso),
    origin='lower',
    zorder=1
)

cmap = plt.get_cmap('jet')
norm = Normalize(vmin=np.nanmin(zi_masked), vmax=np.nanmax(zi_masked))
heatmap = ax.imshow(
    zi_masked.T,
    extent=(xmin, xmax, ymin, ymax),
    origin='lower',
    cmap=cmap,
    norm=norm,
    alpha=0.6,
    zorder=2
)

plt.colorbar(heatmap, ax=ax, label="RSSI")
ax.set_title(f"Signal Heatmap: {signal_type}")
ax.set_xlabel("X")
ax.set_ylabel("Y")

out_path = f"/home/pi/raster_out/overlay_rssi_{session_id}.png"
plt.savefig(out_path, dpi=300, bbox_inches='tight')
plt.close()

print("Overlay heatmap saved:", out_path)
