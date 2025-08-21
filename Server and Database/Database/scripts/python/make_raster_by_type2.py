#!/usr/bin/env python3

import sys, os, psycopg2, numpy as np
from shapely import wkt
from shapely.geometry import Polygon, MultiPolygon
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter
import rasterio
from rasterio.transform import from_origin
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon

signal_type = sys.argv[1]
session_id  = sys.argv[2]

conn = psycopg2.connect(
    dbname="spectrum_atlas",
    user="atlas_user",
    password="Paic2013",
    host="127.0.0.1"
)
cur = conn.cursor()

cur.execute("SELECT ST_AsText(poly) FROM room_shape LIMIT 1;")
poly_wkt = cur.fetchone()[0]
room_poly = wkt.loads(poly_wkt)


print("room_poly type:", type(room_poly))
print("is valid:", room_poly.is_valid)
print("is empty:", room_poly.is_empty)

xmin, ymin, xmax, ymax = room_poly.bounds

cur.execute("""
  SELECT ST_X(location), ST_Y(location), rssi
  FROM signal_samples
  WHERE signal_type = %s AND session_id = %s;
""", (signal_type, session_id))
rows = cur.fetchall()
cur.close()
conn.close()

if not rows:
    sys.exit("No samples found.")

pts  = np.array([[x, y] for x, y, _ in rows])
vals = np.array([r for _, _, r in rows])
res = 0.10
nx  = int(np.ceil((xmax - xmin) / res)) + 1
ny  = int(np.ceil((ymax - ymin) / res)) + 1
grid_x, grid_y = np.mgrid[xmin:xmax:complex(nx), ymin:ymax:complex(ny)]

zi = griddata(pts, vals, (grid_x, grid_y), method='nearest')
zi[np.isnan(zi)] = -9999
zi_sm = gaussian_filter(zi, sigma=1)

transform = from_origin(xmin, ymax, res, res)
out_path = f"/home/pi/raster_out/rssi_{session_id}.tif"
os.makedirs(os.path.dirname(out_path), exist_ok=True)

with rasterio.open(
    out_path, 'w',
    driver='GTiff',
    height=zi_sm.shape[0],
    width=zi_sm.shape[1],
    count=1,
    dtype='float32',
    crs='EPSG:3857',
    transform=transform,
    nodata=-9999
) as dst:
    dst.write(zi_sm.astype('float32'), 1)

print("GeoTIFF written:", out_path)

fig, ax = plt.subplots(figsize=(8, 6))
img = ax.imshow(
    zi_sm.T,
    origin='lower',
    extent=(xmin, xmax, ymin, ymax),
    cmap='jet',
    alpha=0.7,
    zorder=2
)

def draw_patch(geom):
    if geom.is_empty or not geom.is_valid:
        print("Skipping invalid geometry.")
        return
    try:
        coords = np.array(geom.exterior.coords)
        patch = MplPolygon(coords, closed=True, fill=False, edgecolor='black', linewidth=1.5, zorder=3)
        ax.add_patch(patch)
    except Exception as e:
        print("Failed to draw patch:", e)

cleaned_poly = room_poly.buffer(0)

if isinstance(cleaned_poly, Polygon):
    draw_patch(cleaned_poly)
elif isinstance(cleaned_poly, MultiPolygon):
    for poly in cleaned_poly.geoms:
        draw_patch(poly)

plt.colorbar(img, ax=ax, label="RSSI")
ax.set_title(f"Signal Heatmap: {signal_type}")
ax.set_xlabel("X")
ax.set_ylabel("Y")
png_out = out_path.replace('.tif', '.png')
plt.savefig(png_out, dpi=300, bbox_inches='tight')
plt.close()

print("Overlay PNG heatmap written:", png_out)
