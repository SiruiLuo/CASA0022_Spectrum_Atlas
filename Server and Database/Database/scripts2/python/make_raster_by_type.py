#!/usr/bin/env python3
"""
Usage:
  python3 make_raster_by_type.py "TETRA / Emergency" "room01_20250709_151402"
"""
import sys, os, psycopg2, numpy as np
from shapely import wkt
from shapely.geometry import Point
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter
import rasterio
from rasterio.transform import from_origin


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
