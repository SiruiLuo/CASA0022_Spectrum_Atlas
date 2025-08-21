#!/usr/bin/env python3
import cv2, yaml, psycopg2, numpy as np, os, glob
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union

MAP_DIR = os.path.expanduser('~/incoming_maps')
pgm_files = sorted(glob.glob(f'{MAP_DIR}/map*.pgm'))
if not pgm_files:
    raise FileNotFoundError("No map.pgm file found in ~/incoming_maps")

pgm = pgm_files[-1]
meta_file = pgm.replace('.pgm', '.yaml')
if not os.path.exists(meta_file):
    raise FileNotFoundError(f"Missing corresponding YAML file: {meta_file}")
meta = yaml.safe_load(open(meta_file))

reso = float(meta['resolution'])
ox, oy, _ = meta['origin']

img = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
h, w = img.shape
_, bw = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY_INV)
contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

polygons = []
for cnt in contours:
    if len(cnt) < 3:
        continue
    cnt = cnt.reshape(-1, 2)
    coords = []
    for px, py in cnt:
        mx = ox + px * reso
        my = oy + (h - py) * reso
        coords.append((mx, my))
    if coords[0] != coords[-1]:
        coords.append(coords[0])
    poly = Polygon(coords)
    if poly.is_valid and poly.area > 0:
        polygons.append(poly)

if not polygons:
    raise ValueError("No valid polygons found.")

merged = unary_union(polygons)
wkt = merged.wkt

try:
    conn = psycopg2.connect(
        dbname="spectrum_atlas",
        user="atlas_user",
        password="Paic2013",
        host="127.0.0.1"
    )
    cur = conn.cursor()
    cur.execute("""
    CREATE TABLE IF NOT EXISTS room_shape(id serial, poly geometry(Polygon,0));
    TRUNCATE room_shape;
    INSERT INTO room_shape(poly) VALUES (ST_GeomFromText(%s,0));
    """, (wkt,))
    conn.commit()
    cur.close()
    conn.close()
    print("Room polygon updated.")
except Exception as e:
    print("Database error:", e)
