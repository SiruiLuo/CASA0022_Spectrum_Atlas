#!/usr/bin/env python3
from flask import Flask, jsonify, request, send_from_directory
import psycopg2, yaml, cv2, os, numpy as np
from scipy.interpolate import Rbf
from scipy.ndimage import gaussian_filter
from collections import defaultdict
from urllib.parse import unquote

###############################################################################

###############################################################################
DB_CONFIG = {
    "dbname":     "spectrum_atlas",
    "user":       "atlas_user",
    "password":   "Paic2013",
    "host":       "localhost",     
    "port":       5432
}
MAP_DIR   = "maps"                  
PORT      = 5002                   
###############################################################################

app = Flask(__name__)

def db():
    """Easy Connect"""
    return psycopg2.connect(**DB_CONFIG)

@app.route("/api/signal_types")
def signal_types():
    with db() as conn, conn.cursor() as cur:
        cur.execute("SELECT DISTINCT signal_type FROM signal_samples ORDER BY signal_type")
        return jsonify([r[0] for r in cur.fetchall()])

@app.route("/api/sessions")
def sessions():
    with db() as conn, conn.cursor() as cur:
        cur.execute("SELECT DISTINCT session_id FROM signal_samples ORDER BY session_id DESC")
        return jsonify([{"id": r[0]} for r in cur.fetchall()])

@app.route("/api/signal_samples")
def signal_samples():
    signal_type = request.args.get("type")
    session_id  = request.args.get("session_id")
    if not signal_type or not session_id:
        return jsonify({"error": "missing params"}), 400
    with db() as conn, conn.cursor() as cur:
        cur.execute("""
            SELECT ST_X(location), ST_Y(location), rssi
            FROM   signal_samples
            WHERE  signal_type = %s AND session_id = %s
        """, (signal_type, session_id))
        rows = cur.fetchall()
    return jsonify([{"x":x, "y":y, "rssi":r} for x,y,r in rows])


@app.route("/api/heatmap/<path:signal_type>/<session_id>")
def heatmap(signal_type, session_id):
    signal_type = unquote(signal_type).strip()
    session_id  = session_id.strip()


    pgm_path  = os.path.join(MAP_DIR, "map.pgm")
    yaml_path = os.path.join(MAP_DIR, "map.yaml")
    if not (os.path.exists(pgm_path) and os.path.exists(yaml_path)):
        return jsonify({"error": "map files not found"}), 404

    meta      = yaml.safe_load(open(yaml_path))
    reso      = float(meta["resolution"])
    ox, oy, _ = meta["origin"]

    img = cv2.flip(cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE), 0)
    h, w = img.shape
    x0, x1 = ox, ox + w * reso
    y0, y1 = oy, oy + h * reso


    with db() as conn, conn.cursor() as cur:
        cur.execute("""
            SELECT ST_X(location), ST_Y(location), rssi
            FROM   signal_samples
            WHERE  signal_type = %s AND session_id = %s
        """, (signal_type, session_id))
        rows = cur.fetchall()
    if not rows:
        return jsonify({"error": "no data"}), 404


    pts = defaultdict(list)
    for x,y,r in rows:
        pts[(x,y)].append(r)
    xs,ys,rs = zip(*[(x,y,np.mean(r)) for (x,y),r in pts.items()])
    xs,ys,rs = map(np.array, (xs,ys,rs))


    gx,gy = np.meshgrid(np.linspace(x0,x1,w), np.linspace(y0,y1,h))
    grid  = gaussian_filter(Rbf(xs,ys,rs, function="multiquadric")(gx,gy), 1)

    mask  = (img==254)|(img==255)
    grid  = np.where(mask, grid, np.nan)

    json_grid = [[None if np.isnan(v) else float(v) for v in row] for row in grid]
    return jsonify({
        "extent": [x0,x1,y0,y1],
        "width" : w,
        "height": h,
        "data"  : json_grid,
        "min_rssi": float(np.nanmin(grid)),
        "max_rssi": float(np.nanmax(grid)),
        "sample_points": {"x": xs.tolist(), "y": ys.tolist(), "rssi": rs.tolist()}
    })


@app.route("/maps/<path:filename>")
def maps(filename):
    return send_from_directory(MAP_DIR, filename)


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=PORT, debug=True)
