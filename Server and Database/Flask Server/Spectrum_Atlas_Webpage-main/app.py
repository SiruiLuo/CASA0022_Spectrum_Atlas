from flask import Flask, render_template, send_from_directory, jsonify, request
import os
import sys
import yaml
import cv2
import psycopg2
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import Rbf
from scipy.ndimage import gaussian_filter
from collections import defaultdict
import json
import base64
from io import BytesIO
from urllib.parse import unquote

app = Flask(__name__)

# Configuration
app.config['SECRET_KEY'] = 'your-secret-key-here'
app.config['STATIC_FOLDER'] = 'static'

# Database configuration
DB_CONFIG = {
    'dbname': 'spectrum_atlas',
    'user': 'atlas_user',
    'password': 'Paic2013',
    'host': '127.0.0.1'
}

# Signal bands configuration
BANDS = [
    ("RC Low Band", 27, 28),
    ("RC Aircraft", 35, 36),
    ("RC Ground", 40, 41),
    ("TETRA / Emergency", 380, 400),
    ("LoRa / ISM 433", 433, 434),
    ("ISM 868", 868, 870),
    ("GSM 900 UL", 880, 915),
    ("GSM 900 DL", 925, 960),
    ("FM Radio", 88, 108),
    ("Airband (AM)", 118, 137),
    ("AIS / Marine", 161, 163),
    ("ADS-B 1090", 1090, 1090),
    ("LTE 1800", 1710, 1760),
]

def get_db_connection():
    """Create database connection"""
    try:
        return psycopg2.connect(**DB_CONFIG)
    except Exception as e:
        print(f"Database connection error: {str(e)}")
        return None

def generate_heatmap_data(signal_type, session_id):
    """Generate heatmap data for interactive visualization"""
    try:
        print(f"Generating heatmap for signal_type: {signal_type}, session_id: {session_id}")
        
        # Load map configuration
        pgm_path = 'maps/map.pgm'
        yaml_path = 'maps/map.yaml'
        
        if not os.path.exists(pgm_path) or not os.path.exists(yaml_path):
            print(f"Map files not found: {pgm_path} or {yaml_path}")
            return None
            
        meta = yaml.safe_load(open(yaml_path))
        reso = float(meta['resolution'])
        ox, oy, _ = meta['origin']

        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        img = cv2.flip(img, 0)

        h, w = img.shape
        x0, x1 = ox, ox + w * reso
        y0, y1 = oy, oy + h * reso

        # Get data from database
        conn = get_db_connection()
        if not conn:
            print("Failed to connect to database")
            return None
            
        cur = conn.cursor()
        
        # First check if data exists
        cur.execute("""
            SELECT COUNT(*) 
            FROM signal_samples 
            WHERE signal_type = %s AND session_id = %s;
        """, (signal_type, session_id))
        
        count = cur.fetchone()[0]
        print(f"Found {count} records for signal_type: {signal_type}, session_id: {session_id}")
        
        if count == 0:
            cur.close()
            conn.close()
            print(f"No data found for signal_type: {signal_type}, session_id: {session_id}")
            return None
        
        # Get the actual data
        cur.execute("""
            SELECT ST_X(location), ST_Y(location), rssi
            FROM signal_samples
            WHERE signal_type = %s AND session_id = %s;
        """, (signal_type, session_id))
        rows = cur.fetchall()
        cur.close()
        conn.close()

        if not rows:
            print("No rows returned from database")
            return None

        print(f"Processing {len(rows)} data points")

        # Process data
        point_dict = defaultdict(list)
        for x, y, rssi in rows:
            point_dict[(x, y)].append(rssi)

        xs, ys, rssi_vals = [], [], []
        for (x, y), rssis in point_dict.items():
            xs.append(x)
            ys.append(y)
            rssi_vals.append(np.mean(rssis))

        xs, ys, rssi_vals = np.array(xs), np.array(ys), np.array(rssi_vals)

        # Create grid
        grid_x, grid_y = np.meshgrid(
            np.linspace(x0, x1, w),
            np.linspace(y0, y1, h)
        )

        # RBF interpolation
        rbf_func = Rbf(xs, ys, rssi_vals, function='multiquadric')
        grid_z = rbf_func(grid_x, grid_y)
        grid_z = gaussian_filter(grid_z, sigma=1)

        # Apply mask
        white_mask = (img == 254) | (img == 255)
        grid_masked = np.where(white_mask, grid_z, np.nan)

        # Convert to JSON-serializable format
        # Replace NaN values with None for JSON serialization
        data_list = []
        for row in grid_masked:
            data_row = []
            for val in row:
                if np.isnan(val):
                    data_row.append(None)
                else:
                    data_row.append(float(val))
            data_list.append(data_row)
        
        heatmap_data = {
            'extent': [x0, x1, y0, y1],
            'width': w,
            'height': h,
            'data': data_list,
            'min_rssi': float(np.nanmin(grid_masked)),
            'max_rssi': float(np.nanmax(grid_masked)),
            'sample_points': {
                'x': xs.tolist(),
                'y': ys.tolist(),
                'rssi': rssi_vals.tolist()
            }
        }
        
        print(f"Heatmap generated successfully with {len(data_list)} rows")
        return heatmap_data
        
    except Exception as e:
        print(f"Error generating heatmap: {str(e)}")
        import traceback
        traceback.print_exc()
        return None

@app.route('/')
def index():
    """Main page route"""
    return render_template('index.html', bands=BANDS)

@app.route('/api/heatmap/<path:signal_type>/<session_id>')
def get_heatmap_data(signal_type, session_id):
    try:
        # Decode URL-encoded signal_type
        from urllib.parse import unquote
        decoded_signal_type = unquote(signal_type).strip()
        session_id = session_id.strip()
        print(f"API收到: signal_type={repr(decoded_signal_type)}, session_id={repr(session_id)}")
        
        heatmap_data = generate_heatmap_data(decoded_signal_type, session_id)
        if heatmap_data:
            return jsonify(heatmap_data)
        else:
            return jsonify({'error': f'No data found for signal_type: {decoded_signal_type}, session_id: {session_id}'}), 404
    except Exception as e:
        print(f"API error: {str(e)}")
        return jsonify({'error': str(e)}), 500

@app.route('/api/sessions')
def get_sessions():
    """Get available sessions from database"""
    try:
        conn = get_db_connection()
        if not conn:
            return jsonify({'error': 'Database connection failed'}), 500
            
        cur = conn.cursor()
        cur.execute("""
            SELECT DISTINCT session_id 
            FROM signal_samples 
            ORDER BY session_id DESC
        """)
        sessions = [{'id': row[0], 'created_at': 'Unknown'} for row in cur.fetchall()]
        cur.close()
        conn.close()
        return jsonify(sessions)
    except Exception as e:
        print(f"Error getting sessions: {str(e)}")
        return jsonify({'error': str(e)}), 500

@app.route('/api/signal_types')
def get_signal_types():
    """Get available signal types from database"""
    try:
        conn = get_db_connection()
        if not conn:
            return jsonify({'error': 'Database connection failed'}), 500
            
        cur = conn.cursor()
        cur.execute("""
            SELECT DISTINCT signal_type 
            FROM signal_samples 
            ORDER BY signal_type
        """)
        signal_types = [row[0] for row in cur.fetchall()]
        cur.close()
        conn.close()
        return jsonify(signal_types)
    except Exception as e:
        print(f"Error getting signal types: {str(e)}")
        return jsonify({'error': str(e)}), 500

@app.route('/static/<path:filename>')
def static_files(filename):
    """Serve static files"""
    return send_from_directory('static', filename)

@app.route('/pics/<path:filename>')
def pics(filename):
    """Serve image files from pics directory"""
    try:
        # 解码URL编码的文件名
        from urllib.parse import unquote
        decoded_filename = unquote(filename)
        
        # 检查文件是否存在
        file_path = os.path.join('pics', decoded_filename)
        if not os.path.exists(file_path):
            print(f"图片文件不存在: {file_path}")
            return jsonify({'error': f'File not found: {decoded_filename}'}), 404
        
        # 检查文件权限
        if not os.access(file_path, os.R_OK):
            print(f"图片文件无读取权限: {file_path}")
            return jsonify({'error': f'Permission denied: {decoded_filename}'}), 403
        
        print(f"成功提供图片文件: {decoded_filename}")
        return send_from_directory('pics', decoded_filename)
        
    except Exception as e:
        print(f"提供图片文件时出错: {str(e)}")
        return jsonify({'error': str(e)}), 500

@app.route('/maps/<path:filename>')
def maps(filename):
    """Serve map files from maps directory"""
    if filename.endswith('.pgm'):
        # Serve PGM file directly for JavaScript parsing
        pgm_path = os.path.join('maps', filename)
        if os.path.exists(pgm_path):
            return send_from_directory('maps', filename, mimetype='application/octet-stream')
    
    return send_from_directory('maps', filename)

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000) 