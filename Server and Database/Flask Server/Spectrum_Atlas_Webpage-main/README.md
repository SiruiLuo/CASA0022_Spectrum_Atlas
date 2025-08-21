# Spectrum Atlas - Wireless Signal Visualization System

![Spectrum Atlas](https://img.shields.io/badge/Spectrum-Atlas-black?style=for-the-badge&logo=github)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![Flask](https://img.shields.io/badge/Flask-2.3+-lightgrey.svg)
![ROS](https://img.shields.io/badge/ROS-Noetic-orange.svg)

## 🛰️ Project Overview

Spectrum Atlas is a comprehensive wireless signal heatmap generation and visualization system that integrates hardware and software technologies to generate 2D heatmaps of wireless signals in spatial environments, providing professional solutions for spectrum analysis, signal monitoring, and visualization applications.

## 💡 Core Features

- ✅ **Spectrum Sampling**: Multi-band signal automatic scanning
- ✅ **Spatial Positioning**: Each point with 2D coordinates
- ✅ **Data Storage**: PostgreSQL persistent storage
- ✅ **Heatmap Generation**: RBF interpolation + Gaussian filtering + visualization
- ✅ **Batch Processing**: Full frequency band batch output
- ✅ **Cross-device Collaboration**: Raspberry Pi generation → Local download viewing
- ✅ **Interactive Web Interface**: Real-time heatmap visualization with hover information
- ✅ **Dynamic Data Loading**: Live database integration for real-time data access

## 🌐 Web Application Features

### Interactive Heatmap Visualization
- **Real-time Data Loading**: Connect to PostgreSQL database for live signal data
- **Interactive Controls**: Select signal types and sessions dynamically
- **Hover Information**: Display signal strength and position coordinates on mouse hover
- **Responsive Design**: Optimized for desktop and mobile devices
- **Professional UI**: Modern, clean interface with consistent styling

### Supported Signal Types
- **RC Low Band** (27-28 MHz): Toy remote control low frequency
- **RC Aircraft** (35-36 MHz): UK RC aircraft dedicated
- **RC Ground** (40-41 MHz): UK RC car/boat remote control
- **TETRA / Emergency** (380-400 MHz): Police/fire/emergency communications
- **LoRa / ISM 433** (433-434 MHz): LoRa, RF remotes, access control
- **ISM 868** (868-870 MHz): LoRaWAN, Sigfox, NB-IoT, metering
- **GSM 900 UL** (880-915 MHz): User uplink channels (mobile → base station)
- **GSM 900 DL** (925-960 MHz): Downlink channels (base station → mobile)
- **FM Radio** (88-108 MHz): FM broadcast
- **Airband (AM)** (118-137 MHz): Pilot communications (AM)
- **AIS / Marine** (161-163 MHz): Maritime automatic identification system
- **ADS-B 1090** (1090 MHz): Aircraft broadcast position
- **LTE 1800** (1710-1760 MHz): LTE user uplink

## 🏗️ Technical Architecture

### Hardware Platform
- **Raspberry Pi 4B**: Main control device, responsible for running ROS, signal scanning, data storage, and image rendering
- **RTL-SDR USB receiver**: For acquiring signal strength (RSSI) at different frequency bands
- **LiDAR (LD19)**: For 2D plane mapping
- **Positioning module**: Coordinates with LiDAR to obtain spatial location of each sampling point

### Web Application Stack
- **Flask**: Python web framework for backend API
- **PostgreSQL**: Database for signal data storage
- **HTML5 Canvas**: Client-side heatmap rendering
- **JavaScript**: Interactive frontend functionality
- **CSS3**: Modern responsive styling

### Data Acquisition & Storage
- Use RTL-SDR + Python to sample signal strength at different frequency bands
- Each signal point contains fields: `(x, y, rssi, signal_type, session_id)`
- Write to PostgreSQL database `signal_samples` table
- Support multiple sessions (different sessions) and various signal types (GSM, LoRa, TETRA, etc.)

### Map Generation
- Use Hector SLAM with LiDAR to generate `.pgm + .yaml` maps
- Maps used for background images and coordinate transformation in subsequent heatmap drawing

### Heatmap Generation
- Use modules such as `matplotlib`, `scipy`, `numpy`, `cv2`, `yaml`, `psycopg2`
- RBF interpolation to generate continuous signal distribution
- Gaussian filtering to smooth images
- Mask processing: Only generate images in white areas of the map (i.e., actual open areas)
- Output: Save as `heatmap_<signal_type>_<session_id>.png`

### Batch Processing System
- Automatically traverse a series of signal types (such as "LoRa / ISM 433", "GSM 900 DL")
- Batch call heatmap.py according to a specified session ID to generate heatmaps
- Process illegal characters in filenames (such as /, spaces) to avoid path errors

## 📦 Dependencies

```bash
# Create virtual environment
python -m venv spectrum_env
source spectrum_env/bin/activate  # Linux/Mac
# or
spectrum_env\Scripts\activate     # Windows

# Install dependencies
pip install -r requirements.txt
```

## 🚀 Quick Start

### 1. Hardware Preparation
- Raspberry Pi 4B
- RTL-SDR USB receiver
- LiDAR sensor (recommended LD19)
- Positioning module

### 2. Software Installation
```bash
# Clone project
git clone https://github.com/your-username/spectrum-atlas.git
cd spectrum-atlas

# Install dependencies
pip install -r requirements.txt
```

### 3. Database Configuration
```sql
-- Create database table
CREATE TABLE signal_samples (
    id SERIAL PRIMARY KEY,
    location GEOMETRY(POINT, 4326) NOT NULL,
    rssi FLOAT NOT NULL,
    signal_type VARCHAR(50) NOT NULL,
    session_id VARCHAR(50) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create spatial index
CREATE INDEX idx_signal_samples_location ON signal_samples USING GIST (location);
```

### 4. Run Web Application
```bash
# Start Flask web application
python run.py

# Or directly run Flask app
python app.py
```

The web application will be available at: http://localhost:5000

### 5. Run Signal Collection (Optional)
```bash
# Start signal collection
python signal_collector.py

# Generate static heatmap
python heatmap.py --signal_type "GSM 900 DL" --session_id "session_001"

# Batch generation
python batch_heatmap.py --session_id "session_001"
```

## 🌐 Web Application Usage

### Interactive Heatmap Features
1. **Select Signal Type**: Choose from the dropdown menu of available signal types
2. **Select Session**: Choose a data collection session from the database
3. **Generate Heatmap**: Click "Generate Heatmap" to create interactive visualization
4. **Hover Information**: Move mouse over the heatmap to see signal strength and coordinates
5. **Info Panel**: View real-time statistics including min/max RSSI and data points

### API Endpoints
- `GET /api/sessions` - Get available sessions from database
- `GET /api/heatmap/<signal_type>/<session_id>` - Get heatmap data for specific parameters

## 📊 System Demo

Visit our official website to view real-time demos: [Spectrum Atlas Official Website](https://your-domain.com)

### Demo Features
- Real-time signal strength monitoring
- Dynamic heatmap generation
- Multi-band signal analysis
- Interactive visualization interface
- Hover information display
- Responsive design for all devices

## 🔧 Configuration

### Database Connection
```python
# app.py
DB_CONFIG = {
    'dbname': 'spectrum_atlas',
    'user': 'atlas_user',
    'password': 'your_password',
    'host': 'your_host',
    'port': 5432
}
```

### Signal Type Configuration
```python
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
```

## 📁 Project Structure

```
spectrum-atlas/
├── app.py                    # Flask web application
├── run.py                    # Application launcher
├── requirements.txt          # Python dependencies
├── static/                   # Static web assets
│   ├── styles.css           # CSS styles
│   └── script.js            # JavaScript functionality
├── templates/                # HTML templates
│   └── index.html           # Main web page
├── hardware/                 # Hardware-related code
│   ├── rtl_sdr/             # RTL-SDR drivers
│   ├── lidar/               # LiDAR processing
│   └── positioning/         # Positioning module
├── software/                 # Software core
│   ├── signal_collector.py  # Signal collection
│   ├── heatmap.py           # Heatmap generation
│   ├── batch_heatmap.py     # Batch processing
│   └── database/            # Database operations
├── pics/                     # Static heatmap images
├── maps/                     # Map files (.pgm, .yaml)
├── docs/                     # Documentation
├── tests/                    # Tests
└── README.md                # Project documentation
```

## 🤝 Contributing

We welcome all forms of contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for details.

### How to Contribute
1. Fork the project
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Contact Us

- **Project Homepage**: [https://github.com/your-username/spectrum-atlas](https://github.com/your-username/spectrum-atlas)
- **Official Website**: [https://your-domain.com](https://your-domain.com)
- **Email**: contact@spectrum-atlas.com

## 🙏 Acknowledgments

Thanks to the following open-source projects and technologies:

- [RTL-SDR](https://www.rtl-sdr.com/) - Software-defined radio
- [Flask](https://flask.palletsprojects.com/) - Python web framework
- [PostgreSQL](https://www.postgresql.org/) - Database
- [ROS](https://www.ros.org/) - Robot Operating System
- [Hector SLAM](http://wiki.ros.org/hector_slam) - SLAM algorithm
- [matplotlib](https://matplotlib.org/) - Data visualization

---

**Spectrum Atlas** - Making wireless signal visualization simple and powerful 🛰️ 