# Spectrum Atlas - Wireless Signal Heatmap Visualization System

![Spectrum Atlas](https://img.shields.io/badge/Spectrum-Atlas-black?style=for-the-badge&logo=github)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
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

## 🏗️ Technical Architecture

### Hardware Platform
- **Raspberry Pi 4B**: Main control device, responsible for running ROS, signal scanning, data storage, and image rendering
- **RTL-SDR USB receiver**: For acquiring signal strength (RSSI) at different frequency bands
- **LiDAR (LD19)**: For 2D plane mapping
- **Positioning module**: Coordinates with LiDAR to obtain spatial location of each sampling point

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
pip install opencv-python PyYAML psycopg2-binary numpy matplotlib scipy
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
    x FLOAT NOT NULL,
    y FLOAT NOT NULL,
    rssi FLOAT NOT NULL,
    signal_type VARCHAR(50) NOT NULL,
    session_id VARCHAR(50) NOT NULL,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

### 4. Run System
```bash
# Start signal collection
python signal_collector.py

# Generate heatmap
python heatmap.py --signal_type "GSM 900 DL" --session_id "session_001"

# Batch generation
python batch_heatmap.py --session_id "session_001"
```

## 📊 System Demo

Visit our official website to view real-time demos: [Spectrum Atlas Official Website](https://your-domain.com)

### Demo Features
- Real-time signal strength monitoring
- Dynamic heatmap generation
- Multi-band signal analysis
- Interactive visualization interface

## 🔧 Configuration

### Database Connection
```python
# config.py
DATABASE_CONFIG = {
    'host': 'localhost',
    'port': 5432,
    'database': 'spectrum_atlas',
    'user': 'your_username',
    'password': 'your_password'
}
```

### Signal Type Configuration
```python
SIGNAL_TYPES = [
    "GSM 900 DL",
    "GSM 900 UL", 
    "LoRa / ISM 433",
    "TETRA",
    "WiFi 2.4GHz",
    "WiFi 5GHz"
]
```

## 📁 Project Structure

```
spectrum-atlas/
├── hardware/              # Hardware-related code
│   ├── rtl_sdr/          # RTL-SDR drivers
│   ├── lidar/            # LiDAR processing
│   └── positioning/      # Positioning module
├── software/             # Software core
│   ├── signal_collector.py    # Signal collection
│   ├── heatmap.py            # Heatmap generation
│   ├── batch_heatmap.py      # Batch processing
│   └── database/             # Database operations
├── web/                  # Official website
│   ├── index.html
│   ├── styles.css
│   └── script.js
├── pics/                 # Heatmap images
├── docs/                 # Documentation
├── tests/                # Tests
└── requirements.txt      # Dependency list
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
- [ROS](https://www.ros.org/) - Robot Operating System
- [Hector SLAM](http://wiki.ros.org/hector_slam) - SLAM algorithm
- [PostgreSQL](https://www.postgresql.org/) - Database
- [matplotlib](https://matplotlib.org/) - Data visualization

---

**Spectrum Atlas** - Making wireless signal visualization simple and powerful 🛰️ 