# 🌐 Spectrum Atlas - Wireless Signal Spectrum Mapping System

![Spectrum Atlas](https://img.shields.io/badge/Spectrum-Atlas-black?style=for-the-badge&logo=github) ![Python](https://img.shields.io/badge/Python-3.8+-blue.svg) ![C++](https://img.shields.io/badge/C++-17-blue.svg) ![ROS](https://img.shields.io/badge/ROS-Melodic-orange.svg) ![Qt](https://img.shields.io/badge/Qt-5.12+-green.svg)

## 📖 Project Overview

Spectrum Atlas is a comprehensive wireless signal spectrum mapping and visualization system that integrates hardware and software technologies to generate 2D heatmaps of wireless signals in spatial environments, providing professional solutions for spectrum analysis, signal monitoring, and visualization applications.

![Spectrum Atlas System](Images/Spectrum%20Atlas.jpg)

## ✨ Core Features

- 🎯 **Multi-band Signal Scanning**: Supports automatic scanning of multiple frequency bands from 27MHz to 1800MHz
- 📍 **Spatial Positioning**: Each sampling point contains precise 2D coordinate information
- 💾 **Data Persistence**: PostgreSQL database storage with multi-session management support
- 🗺️ **Heatmap Generation**: RBF interpolation + Gaussian filtering + visualization rendering
- 🔄 **Batch Processing**: Full frequency band batch output supporting multiple signal types
- 🌐 **Cross-device Collaboration**: Raspberry Pi generation → Local download viewing
- 🖥️ **Interactive Web Interface**: Real-time heatmap visualization with hover information display
- 📊 **Dynamic Data Loading**: Real-time database integration for live data access

## 🏗️ System Architecture

### Hardware Platform
- **Raspberry Pi 4B**: Main control device responsible for running ROS, signal scanning, data storage, and image rendering
- **RTL-SDR USB Receiver**: For acquiring signal strength (RSSI) at different frequency bands
- **LiDAR (LD19)**: For 2D plane mapping construction
- **Positioning Module**: Coordinates with LiDAR to obtain spatial location of each sampling point

![System Structure](Images/System%20Structure.png)

### Software Architecture
- **ROS (Robot Operating System)**: Robot operating system for hardware control and data management
- **Qt GUI**: C++ desktop application providing user interaction interface
- **Flask Web Server**: Python web framework providing backend API services
- **PostgreSQL**: Relational database for signal data storage
- **LiDAR SLAM**: Uses Hector SLAM for environmental map construction

## 🔧 Technical Features

### Supported Signal Types
- **RC Low Band** (27-28 MHz): Toy remote control low frequency band
- **RC Aircraft** (35-36 MHz): UK RC aircraft dedicated frequency band
- **RC Ground** (40-41 MHz): UK RC car/boat remote control frequency band
- **TETRA / Emergency** (380-400 MHz): Police/fire/emergency communications
- **LoRa / ISM 433** (433-434 MHz): LoRa, RF remotes, access control
- **ISM 868** (868-870 MHz): LoRaWAN, Sigfox, NB-IoT, metering
- **GSM 900 UL** (880-915 MHz): User uplink channels (mobile device → base station)
- **GSM 900 DL** (925-960 MHz): Downlink channels (base station → mobile device)
- **FM Radio** (88-108 MHz): FM broadcast
- **Airband (AM)** (118-137 MHz): Pilot communications (AM)
- **AIS / Marine** (161-163 MHz): Maritime automatic identification system
- **ADS-B 1090** (1090 MHz): Aircraft broadcast position
- **LTE 1800** (1710-1760 MHz): LTE user uplink

### Algorithm Flow

#### Algorithm 1: Map Construction and Data Acquisition
![Algorithm 1](Images/Algorithm%201%20flow%20chart%20and%20maps.png)

#### Algorithm 2: Heatmap Generation and Visualization
![Algorithm 2](Images/Algorithm%202%20flowchart%20and%20example.png)

## 📁 Project Structure

```
CASA0022_Spectrum_Atlas/
├── 3D Models/                    # 3D printed model files
├── Detector/                     # Detector related code
│   ├── ble_scan.py              # BLE signal scanning
│   ├── ldlidar_ros_ws/         # LiDAR ROS workspace
│   └── ros_catkin_ws/          # ROS Catkin workspace
├── Spectrum_Atlas_GUI/          # Qt GUI application
├── Server and Database/         # Server and database
│   ├── Database/               # Database scripts and API
│   └── Flask Server/           # Flask web server
├── Wireless Signal Dataset/     # Wireless signal dataset
└── Images/                     # Project images and charts
```

## 🚀 Quick Start

### Requirements
- **Operating System**: Ubuntu 18.04+ / Raspberry Pi OS
- **Python**: 3.8+
- **ROS**: Melodic (recommended)
- **Qt**: 5.12+
- **Database**: PostgreSQL 12+

### Installation Steps

1. **Clone the Project**
```bash
git clone https://github.com/SiruiLuo/CASA0022_Spectrum_Atlas.git
cd CASA0022_Spectrum_Atlas
```

2. **Install ROS Dependencies**
```bash
# Install ROS Melodic
sudo apt update
sudo apt install ros-melodic-desktop-full

# Initialize ROS workspace
cd Detector/ros_catkin_ws
catkin_make
source devel/setup.bash
```

3. **Install Python Dependencies**
```bash
cd Server and Database/Flask Server/Spectrum_Atlas_Webpage-main
python -m venv spectrum_env
source spectrum_env/bin/activate  # Linux/Mac
# or
spectrum_env\Scripts\activate     # Windows

pip install -r requirements.txt
```

4. **Configure Database**
```bash
# Install PostgreSQL
sudo apt install postgresql postgresql-contrib

# Create database and user
sudo -u postgres psql
CREATE DATABASE spectrum_atlas;
CREATE USER spectrum_user WITH PASSWORD 'your_password';
GRANT ALL PRIVILEGES ON DATABASE spectrum_atlas TO spectrum_user;
\q
```

5. **Compile Qt Application**
```bash
cd Spectrum_Atlas_GUI
qmake
make
```

## 📊 Usage Examples

### Signal Acquisition
The system can automatically scan multiple frequency bands and generate heatmaps:

#### Room01 Signal Heatmaps (2025-07-09)
All frequency band heatmaps generated from room01 data collection:

**Low Frequency Bands:**
![RC Low Band](Images/heatmap_RC_Low_Band_room01_20250709_151402.png) ![RC Aircraft](Images/heatmap_RC_Aircraft_room01_20250709_151402.png) ![RC Ground](Images/heatmap_RC_Ground_room01_20250709_151402.png)

**Medium Frequency Bands:**
![FM Radio](Images/heatmap_FM_Radio_room01_20250709_151402.png) ![Airband AM](Images/heatmap_Airband_(AM)_room01_20250709_151402.png) ![AIS Marine](Images/heatmap_AIS_-_Marine_room01_20250709_151402.png)

**High Frequency Bands:**
![GSM 900 UL](Images/heatmap_GSM_900_UL_room01_20250709_151402.png) ![GSM 900 DL](Images/heatmap_GSM_900_DL_room01_20250709_151402.png) ![LoRa ISM 433](Images/heatmap_LoRa_-_ISM_433_room01_20250709_151402.png)

**Ultra High Frequency Bands:**
![ISM 868](Images/heatmap_ISM_868_room01_20250709_151402.png) ![ADS-B 1090](Images/heatmap_ADS-B_1090_room01_20250709_151402.png) ![LTE 1800](Images/heatmap_LTE_1800_room01_20250709_151402.png)

**Emergency & Special Bands:**
![TETRA Emergency](Images/heatmap_TETRA_-_Emergency_room01_20250709_151402.png)

### GUI Interface
![GUI and Map](Images/GUI%20and%20Map.png)

### System Deployment
![Deployment in Exhibition](Images/Deployment%20in%20exhibition.png)

## 🔍 Data Acquisition Process

### Sampling Flowchart
![Sampling Flowchart](Images/Sampling%20Flowchart.png)

### Data Acquisition Commands and Data
![Data Acquisition](Images/Data%20acquisition%20command%20and%20data.png)

## 🗺️ Map Generation

### Hector SLAM
Uses LiDAR for environmental map construction:

![Hector SLAM](Images/Hector%20SLAM.png)

### Rviz Example
![Rviz Example](Images/Rviz%20Example.png)

## 💻 Web Application Features

### Interactive Heatmap Visualization
- **Real-time Data Loading**: Connects to PostgreSQL database for live signal data
- **Interactive Controls**: Dynamically select signal types and sessions
- **Hover Information**: Display signal strength and position coordinates on mouse hover
- **Responsive Design**: Optimized for desktop and mobile devices
- **Professional UI**: Modern, clean interface with consistent styling

### Flask Loading Interface
![Flask Loading](Images/Flask%20loading.png)

## 🔧 Hardware Components

### Main Components
- **RTL-SDR Receiver**: For signal reception and analysis
- **LiDAR Sensor**: For environmental map construction
- **Antenna System**: For signal reception
- **Display Module**: Waveshare display screen
- **Cooling System**: Fan and heat sink

### 3D Printed Enclosure
The project includes complete 3D printed enclosure design:

#### Main Box
![Box](Images/Box.png)

#### Lid Cover
![Lid](Images/Lid.png)

#### Handle Components
![Handles](Images/Handles.png)

## 📈 Performance Metrics

- **Scanning Frequency**: Supports full frequency band scanning from 27MHz to 1800MHz
- **Positioning Accuracy**: Centimeter-level positioning accuracy based on LiDAR
- **Data Update Rate**: Real-time data acquisition and visualization
- **Storage Capacity**: Supports TB-level data storage
- **Concurrent Users**: Web interface supports multiple users accessing simultaneously


## 📞 Contact

- **Project Maintainer**: [Sirui Luo]
- **Email**: [sirui.luo.24@ucl.ac.uk]

## 🙏 Acknowledgments

Thanks to the following open source projects and communities for their support:
- [ROS (Robot Operating System)](https://www.ros.org/)
- [RTL-SDR](https://www.rtl-sdr.com/)
- [Qt Framework](https://www.qt.io/)
- [Flask](https://flask.palletsprojects.com/)
- [PostgreSQL](https://www.postgresql.org/)

---

⭐ If this project is helpful to you, please give me a star! 
