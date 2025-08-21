# Spectrum Atlas Web Application - Startup Guide

## 🚀 Quick Start

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Database Setup
Make sure your PostgreSQL database is running and accessible with the following configuration:
- Database: `spectrum_atlas`
- User: `atlas_user`
- Password: `Paic2013`
- Host: `100.82.123.4`

### 3. Start the Application
```bash
python run.py
```

The application will be available at: **http://localhost:5000**

## 🌐 Features

### Interactive Heatmap Visualization
- **Real-time Data Loading**: Connect to PostgreSQL database for live signal data
- **Interactive Controls**: Select signal types and sessions dynamically
- **Hover Information**: Display signal strength and position coordinates on mouse hover
- **Responsive Design**: Optimized for desktop and mobile devices

### Supported Signal Types
- RC Low Band (27-28 MHz)
- RC Aircraft (35-36 MHz)
- RC Ground (40-41 MHz)
- TETRA / Emergency (380-400 MHz)
- LoRa / ISM 433 (433-434 MHz)
- ISM 868 (868-870 MHz)
- GSM 900 UL (880-915 MHz)
- GSM 900 DL (925-960 MHz)
- FM Radio (88-108 MHz)
- Airband (AM) (118-137 MHz)
- AIS / Marine (161-163 MHz)
- ADS-B 1090 (1090 MHz)
- LTE 1800 (1710-1760 MHz)

## 🔧 Configuration

### Database Connection
Edit `app.py` to modify database settings:
```python
DB_CONFIG = {
    'dbname': 'spectrum_atlas',
    'user': 'atlas_user',
    'password': 'Paic2013',
    'host': '100.82.123.4'
}
```

### Map Files
Ensure the following files exist in the `maps/` directory:
- `map.pgm` - Map image file
- `map.yaml` - Map configuration file

## 📊 Usage

1. **Select Signal Type**: Choose from the dropdown menu
2. **Select Session**: Choose a data collection session
3. **Generate Heatmap**: Click "Generate Heatmap" button
4. **Interactive Viewing**: Hover over the heatmap to see signal strength and coordinates
5. **Info Panel**: View real-time statistics

## 🧪 Testing

Run the test script to verify the application:
```bash
python test_app.py
```

## 📁 File Structure

```
Spectrum_Atlas_Web/
├── app.py                    # Flask application
├── run.py                    # Application launcher
├── test_app.py              # Test script
├── requirements.txt          # Dependencies
├── static/                   # Static assets
│   ├── styles.css           # CSS styles
│   └── script.js            # JavaScript
├── templates/                # HTML templates
│   └── index.html           # Main page
├── pics/                     # Static images
├── maps/                     # Map files
└── README.md                # Documentation
```

## 🐛 Troubleshooting

### Common Issues

1. **Database Connection Error**
   - Verify PostgreSQL is running
   - Check database credentials in `app.py`
   - Ensure network connectivity to database host

2. **Missing Dependencies**
   - Run `pip install -r requirements.txt`
   - Check Python version (3.8+ required)

3. **Map Files Not Found**
   - Ensure `maps/map.pgm` and `maps/map.yaml` exist
   - Check file permissions

4. **Port Already in Use**
   - Change port in `app.py` or `run.py`
   - Kill existing process using port 5000

## 📞 Support

For issues or questions, please refer to the main README.md file or create an issue in the project repository. 