#!/usr/bin/env python3
"""
Spectrum Atlas Web Application
Interactive wireless signal heatmap visualization system
"""

from app import app

if __name__ == '__main__':
    print("Starting Spectrum Atlas Web Application...")
    print("Access the application at: http://localhost:5000")
    print("Press Ctrl+C to stop the server")
    
    try:
        app.run(debug=True, host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        print("\nServer stopped by user")
    except Exception as e:
        print(f"Error starting server: {e}") 