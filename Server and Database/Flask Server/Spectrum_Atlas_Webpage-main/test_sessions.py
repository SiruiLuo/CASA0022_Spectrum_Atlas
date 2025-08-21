#!/usr/bin/env python3
"""
Test script to verify session loading functionality
"""

import requests
import json

def test_session_loading():
    """Test session loading from database"""
    base_url = "http://localhost:5000"
    
    print("Testing Session Loading Functionality...")
    print("=" * 50)
    
    try:
        # Test sessions API
        print("1. Testing sessions API...")
        response = requests.get(f"{base_url}/api/sessions")
        
        if response.status_code == 200:
            sessions = response.json()
            print(f"✅ Sessions API working, found {len(sessions)} sessions:")
            
            for i, session in enumerate(sessions[:5], 1):  # Show first 5 sessions
                print(f"   {i}. Session ID: {session['id']}")
                if session['created_at'] != 'Unknown':
                    print(f"      Created: {session['created_at']}")
                else:
                    print(f"      Created: No date info")
            
            if len(sessions) > 5:
                print(f"   ... and {len(sessions) - 5} more sessions")
                
            # Test with a specific session
            if sessions:
                test_session = sessions[0]['id']
                test_signal = "GSM 900 DL"
                
                print(f"\n2. Testing heatmap generation with session '{test_session}'...")
                response = requests.get(f"{base_url}/api/heatmap/{test_signal}/{test_session}")
                
                if response.status_code == 200:
                    data = response.json()
                    print(f"✅ Heatmap data generated successfully")
                    print(f"   - Data points: {len(data['sample_points']['x'])}")
                    print(f"   - RSSI range: {data['min_rssi']:.1f} to {data['max_rssi']:.1f} dBm")
                    print(f"   - Grid size: {data['width']} x {data['height']}")
                else:
                    print(f"❌ Heatmap generation failed: {response.status_code}")
                    print(f"   Response: {response.text}")
            else:
                print("⚠️  No sessions found in database")
                
        else:
            print(f"❌ Sessions API failed: {response.status_code}")
            print(f"   Response: {response.text}")
            
    except requests.exceptions.ConnectionError:
        print("❌ Flask application is not running!")
        print("Please start the application with: python run.py")
    except Exception as e:
        print(f"❌ Test failed with error: {e}")

if __name__ == "__main__":
    test_session_loading() 