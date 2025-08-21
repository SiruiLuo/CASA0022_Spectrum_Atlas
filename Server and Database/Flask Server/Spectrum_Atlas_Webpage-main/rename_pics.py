#!/usr/bin/env python3
"""
重命名pics文件夹中的图片文件为简化的名称
"""

import os
import shutil

def rename_pics_files():
    """重命名pics文件夹中的图片文件"""
    pics_dir = 'pics'
    
    # 定义文件名映射
    file_mapping = {
        'heatmap_ADS-B_1090_room01_20250709_151402.png': 'adsb_1090.png',
        'heatmap_AIS_-_Marine_room01_20250709_151402.png': 'ais_marine.png',
        'heatmap_Airband_(AM)_room01_20250709_151402.png': 'airband_am.png',
        'heatmap_FM_Radio_room01_20250709_151402.png': 'fm_radio.png',
        'heatmap_GSM_900_DL_room01_20250709_151402.png': 'gsm_900_dl.png',
        'heatmap_GSM_900_UL_room01_20250709_151402.png': 'gsm_900_ul.png',
        'heatmap_ISM_868_room01_20250709_151402.png': 'ism_868.png',
        'heatmap_LoRa_-_ISM_433_room01_20250709_151402.png': 'lora_433.png',
        'heatmap_LTE_1800_room01_20250709_151402.png': 'lte_1800.png',
        'heatmap_RC_Aircraft_room01_20250709_151402.png': 'rc_aircraft.png',
        'heatmap_RC_Ground_room01_20250709_151402.png': 'rc_ground.png',
        'heatmap_RC_Low_Band_room01_20250709_151402.png': 'rc_low_band.png',
        'heatmap_TETRA_-_Emergency_room01_20250709_151402.png': 'tetra_emergency.png'
    }
    
    print("🔄 开始重命名图片文件...")
    
    for old_name, new_name in file_mapping.items():
        old_path = os.path.join(pics_dir, old_name)
        new_path = os.path.join(pics_dir, new_name)
        
        if os.path.exists(old_path):
            try:
                # 复制文件而不是移动，保留原文件
                shutil.copy2(old_path, new_path)
                print(f"✅ 已复制: {old_name} -> {new_name}")
            except Exception as e:
                print(f"❌ 复制失败 {old_name}: {str(e)}")
        else:
            print(f"⚠️  文件不存在: {old_name}")
    
    print("\n📋 重命名完成！")
    print("📁 现在pics文件夹包含以下文件:")
    
    # 列出所有PNG文件
    png_files = [f for f in os.listdir(pics_dir) if f.endswith('.png')]
    for filename in sorted(png_files):
        file_path = os.path.join(pics_dir, filename)
        file_size = os.path.getsize(file_path)
        print(f"  📄 {filename} ({file_size:,} bytes)")

if __name__ == '__main__':
    rename_pics_files() 