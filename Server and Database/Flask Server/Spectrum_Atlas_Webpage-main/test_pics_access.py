#!/usr/bin/env python3
"""
测试pics文件夹中图片文件的访问状态
"""

import os
import urllib.parse

def test_pics_files():
    """测试pics文件夹中的所有图片文件"""
    pics_dir = 'pics'
    
    if not os.path.exists(pics_dir):
        print(f"❌ pics文件夹不存在: {pics_dir}")
        return
    
    print(f"📁 检查pics文件夹: {pics_dir}")
    print(f"📂 文件夹权限: {oct(os.stat(pics_dir).st_mode)[-3:]}")
    
    # 获取所有PNG文件
    png_files = [f for f in os.listdir(pics_dir) if f.endswith('.png')]
    
    if not png_files:
        print("❌ 没有找到PNG文件")
        return
    
    print(f"📊 找到 {len(png_files)} 个PNG文件:")
    print("-" * 80)
    
    for filename in sorted(png_files):
        file_path = os.path.join(pics_dir, filename)
        
        # 检查文件是否存在
        if not os.path.exists(file_path):
            print(f"❌ 文件不存在: {filename}")
            continue
        
        # 获取文件信息
        stat_info = os.stat(file_path)
        file_size = stat_info.st_size
        file_permissions = oct(stat_info.st_mode)[-3:]
        
        # 检查文件权限
        is_readable = os.access(file_path, os.R_OK)
        
        # URL编码测试
        encoded_filename = urllib.parse.quote(filename)
        decoded_filename = urllib.parse.unquote(encoded_filename)
        
        status = "✅" if is_readable else "❌"
        print(f"{status} {filename}")
        print(f"   大小: {file_size:,} bytes")
        print(f"   权限: {file_permissions}")
        print(f"   可读: {is_readable}")
        print(f"   URL编码: {encoded_filename}")
        print(f"   解码后: {decoded_filename}")
        print()
    
    print("-" * 80)
    print("🔍 测试URL访问:")
    
    # 测试几个关键文件
    test_files = [
        'Project Structure.png',
        'heat3D.png',
        'heatmap_ADS-B_1090_room01_20250709_151402.png',
        'heatmap_FM_Radio_room01_20250709_151402.png'
    ]
    
    for filename in test_files:
        file_path = os.path.join(pics_dir, filename)
        if os.path.exists(file_path):
            encoded = urllib.parse.quote(filename)
            print(f"✅ {filename} -> {encoded}")
        else:
            print(f"❌ {filename} (文件不存在)")

if __name__ == '__main__':
    test_pics_files() 