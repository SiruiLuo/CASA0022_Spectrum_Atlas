#!/usr/bin/env python3
"""
Test script for Spectrum Atlas Flask application
"""

import requests
import json
from urllib.parse import quote

# 配置
BASE_URL = "http://100.89.2.88:5000"

def check_data_distribution():
    """检查数据库中每个会话和信号类型的数据分布"""
    print("=== 检查数据库数据分布 ===\n")
    
    try:
        # 获取所有会话
        sessions_response = requests.get(f"{BASE_URL}/api/sessions")
        if sessions_response.status_code != 200:
            print("无法获取会话列表")
            return
            
        sessions = sessions_response.json()
        print(f"找到 {len(sessions)} 个会话:")
        for session in sessions:
            print(f"  - {session['id']}")
        print()
        
        # 获取所有信号类型
        signal_types_response = requests.get(f"{BASE_URL}/api/signal_types")
        if signal_types_response.status_code != 200:
            print("无法获取信号类型列表")
            return
            
        signal_types = signal_types_response.json()
        print(f"找到 {len(signal_types)} 个信号类型:")
        for signal_type in signal_types:
            print(f"  - {signal_type}")
        print()
        
        # 重点检查TETRA、LoRa、AIS信号类型
        target_signals = ["TETRA / Emergency", "LoRa / ISM 433", "AIS / Marine"]
        print("重点检查目标信号类型的数据分布:")
        print("-" * 80)
        
        found_data = False
        for session in sessions:
            session_id = session['id']
            print(f"\n会话: {session_id}")
            print("-" * 40)
            
            for signal_type in target_signals:
                # 测试热力图生成
                encoded_signal_type = quote(signal_type)
                response = requests.get(f"{BASE_URL}/api/heatmap/{encoded_signal_type}/{session_id}")
                
                if response.status_code == 200:
                    data = response.json()
                    data_points = len(data.get('sample_points', {}).get('x', []))
                    print(f"  ✅ {signal_type}: {data_points} 个数据点")
                    found_data = True
                else:
                    print(f"  ❌ {signal_type}: 无数据")
        
        if not found_data:
            print("\n⚠️  警告: 在所有会话中都没有找到 TETRA / Emergency、LoRa / ISM 433、AIS / Marine 的数据!")
            print("这可能意味着:")
            print("1. 这些信号类型的数据还没有被采集")
            print("2. 数据在不同的会话ID中")
            print("3. 数据库中的signal_type名称与前端不匹配")
        
        print("\n" + "=" * 80)
        
    except Exception as e:
        print(f"检查数据分布时出错: {e}")

def test_api_endpoints():
    """测试所有API端点"""
    print("=== 测试 Spectrum Atlas API 端点 ===\n")
    
    # 测试1: 获取会话列表
    print("1. 测试获取会话列表...")
    try:
        response = requests.get(f"{BASE_URL}/api/sessions")
        print(f"   状态码: {response.status_code}")
        if response.status_code == 200:
            sessions = response.json()
            print(f"   找到 {len(sessions)} 个会话")
            for session in sessions[:5]:  # 只显示前5个
                print(f"   - {session['id']}")
        else:
            print(f"   错误: {response.text}")
    except Exception as e:
        print(f"   连接错误: {e}")
    print()
    
    # 测试2: 获取信号类型列表
    print("2. 测试获取信号类型列表...")
    try:
        response = requests.get(f"{BASE_URL}/api/signal_types")
        print(f"   状态码: {response.status_code}")
        if response.status_code == 200:
            signal_types = response.json()
            print(f"   找到 {len(signal_types)} 个信号类型")
            for signal_type in signal_types[:5]:  # 只显示前5个
                print(f"   - {signal_type}")
        else:
            print(f"   错误: {response.text}")
    except Exception as e:
        print(f"   连接错误: {e}")
    print()
    
    # 测试3: 测试热力图生成（使用第一个可用的会话和信号类型）
    print("3. 测试热力图生成...")
    try:
        # 先获取会话和信号类型
        sessions_response = requests.get(f"{BASE_URL}/api/sessions")
        signal_types_response = requests.get(f"{BASE_URL}/api/signal_types")
        
        if sessions_response.status_code == 200 and signal_types_response.status_code == 200:
            sessions = sessions_response.json()
            signal_types = signal_types_response.json()
            
            if sessions and signal_types:
                test_session = sessions[0]['id']
                test_signal_type = signal_types[0]
                
                print(f"   测试参数: session={test_session}, signal_type={test_signal_type}")
                
                # 测试热力图生成
                encoded_signal_type = quote(test_signal_type)
                response = requests.get(f"{BASE_URL}/api/heatmap/{encoded_signal_type}/{test_session}")
                print(f"   状态码: {response.status_code}")
                
                if response.status_code == 200:
                    data = response.json()
                    print(f"   成功生成热力图:")
                    print(f"   - 数据维度: {data.get('width', 'N/A')} x {data.get('height', 'N/A')}")
                    print(f"   - RSSI范围: {data.get('min_rssi', 'N/A')} 到 {data.get('max_rssi', 'N/A')}")
                    print(f"   - 数据点数量: {len(data.get('sample_points', {}).get('x', []))}")
                else:
                    print(f"   错误: {response.text}")
            else:
                print("   没有可用的会话或信号类型进行测试")
        else:
            print("   无法获取会话或信号类型列表")
    except Exception as e:
        print(f"   连接错误: {e}")
    print()
    
    # 测试4: 测试404错误处理
    print("4. 测试404错误处理...")
    try:
        response = requests.get(f"{BASE_URL}/api/heatmap/NonExistentSignal/NonExistentSession")
        print(f"   状态码: {response.status_code}")
        if response.status_code == 404:
            error_data = response.json()
            print(f"   错误消息: {error_data.get('error', 'Unknown error')}")
        else:
            print(f"   意外状态码: {response.status_code}")
    except Exception as e:
        print(f"   连接错误: {e}")
    print()
    
    print("=== 测试完成 ===")

if __name__ == "__main__":
    # 先检查数据分布
    check_data_distribution()
    
    # 然后运行常规测试
    test_api_endpoints() 