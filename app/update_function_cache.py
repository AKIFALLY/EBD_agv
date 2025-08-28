#!/usr/bin/env python3
"""
強制更新 Flow Functions 快取
"""
import requests
import json
import yaml
from datetime import datetime

def update_cache():
    """強制更新函數快取"""
    print("\n=== 更新 Flow Functions 快取 ===\n")
    
    # 1. 調用 API 獲取最新函數（這會觸發快取更新）
    print("1. 從 Flow WCS 獲取最新函數...")
    try:
        response = requests.get("http://localhost:8001/linear-flow/api/functions?source=flow_wcs")
        if response.status_code == 200:
            data = response.json()
            if data.get("success"):
                print(f"   ✅ 成功獲取函數 (來源: {data.get('source')})")
                
                # 檢查是否有 control.switch
                functions = data.get("functions", {})
                control_funcs = functions.get("control", [])
                
                has_switch = False
                for func in control_funcs:
                    if func.get("name") == "control.switch":
                        has_switch = True
                        print(f"   ✅ control.switch 已找到")
                        break
                
                if not has_switch:
                    print(f"   ⚠️ control.switch 未找到")
                    print("   可用的 control 函數:")
                    for func in control_funcs:
                        print(f"      - {func.get('name')}")
            else:
                print(f"   ❌ API 返回失敗: {data.get('error')}")
        else:
            print(f"   ❌ HTTP 錯誤: {response.status_code}")
    except Exception as e:
        print(f"   ❌ 連接錯誤: {e}")
    
    # 2. 驗證快取文件
    print("\n2. 驗證快取文件...")
    try:
        with open("/app/config/wcs/flow_functions_cache.yaml", "r") as f:
            cache_data = yaml.safe_load(f)
            
        print(f"   ✅ 快取文件已載入")
        print(f"   更新時間: {cache_data.get('meta', {}).get('updated_at')}")
        
        # 檢查 control.switch
        control_funcs = cache_data.get("functions", {}).get("control", [])
        has_switch = False
        for func in control_funcs:
            if func.get("name") == "control.switch":
                has_switch = True
                print(f"   ✅ control.switch 在快取中")
                break
        
        if not has_switch:
            print(f"   ⚠️ control.switch 不在快取中")
            
    except Exception as e:
        print(f"   ❌ 讀取快取錯誤: {e}")
    
    print("\n=== 更新完成 ===")

if __name__ == "__main__":
    update_cache()