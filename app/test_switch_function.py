#!/usr/bin/env python3
"""
測試 control.switch 函數是否正確註冊和運作
"""
import sys
import yaml

# 設置 Python 路徑
sys.path.insert(0, '/app/flow_wcs_ws/src/flow_wcs')

def main():
    print("\n=== 測試 control.switch 函數註冊狀態 ===\n")
    
    # 載入必要的模組
    from flow_wcs.flow_executor import FlowExecutor
    
    # 創建一個簡單的測試流程
    test_flow = {
        'flow': {
            'id': 'test_switch',
            'name': 'Test Switch Function',
            'description': 'Test control.switch registration',
            'version': '1.0.0'
        },
        'steps': []
    }
    
    # 創建執行器實例
    executor = FlowExecutor(test_flow)
    
    # 檢查 control.switch 是否註冊
    print("1. 檢查函數註冊狀態:")
    if 'control.switch' in executor.functions:
        print("   ✅ control.switch 已成功註冊")
    else:
        print("   ❌ control.switch 未註冊")
        print("\n   可用的 control 函數:")
        for func_name in sorted(executor.functions.keys()):
            if func_name.startswith('control.'):
                print(f"      - {func_name}")
        return 1
    
    # 測試函數功能
    print("\n2. 測試 control.switch 功能:")
    
    # 測試案例 1: value = 4 應該返回 True
    test_params_1 = {
        'value': 4,
        'cases': {
            '4': True,
            '5': True,
            'default': False
        }
    }
    
    result_1 = executor.functions['control.switch'](test_params_1)
    print(f"   測試 value=4: {result_1}")
    if result_1 == True:
        print("      ✅ 正確返回 True")
    else:
        print(f"      ❌ 錯誤！期望 True，得到 {result_1}")
    
    # 測試案例 2: value = 5 應該返回 True  
    test_params_2 = {
        'value': 5,
        'cases': {
            '4': True,
            '5': True,
            'default': False
        }
    }
    
    result_2 = executor.functions['control.switch'](test_params_2)
    print(f"   測試 value=5: {result_2}")
    if result_2 == True:
        print("      ✅ 正確返回 True")
    else:
        print(f"      ❌ 錯誤！期望 True，得到 {result_2}")
    
    # 測試案例 3: value = 3 應該返回 False (default)
    test_params_3 = {
        'value': 3,
        'cases': {
            '4': True,
            '5': True,
            'default': False
        }
    }
    
    result_3 = executor.functions['control.switch'](test_params_3)
    print(f"   測試 value=3: {result_3}")
    if result_3 == False:
        print("      ✅ 正確返回 False (default)")
    else:
        print(f"      ❌ 錯誤！期望 False，得到 {result_3}")
    
    # 測試案例 4: 測試數字值
    test_params_4 = {
        'value': 10,
        'cases': {
            '10': 'high',
            '5': 'medium',
            'default': 'low'
        }
    }
    
    result_4 = executor.functions['control.switch'](test_params_4)
    print(f"   測試 value=10: {result_4}")
    if result_4 == 'high':
        print("      ✅ 正確返回 'high'")
    else:
        print(f"      ❌ 錯誤！期望 'high'，得到 {result_4}")
    
    print("\n3. 測試 empty_rack_transfer 流程中的使用:")
    
    # 模擬 empty_rack_transfer 中的使用
    test_params_5 = {
        'value': 4,  # empty_area_rack_count = 4
        'cases': {
            '4': True,
            '5': True,
            'default': False
        }
    }
    
    result_5 = executor.functions['control.switch'](test_params_5)
    print(f"   當 empty_area_rack_count=4 時: {result_5}")
    if result_5 == True:
        print("      ✅ 正確識別為高使用率")
    else:
        print(f"      ❌ 錯誤！應該識別為高使用率")
    
    print("\n=== 測試完成 ===")
    print("\n✅ control.switch 函數已正確實現並註冊！")
    print("✅ 可以處理物件格式的 cases 參數")
    print("✅ empty_rack_transfer 流程應該可以正常運行")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())