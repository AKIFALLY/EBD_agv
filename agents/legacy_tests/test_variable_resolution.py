#!/usr/bin/env python3
"""
測試檔案: test_variable_resolution.py
用途: 測試 FlowExecutor 的變數解析修復是否正確處理陣列類型
創建日期: 2025-01-13
AI Agent: Claude
狀態: 臨時測試檔案，功能驗證後可刪除
"""

import asyncio
import json
import sys
import os

# 添加模組路徑
# 容器內路徑
if os.path.exists('/app/flow_wcs_ws/src/flow_wcs'):
    sys.path.insert(0, '/app/flow_wcs_ws/src/flow_wcs')
# 宿主機路徑  
else:
    sys.path.insert(0, '/home/ct/RosAGV/app/flow_wcs_ws/src/flow_wcs')

from flow_wcs.flow_executor import FlowExecutor

async def test_variable_resolution():
    """測試變數解析功能"""
    
    # 創建最小的 flow_data 用於測試
    flow_data = {
        "flow": {
            "id": "test_flow",
            "name": "Test Flow"
        },
        "workflow": []
    }
    
    # 創建 FlowExecutor 實例
    executor = FlowExecutor(flow_data)
    
    # 設置測試變數
    executor.context['variables'] = {
        'test_rooms': [1, 2, 3, 4, 5],
        'test_string': 'hello world',
        'test_number': 42,
        'test_dict': {'key': 'value'},
        'room_prefix': 'Room'
    }
    
    print("=" * 60)
    print("測試 FlowExecutor 變數解析功能")
    print("=" * 60)
    
    # 測試案例
    test_cases = [
        {
            'name': '純變數引用（陣列）',
            'input': '${test_rooms}',
            'expected_type': list,
            'expected_value': [1, 2, 3, 4, 5]
        },
        {
            'name': '純變數引用（字串）',
            'input': '${test_string}',
            'expected_type': str,
            'expected_value': 'hello world'
        },
        {
            'name': '純變數引用（數字）',
            'input': '${test_number}',
            'expected_type': int,
            'expected_value': 42
        },
        {
            'name': '純變數引用（字典）',
            'input': '${test_dict}',
            'expected_type': dict,
            'expected_value': {'key': 'value'}
        },
        {
            'name': '字串內嵌變數',
            'input': '${room_prefix} ${test_number}',
            'expected_type': str,
            'expected_value': 'Room 42'
        },
        {
            'name': '字典中的變數',
            'input': {
                'rooms': '${test_rooms}',
                'count': '${test_number}',
                'label': 'Total: ${test_number}'
            },
            'expected_type': dict,
            'expected_value': {
                'rooms': [1, 2, 3, 4, 5],
                'count': 42,
                'label': 'Total: 42'
            }
        },
        {
            'name': '列表中的變數',
            'input': ['${test_rooms}', '${test_string}', 'static text'],
            'expected_type': list,
            'expected_value': [[1, 2, 3, 4, 5], 'hello world', 'static text']
        }
    ]
    
    # 執行測試
    all_passed = True
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n測試 {i}: {test_case['name']}")
        print("-" * 40)
        
        try:
            # 解析變數
            result = await executor.resolve_variables(test_case['input'])
            
            # 檢查類型
            if type(result) != test_case['expected_type']:
                print(f"❌ 類型錯誤:")
                print(f"   預期: {test_case['expected_type'].__name__}")
                print(f"   實際: {type(result).__name__}")
                all_passed = False
            else:
                print(f"✅ 類型正確: {test_case['expected_type'].__name__}")
            
            # 檢查值
            if result != test_case['expected_value']:
                print(f"❌ 值錯誤:")
                print(f"   預期: {test_case['expected_value']}")
                print(f"   實際: {result}")
                all_passed = False
            else:
                print(f"✅ 值正確: {result}")
                
        except Exception as e:
            print(f"❌ 測試失敗: {e}")
            all_passed = False
    
    # 特別測試 foreach 場景
    print("\n" + "=" * 60)
    print("特別測試: foreach 參數解析")
    print("=" * 60)
    
    foreach_params = {
        'items': '${test_rooms}',
        'var': 'room_number',
        'steps': [
            {
                'id': 'log_room',
                'exec': 'action.log_message',
                'params': {
                    'message': 'Room: ${room_number}'
                }
            }
        ]
    }
    
    resolved_params = await executor.resolve_variables(foreach_params)
    
    print("\n原始參數:")
    print(json.dumps(foreach_params, indent=2, ensure_ascii=False))
    
    print("\n解析後參數:")
    print(json.dumps(resolved_params, indent=2, ensure_ascii=False, default=str))
    
    # 檢查 items 是否保持為陣列
    if isinstance(resolved_params['items'], list):
        print(f"\n✅ items 正確解析為陣列: {resolved_params['items']}")
    else:
        print(f"\n❌ items 錯誤地解析為: {type(resolved_params['items']).__name__}")
        print(f"   值: {resolved_params['items']}")
        all_passed = False
    
    # 總結
    print("\n" + "=" * 60)
    if all_passed:
        print("✅ 所有測試通過！變數解析功能正常")
    else:
        print("❌ 部分測試失敗，請檢查修復")
    print("=" * 60)
    
    return all_passed

if __name__ == "__main__":
    # 執行測試
    success = asyncio.run(test_variable_resolution())
    sys.exit(0 if success else 1)