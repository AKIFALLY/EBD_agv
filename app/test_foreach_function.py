#!/usr/bin/env python3
"""
測試 Foreach 函數功能
測試檔案：專門用於驗證 flow_wcs 的 foreach 實現
創建日期：2025-01-13
狀態：臨時測試檔案
"""

import sys
import json
from pathlib import Path

# Add the flow_wcs module to path
sys.path.insert(0, '/app/flow_wcs_ws/src')

from flow_wcs.flow_wcs.flow_executor import FlowExecutor

def test_foreach_basic():
    """測試基本的 foreach 功能"""
    print("\n=== 測試基本 Foreach 功能 ===")
    
    # 創建測試流程
    test_flow = {
        'flow': {
            'id': 'test_foreach_basic',
            'name': 'Basic Foreach Test',
            'work_id': '999001'
        },
        'variables': {
            'test_items': [1, 2, 3, 4, 5]
        }
    }
    
    # 創建執行器
    executor = FlowExecutor(test_flow)
    
    # 測試 foreach 函數
    params = {
        'items': [1, 2, 3, 4, 5],
        'var': 'number',
        'steps': [
            {'id': 'step1', 'exec': 'action.log', 'params': {'message': 'Processing ${number}'}}
        ],
        'max_iterations': 10
    }
    
    try:
        result = executor.foreach(params)
        print(f"✅ Foreach 執行成功")
        print(f"   處理了 {len(result)} 個項目")
        for i, r in enumerate(result):
            print(f"   - 項目 {i+1}: {r['item']}")
    except Exception as e:
        print(f"❌ Foreach 執行失敗: {e}")
        return False
    
    return True

def test_foreach_with_objects():
    """測試處理物件陣列的 foreach"""
    print("\n=== 測試物件陣列 Foreach ===")
    
    test_flow = {
        'flow': {
            'id': 'test_foreach_objects',
            'name': 'Object Foreach Test',
            'work_id': '999002'
        },
        'variables': {}
    }
    
    executor = FlowExecutor(test_flow)
    
    # 測試物件陣列
    test_objects = [
        {'id': 'loc1', 'room': 1, 'type': 'inlet'},
        {'id': 'loc2', 'room': 2, 'type': 'outlet'},
        {'id': 'loc3', 'room': 3, 'type': 'inlet'}
    ]
    
    params = {
        'items': test_objects,
        'var': 'location',
        'steps': [
            {'id': 'log_location', 'exec': 'action.log', 
             'params': {'message': 'Location ${location.id} in room ${location.room}'}}
        ]
    }
    
    try:
        result = executor.foreach(params)
        print(f"✅ 物件陣列 Foreach 執行成功")
        print(f"   處理了 {len(result)} 個位置物件")
        for r in result:
            loc = r['item']
            print(f"   - {loc['id']}: Room {loc['room']}, Type {loc['type']}")
    except Exception as e:
        print(f"❌ 物件陣列 Foreach 執行失敗: {e}")
        return False
    
    return True

def test_foreach_empty_items():
    """測試空陣列的 foreach"""
    print("\n=== 測試空陣列 Foreach ===")
    
    test_flow = {
        'flow': {
            'id': 'test_foreach_empty',
            'name': 'Empty Foreach Test',
            'work_id': '999003'
        },
        'variables': {}
    }
    
    executor = FlowExecutor(test_flow)
    
    params = {
        'items': [],
        'var': 'item',
        'steps': [
            {'id': 'should_not_run', 'exec': 'action.log', 'params': {'message': 'This should not run'}}
        ]
    }
    
    try:
        result = executor.foreach(params)
        print(f"✅ 空陣列 Foreach 正確處理")
        print(f"   結果: {len(result)} 個項目（預期為0）")
        if len(result) == 0:
            print("   ✓ 正確返回空結果")
        else:
            print("   ✗ 錯誤：不應該有結果")
            return False
    except Exception as e:
        print(f"❌ 空陣列 Foreach 執行失敗: {e}")
        return False
    
    return True

def test_foreach_max_iterations():
    """測試最大迭代限制"""
    print("\n=== 測試最大迭代限制 ===")
    
    test_flow = {
        'flow': {
            'id': 'test_foreach_limit',
            'name': 'Max Iterations Test',
            'work_id': '999004'
        },
        'variables': {}
    }
    
    executor = FlowExecutor(test_flow)
    
    # 創建超過限制的項目
    large_array = list(range(1, 11))  # 10 items
    
    params = {
        'items': large_array,
        'var': 'num',
        'steps': [],
        'max_iterations': 5  # 限制為5
    }
    
    try:
        result = executor.foreach(params)
        print(f"✅ 最大迭代限制測試成功")
        print(f"   原始項目: {len(large_array)} 個")
        print(f"   實際處理: {len(result)} 個（限制為5）")
        if len(result) == 5:
            print("   ✓ 正確限制迭代次數")
        else:
            print(f"   ✗ 錯誤：預期5個結果，實際{len(result)}個")
            return False
    except Exception as e:
        print(f"❌ 最大迭代限制測試失敗: {e}")
        return False
    
    return True

def main():
    """執行所有測試"""
    print("=" * 50)
    print("開始測試 Foreach 功能")
    print("=" * 50)
    
    tests = [
        test_foreach_basic,
        test_foreach_with_objects,
        test_foreach_empty_items,
        test_foreach_max_iterations
    ]
    
    results = []
    for test in tests:
        result = test()
        results.append(result)
    
    print("\n" + "=" * 50)
    print("測試結果總結")
    print("=" * 50)
    
    passed = sum(results)
    total = len(results)
    
    print(f"通過測試: {passed}/{total}")
    
    if passed == total:
        print("🎉 所有測試通過！Foreach 功能運作正常")
        return 0
    else:
        print("⚠️ 部分測試失敗，請檢查實現")
        return 1

if __name__ == "__main__":
    sys.exit(main())