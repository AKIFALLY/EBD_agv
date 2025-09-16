#!/usr/bin/env python3
"""
測試 control.switch 函數修復
"""
import sys
import os
sys.path.insert(0, '/app/flow_wcs_ws/src/flow_wcs')

def test_control_switch():
    """測試 control.switch 函數的物件格式支援"""
    print("\n=== 測試 control.switch 函數 ===\n")
    
    # 導入模組
    from flow_wcs.flow_executor import FlowExecutor
    import yaml
    
    # 創建一個簡單的測試流程
    test_flow = {
        'flow_id': 'test_switch',
        'steps': [
            {
                'id': 'test_switch_4',
                'exec': 'control.switch',
                'params': {
                    'value': 4,
                    'cases': {
                        '4': True,
                        '5': True,
                        'default': False
                    }
                },
                'store': 'result'
            }
        ]
    }
    
    # 創建執行器
    executor = FlowExecutor(test_flow)
    
    # 手動測試 switch 函數
    test_params = {
        'value': 4,
        'cases': {
            '4': True,
            '5': True,
            'default': False
        }
    }
    
    result = executor.switch(test_params)
    print(f"測試 value=4: {result}")
    assert result == True, f"Expected True, got {result}"
    
    # 測試 value=5
    test_params['value'] = 5
    result = executor.switch(test_params)
    print(f"測試 value=5: {result}")
    assert result == True, f"Expected True, got {result}"
    
    # 測試 value=3 (應該返回 default)
    test_params['value'] = 3
    result = executor.switch(test_params)
    print(f"測試 value=3: {result}")
    assert result == False, f"Expected False, got {result}"
    
    print("\n✅ control.switch 函數測試通過！")
    print("✅ 物件格式的 cases 參數支援正常運作")
    
    return True

if __name__ == "__main__":
    try:
        test_control_switch()
        print("\n🎉 測試成功！control.switch 函數已修復")
    except Exception as e:
        print(f"\n❌ 測試失敗: {e}")
        sys.exit(1)