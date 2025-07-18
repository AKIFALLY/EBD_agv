#!/usr/bin/env python3
"""
簡化的測試運行器，用於運行 loader_agv take_transfer 流程測試
"""

import sys
import os
import unittest
from unittest.mock import MagicMock

# 添加項目路徑到 Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

# 模擬所有外部依賴
def setup_mocks():
    """設置所有必要的 mock"""
    modules_to_mock = [
        'plc_proxy',
        'plc_proxy.plc_client',
        'rclpy',
        'rclpy.node',
        'rclpy.logging',
        'std_msgs',
        'std_msgs.msg',
        'agv_base',
        'agv_base.hokuyo_dms_8bit',
        'agv_base.robot',
        'agv_base.states',
        'agv_base.states.state',
        'agv_base.base_context',
        'db_proxy',
        'db_proxy.carrier_query_client',
        'db_proxy.eqp_signal_query_client',
        'db_proxy.agvc_database_client',
        'db_proxy.models',
    ]
    
    for module in modules_to_mock:
        sys.modules[module] = MagicMock()

def run_single_test(test_file):
    """運行單個測試文件"""
    print(f"\n{'='*60}")
    print(f"運行測試文件: {test_file}")
    print(f"{'='*60}")
    
    # 導入測試模組
    test_module_name = test_file.replace('.py', '')
    try:
        test_module = __import__(test_module_name)
        
        # 創建測試套件
        loader = unittest.TestLoader()
        suite = loader.loadTestsFromModule(test_module)
        
        # 運行測試
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        
        return result.wasSuccessful(), result.testsRun, len(result.failures), len(result.errors)
        
    except Exception as e:
        print(f"❌ 無法運行測試文件 {test_file}: {e}")
        return False, 0, 0, 1

def main():
    """主函數"""
    # 設置 mock
    setup_mocks()
    
    # 測試文件列表
    test_files = [
        'test_agv_port_check_empty_state.py',
        'test_transfer_check_have_state.py',
        'test_take_transfer_state.py',
        'test_put_agv_state.py',
        'test_transfer_vision_position_state.py',
        'test_take_transfer_integration.py',
    ]
    
    total_tests = 0
    total_failures = 0
    total_errors = 0
    successful_files = 0
    
    print("🚀 開始運行 loader_agv take_transfer 流程測試...")
    
    for test_file in test_files:
        if os.path.exists(test_file):
            success, tests, failures, errors = run_single_test(test_file)
            total_tests += tests
            total_failures += failures
            total_errors += errors
            
            if success:
                successful_files += 1
                print(f"✅ {test_file}: 通過")
            else:
                print(f"❌ {test_file}: 失敗")
        else:
            print(f"⚠️  測試文件不存在: {test_file}")
    
    # 總結報告
    print(f"\n{'='*60}")
    print("測試總結報告")
    print(f"{'='*60}")
    print(f"測試文件總數: {len(test_files)}")
    print(f"成功的文件: {successful_files}")
    print(f"失敗的文件: {len(test_files) - successful_files}")
    print(f"總測試數量: {total_tests}")
    print(f"失敗測試: {total_failures}")
    print(f"錯誤測試: {total_errors}")
    
    if total_failures == 0 and total_errors == 0:
        print("\n🎉 所有測試都通過了！")
        return True
    else:
        print(f"\n💥 有 {total_failures + total_errors} 個測試失敗或出錯")
        return False

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
