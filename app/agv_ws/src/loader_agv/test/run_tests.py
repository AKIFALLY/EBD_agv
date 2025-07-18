#!/usr/bin/env python3
"""
運行 loader_agv take_transfer 流程的所有測試
"""

import sys
import os
import unittest
import coverage

# 添加項目路徑到 Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def run_tests_with_coverage():
    """運行測試並生成覆蓋率報告"""
    
    # 初始化覆蓋率測量
    cov = coverage.Coverage(
        source=['loader_agv.robot_states.take_transfer'],
        omit=[
            '*/test*',
            '*/__pycache__/*',
            '*/.*'
        ]
    )
    cov.start()
    
    # 發現並運行測試
    loader = unittest.TestLoader()
    start_dir = os.path.dirname(__file__)
    suite = loader.discover(start_dir, pattern='test_*.py')
    
    # 運行測試
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 停止覆蓋率測量
    cov.stop()
    cov.save()
    
    # 生成覆蓋率報告
    print("\n" + "="*50)
    print("覆蓋率報告")
    print("="*50)
    cov.report()
    
    # 生成 HTML 報告
    try:
        cov.html_report(directory='htmlcov')
        print(f"\nHTML 覆蓋率報告已生成到: {os.path.join(start_dir, 'htmlcov')}")
    except Exception as e:
        print(f"生成 HTML 報告時出錯: {e}")
    
    # 返回測試結果
    return result.wasSuccessful()

def run_specific_test(test_name):
    """運行特定的測試"""
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromName(test_name)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result.wasSuccessful()

def main():
    """主函數"""
    if len(sys.argv) > 1:
        # 運行特定測試
        test_name = sys.argv[1]
        print(f"運行特定測試: {test_name}")
        success = run_specific_test(test_name)
    else:
        # 運行所有測試
        print("運行所有 take_transfer 流程測試...")
        success = run_tests_with_coverage()
    
    if success:
        print("\n✅ 所有測試通過！")
        sys.exit(0)
    else:
        print("\n❌ 有測試失敗！")
        sys.exit(1)

if __name__ == '__main__':
    main()
