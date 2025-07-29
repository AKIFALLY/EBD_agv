#!/usr/bin/env python3
"""
AI WCS 統一測試執行器
提供分類測試執行和報告生成功能
"""

import sys
import os
import unittest
import subprocess
import argparse
from typing import List, Dict, Any
from pathlib import Path

# 添加路徑
sys.path.append('/app/ai_wcs_ws/src')

# 測試類別定義
TEST_CATEGORIES = {
    'unit': {
        'description': '單元測試 - 測試單一類別和函數的基本功能',
        'path': 'unit',
        'tests': [
            'test_business_flow_priority.py',
            'test_task_decision.py', 
            'test_work_id_category.py',
            'test_parameter_manager_unit.py'
        ]
    },
    'integration': {
        'description': '整合測試 - 測試模組間協作的整合功能',
        'path': 'integration',
        'tests': [
            'test_decision_engine_integration.py'
        ]
    },
    'functional': {
        'description': '功能測試 - 測試完整業務流程的端到端功能',
        'path': 'functional',
        'tests': []  # 待建立
    }
}


class AIWCSTestRunner:
    """AI WCS 測試執行器"""
    
    def __init__(self):
        self.test_dir = Path(__file__).parent
        self.results = {}
    
    def run_category_tests(self, category: str) -> Dict[str, Any]:
        """執行特定類別的測試"""
        if category not in TEST_CATEGORIES:
            print(f"❌ 不支援的測試類別: {category}")
            return {'success': False, 'error': 'Invalid category'}
        
        category_info = TEST_CATEGORIES[category]
        category_path = self.test_dir / category_info['path']
        
        if not category_path.exists():
            print(f"❌ 測試目錄不存在: {category_path}")
            return {'success': False, 'error': 'Directory not found'}
        
        print(f"\n🧪 執行 {category.upper()} 測試")
        print("=" * 60)
        print(f"📋 {category_info['description']}")
        print(f"📂 路徑: {category_path}")
        print("=" * 60)
        
        total_tests = 0
        passed_tests = 0
        failed_tests = 0
        test_results = []
        
        for test_file in category_info['tests']:
            test_path = category_path / test_file
            
            if not test_path.exists():
                print(f"⚠️  測試檔案不存在: {test_file}")
                continue
            
            print(f"\n🔸 執行測試: {test_file}")
            
            try:
                # 執行測試
                result = subprocess.run(
                    [sys.executable, str(test_path)],
                    cwd=str(category_path),
                    capture_output=True,
                    text=True,
                    timeout=60
                )
                
                # 分析結果
                if result.returncode == 0:
                    print(f"✅ {test_file} - 通過")
                    status = 'PASSED'
                    passed_tests += 1
                else:
                    print(f"❌ {test_file} - 失敗")
                    if result.stderr:
                        print(f"   錯誤: {result.stderr.strip()}")
                    status = 'FAILED'
                    failed_tests += 1
                
                # 提取測試數量
                output_lines = result.stdout.split('\n')
                for line in output_lines:
                    if 'Ran' in line and 'test' in line:
                        try:
                            test_count = int(line.split()[1])
                            total_tests += test_count
                        except (IndexError, ValueError):
                            pass
                
                test_results.append({
                    'file': test_file,
                    'status': status,
                    'output': result.stdout,
                    'error': result.stderr
                })
                
            except subprocess.TimeoutExpired:
                print(f"⏰ {test_file} - 超時")
                failed_tests += 1
                test_results.append({
                    'file': test_file,
                    'status': 'TIMEOUT',
                    'output': '',
                    'error': 'Test timed out after 60 seconds'
                })
            except Exception as e:
                print(f"💥 {test_file} - 執行錯誤: {e}")
                failed_tests += 1
                test_results.append({
                    'file': test_file,
                    'status': 'ERROR',
                    'output': '',
                    'error': str(e)
                })
        
        # 生成結果摘要
        success_rate = (passed_tests / len(category_info['tests']) * 100) if category_info['tests'] else 0
        
        result = {
            'category': category,
            'success': failed_tests == 0,
            'total_files': len(category_info['tests']),
            'passed_files': passed_tests,
            'failed_files': failed_tests,
            'total_tests': total_tests,
            'success_rate': success_rate,
            'test_results': test_results
        }
        
        self.print_category_summary(result)
        return result
    
    def print_category_summary(self, result: Dict[str, Any]):
        """列印類別測試摘要"""
        print(f"\n📊 {result['category'].upper()} 測試結果摘要")
        print("-" * 40)
        print(f"📁 測試檔案: {result['passed_files']}/{result['total_files']} 通過")
        print(f"🧪 測試案例: {result['total_tests']} 個")
        print(f"📈 成功率: {result['success_rate']:.1f}%")
        
        if result['success']:
            print("🎉 所有測試都通過！")
        else:
            print(f"⚠️  有 {result['failed_files']} 個測試檔案失敗")
    
    def run_all_tests(self) -> Dict[str, Any]:
        """執行所有類別的測試"""
        print("🚀 開始執行 AI WCS 完整測試套件")
        print("=" * 60)
        
        all_results = {}
        total_passed = 0
        total_failed = 0
        
        for category in TEST_CATEGORIES.keys():
            result = self.run_category_tests(category)
            all_results[category] = result
            
            if result['success']:
                total_passed += 1
            else:
                total_failed += 1
        
        # 生成總體摘要
        overall_result = {
            'success': total_failed == 0,
            'categories': all_results,
            'total_categories': len(TEST_CATEGORIES),
            'passed_categories': total_passed,
            'failed_categories': total_failed
        }
        
        self.print_overall_summary(overall_result)
        return overall_result
    
    def print_overall_summary(self, result: Dict[str, Any]):
        """列印總體測試摘要"""
        print("\n" + "=" * 60)
        print("📊 AI WCS 測試總體結果摘要")
        print("=" * 60)
        
        for category, category_result in result['categories'].items():
            status = "✅" if category_result['success'] else "❌"
            print(f"{status} {category.upper()}: {category_result['passed_files']}/{category_result['total_files']} 檔案通過")
        
        print(f"\n📈 總體統計:")
        print(f"   測試類別: {result['passed_categories']}/{result['total_categories']} 通過")
        
        if result['success']:
            print("\n🎉 所有測試類別都通過！AI WCS 功能正常運作")
        else:
            print(f"\n⚠️  有 {result['failed_categories']} 個測試類別失敗，需要進一步檢查")
    
    def list_available_tests(self):
        """列出可用的測試"""
        print("📋 AI WCS 可用測試類別:")
        print("=" * 60)
        
        for category, info in TEST_CATEGORIES.items():
            print(f"\n🔸 {category.upper()}")
            print(f"   📝 {info['description']}")
            print(f"   📂 路徑: {info['path']}/")
            
            if info['tests']:
                print(f"   🧪 測試檔案:")
                for test_file in info['tests']:
                    test_path = self.test_dir / info['path'] / test_file
                    status = "✅" if test_path.exists() else "❌"
                    print(f"      {status} {test_file}")
            else:
                print(f"   ⚠️  暫無測試檔案")


def main():
    """主函數"""
    parser = argparse.ArgumentParser(
        description='AI WCS 統一測試執行器',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
測試類別:
  unit          單元測試 - 測試單一類別和函數的基本功能
  integration   整合測試 - 測試模組間協作的整合功能  
  functional    功能測試 - 測試完整業務流程的端到端功能
  all           執行所有測試類別

範例:
  python3 run_tests.py unit                # 執行單元測試
  python3 run_tests.py integration         # 執行整合測試
  python3 run_tests.py all                 # 執行所有測試
  python3 run_tests.py --list              # 列出可用測試
        """
    )
    
    parser.add_argument(
        'category',
        nargs='?',
        choices=['unit', 'integration', 'functional', 'all'],
        default='all',
        help='要執行的測試類別 (預設: all)'
    )
    
    parser.add_argument(
        '--list', '-l',
        action='store_true',
        help='列出可用的測試類別和檔案'
    )
    
    args = parser.parse_args()
    
    runner = AIWCSTestRunner()
    
    if args.list:
        runner.list_available_tests()
        return
    
    if args.category == 'all':
        result = runner.run_all_tests()
    else:
        result = runner.run_category_tests(args.category)
    
    # 根據結果設定退出碼
    exit_code = 0 if result['success'] else 1
    sys.exit(exit_code)


if __name__ == '__main__':
    main()