#!/usr/bin/env python3
"""
KUKA 整合測試運行腳本
執行完整的 KUKA 整合測試套件，包括單元測試和整合測試
"""
import os
import sys
import subprocess
import argparse
import time
import json
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any

# 添加路徑
sys.path.insert(0, str(Path(__file__).parent))
from mock_environment import MockTestEnvironment


class IntegrationTestRunner:
    """整合測試運行器"""
    
    def __init__(self):
        self.test_dir = Path(__file__).parent
        self.results = {
            "start_time": None,
            "end_time": None,
            "duration": 0,
            "tests": {
                "unit": {"passed": 0, "failed": 0, "skipped": 0},
                "integration": {"passed": 0, "failed": 0, "skipped": 0}
            },
            "coverage": {},
            "errors": []
        }
        self.mock_env = MockTestEnvironment()
    
    def setup_test_environment(self):
        """設置測試環境"""
        print("🔧 設置測試環境...")
        
        # 創建測試場景
        self._create_test_scenarios()
        
        # 設置環境變數
        os.environ['PYTEST_CURRENT_TEST'] = 'true'
        os.environ['RCS_TEST_MODE'] = '1'
        
        print("✅ 測試環境設置完成")
    
    def _create_test_scenarios(self):
        """創建測試場景"""
        
        # 場景1: 基本功能測試
        basic_scenario = {
            "name": "basic_functionality",
            "description": "基本功能測試場景",
            "agvs": [
                {
                    "id": 101,
                    "name": "KUKA101",
                    "model": "KUKA400i",
                    "enable": 1,
                    "status_id": 3,  # idle
                    "x": 100.0,
                    "y": 200.0,
                    "heading": 90.0,
                    "battery": 85
                }
            ],
            "tasks": [
                {
                    "id": 1,
                    "name": "測試移動任務",
                    "description": "基本移動任務測試",
                    "work_id": 210001,
                    "status_id": 1,  # pending
                    "priority": 50,
                    "room_id": None,
                    "agv_id": None,
                    "mission_code": None,
                    "parameters": {
                        "model": "KUKA400i",
                        "nodes": [10, 20, 30]
                    }
                }
            ],
            "racks": [
                {
                    "id": 1,
                    "name": "RACK001",
                    "agv_id": None,
                    "location_id": 10,
                    "product_id": None,
                    "is_carry": 0,
                    "is_in_map": 1,
                    "is_docked": 0,
                    "status_id": 1,
                    "direction": 0
                }
            ]
        }
        
        self.mock_env.add_test_scenario("basic", basic_scenario)
        
        # 場景2: 高負載測試
        high_load_scenario = {
            "name": "high_load",
            "description": "高負載測試場景",
            "agvs": [
                {
                    "id": 101 + i,
                    "name": f"KUKA{101 + i}",
                    "model": "KUKA400i",
                    "enable": 1,
                    "status_id": 3,
                    "x": 100.0 + i * 10,
                    "y": 200.0 + i * 10,
                    "heading": 90.0,
                    "battery": 80 - i * 5
                }
                for i in range(5)
            ],
            "tasks": [
                {
                    "id": i + 1,
                    "name": f"高負載任務{i + 1}",
                    "description": f"高負載測試任務{i + 1}",
                    "work_id": 220001,
                    "status_id": 1,
                    "priority": 50 + i,
                    "room_id": None,
                    "agv_id": None,
                    "mission_code": None,
                    "parameters": {
                        "model": "KUKA400i",
                        "nodes": [10 + i, 20 + i, 30 + i]
                    }
                }
                for i in range(10)
            ]
        }
        
        self.mock_env.add_test_scenario("high_load", high_load_scenario)
    
    def run_unit_tests(self, verbose: bool = False, coverage: bool = False) -> bool:
        """運行單元測試"""
        print("\n🧪 運行單元測試...")
        
        cmd = ["python", "-m", "pytest"]
        
        if verbose:
            cmd.append("-v")
        
        if coverage:
            cmd.extend([
                "--cov=rcs.kuka_manager",
                "--cov=rcs.kuka_robot",
                "--cov=rcs.kuka_container",
                "--cov-report=json:coverage_unit.json"
            ])
        
        # 添加單元測試檔案
        unit_test_files = [
            "test_kuka_robot.py",
            "test_kuka_container.py",
            "test_kuka_manager.py"
        ]
        
        for test_file in unit_test_files:
            cmd.append(str(self.test_dir / test_file))
        
        try:
            result = subprocess.run(
                cmd, 
                cwd=str(self.test_dir), 
                capture_output=True, 
                text=True,
                timeout=300  # 5 分鐘超時
            )
            
            if result.returncode == 0:
                print("✅ 單元測試通過")
                self._parse_test_results(result.stdout, "unit")
                return True
            else:
                print("❌ 單元測試失敗")
                print(result.stdout)
                print(result.stderr)
                self.results["errors"].append({
                    "type": "unit_test_failure",
                    "message": result.stderr
                })
                return False
                
        except subprocess.TimeoutExpired:
            print("❌ 單元測試超時")
            self.results["errors"].append({
                "type": "unit_test_timeout",
                "message": "Unit tests timed out after 5 minutes"
            })
            return False
        except Exception as e:
            print(f"❌ 單元測試執行錯誤: {e}")
            self.results["errors"].append({
                "type": "unit_test_error",
                "message": str(e)
            })
            return False
    
    def run_integration_tests(self, verbose: bool = False, coverage: bool = False) -> bool:
        """運行整合測試"""
        print("\n🔗 運行整合測試...")
        
        cmd = ["python", "-m", "pytest"]
        
        if verbose:
            cmd.append("-v")
        
        if coverage:
            cmd.extend([
                "--cov=rcs.kuka_manager",
                "--cov=rcs.kuka_robot",
                "--cov=rcs.kuka_container",
                "--cov-append",
                "--cov-report=json:coverage_integration.json"
            ])
        
        # 添加整合測試檔案
        cmd.append(str(self.test_dir / "test_kuka_integration.py"))
        
        try:
            result = subprocess.run(
                cmd,
                cwd=str(self.test_dir),
                capture_output=True,
                text=True,
                timeout=600  # 10 分鐘超時
            )
            
            if result.returncode == 0:
                print("✅ 整合測試通過")
                self._parse_test_results(result.stdout, "integration")
                return True
            else:
                print("❌ 整合測試失敗")
                print(result.stdout)
                print(result.stderr)
                self.results["errors"].append({
                    "type": "integration_test_failure",
                    "message": result.stderr
                })
                return False
                
        except subprocess.TimeoutExpired:
            print("❌ 整合測試超時")
            self.results["errors"].append({
                "type": "integration_test_timeout",
                "message": "Integration tests timed out after 10 minutes"
            })
            return False
        except Exception as e:
            print(f"❌ 整合測試執行錯誤: {e}")
            self.results["errors"].append({
                "type": "integration_test_error",
                "message": str(e)
            })
            return False
    
    def run_scenario_tests(self, scenario: str = "basic") -> bool:
        """運行場景測試"""
        print(f"\n🎭 運行場景測試: {scenario}")
        
        try:
            # 載入測試場景
            self.mock_env.load_scenario(scenario)
            
            # 匯出場景狀態供測試使用
            scenario_file = self.test_dir / f"scenario_{scenario}.json"
            self.mock_env.export_state(str(scenario_file))
            
            # 運行場景相關測試
            cmd = [
                "python", "-m", "pytest", 
                "-v",
                "-k", f"test_{scenario}",
                str(self.test_dir / "test_kuka_integration.py")
            ]
            
            result = subprocess.run(
                cmd,
                cwd=str(self.test_dir),
                capture_output=True,
                text=True,
                timeout=300
            )
            
            # 清理場景檔案
            if scenario_file.exists():
                scenario_file.unlink()
            
            if result.returncode == 0:
                print(f"✅ 場景測試 {scenario} 通過")
                return True
            else:
                print(f"❌ 場景測試 {scenario} 失敗")
                print(result.stdout)
                print(result.stderr)
                return False
                
        except Exception as e:
            print(f"❌ 場景測試執行錯誤: {e}")
            return False
    
    def _parse_test_results(self, stdout: str, test_type: str):
        """解析測試結果"""
        lines = stdout.split('\n')
        
        for line in lines:
            if "passed" in line and "failed" in line:
                # 解析 pytest 結果行，如: "5 passed, 1 failed, 2 skipped"
                parts = line.split(',')
                for part in parts:
                    part = part.strip()
                    if "passed" in part:
                        count = int(part.split()[0])
                        self.results["tests"][test_type]["passed"] = count
                    elif "failed" in part:
                        count = int(part.split()[0])
                        self.results["tests"][test_type]["failed"] = count
                    elif "skipped" in part:
                        count = int(part.split()[0])
                        self.results["tests"][test_type]["skipped"] = count
                break
    
    def generate_report(self, output_file: str = None):
        """生成測試報告"""
        print("\n📊 生成測試報告...")
        
        self.results["end_time"] = datetime.now().isoformat()
        
        if self.results["start_time"]:
            start = datetime.fromisoformat(self.results["start_time"])
            end = datetime.fromisoformat(self.results["end_time"])
            self.results["duration"] = (end - start).total_seconds()
        
        # 計算總計
        total_passed = (self.results["tests"]["unit"]["passed"] + 
                       self.results["tests"]["integration"]["passed"])
        total_failed = (self.results["tests"]["unit"]["failed"] + 
                       self.results["tests"]["integration"]["failed"])
        total_skipped = (self.results["tests"]["unit"]["skipped"] + 
                        self.results["tests"]["integration"]["skipped"])
        
        # 生成報告
        report = f"""
# KUKA 整合測試報告

## 測試概要
- **測試時間**: {self.results["start_time"]} - {self.results["end_time"]}
- **測試持續時間**: {self.results["duration"]:.2f} 秒
- **總測試數**: {total_passed + total_failed + total_skipped}
- **通過**: {total_passed}
- **失敗**: {total_failed}
- **跳過**: {total_skipped}
- **成功率**: {(total_passed / max(total_passed + total_failed, 1) * 100):.1f}%

## 單元測試結果
- **通過**: {self.results["tests"]["unit"]["passed"]}
- **失敗**: {self.results["tests"]["unit"]["failed"]}
- **跳過**: {self.results["tests"]["unit"]["skipped"]}

## 整合測試結果
- **通過**: {self.results["tests"]["integration"]["passed"]}
- **失敗**: {self.results["tests"]["integration"]["failed"]}
- **跳過**: {self.results["tests"]["integration"]["skipped"]}

## 錯誤記錄
"""
        
        if self.results["errors"]:
            for error in self.results["errors"]:
                report += f"- **{error['type']}**: {error['message']}\n"
        else:
            report += "無錯誤記錄\n"
        
        # 輸出報告
        if output_file:
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(report)
            print(f"📝 報告已儲存至: {output_file}")
        else:
            print(report)
        
        # 同時生成 JSON 格式報告
        json_file = output_file.replace('.md', '.json') if output_file else 'test_results.json'
        with open(json_file, 'w', encoding='utf-8') as f:
            json.dump(self.results, f, indent=2, default=str)
        print(f"📊 JSON 報告已儲存至: {json_file}")
    
    def run_all_tests(self, verbose: bool = False, coverage: bool = False, scenarios: List[str] = None) -> bool:
        """運行所有測試"""
        print("🚀 開始 KUKA 整合測試套件")
        print("=" * 60)
        
        self.results["start_time"] = datetime.now().isoformat()
        
        # 設置測試環境
        self.setup_test_environment()
        
        success = True
        
        # 運行單元測試
        if not self.run_unit_tests(verbose, coverage):
            success = False
        
        # 運行整合測試
        if not self.run_integration_tests(verbose, coverage):
            success = False
        
        # 運行場景測試
        if scenarios:
            for scenario in scenarios:
                if not self.run_scenario_tests(scenario):
                    success = False
        
        return success


def main():
    """主函數"""
    parser = argparse.ArgumentParser(description="運行 KUKA 整合測試套件")
    parser.add_argument("-v", "--verbose", action="store_true", help="詳細輸出")
    parser.add_argument("--coverage", action="store_true", help="生成覆蓋率報告")
    parser.add_argument("--unit-only", action="store_true", help="只運行單元測試")
    parser.add_argument("--integration-only", action="store_true", help="只運行整合測試")
    parser.add_argument("--scenarios", nargs="*", help="運行指定場景測試", 
                       choices=["basic", "high_load"], default=["basic"])
    parser.add_argument("--output", help="測試報告輸出檔案", default="kuka_test_report.md")
    
    args = parser.parse_args()
    
    runner = IntegrationTestRunner()
    
    try:
        if args.unit_only:
            success = runner.run_unit_tests(args.verbose, args.coverage)
        elif args.integration_only:
            success = runner.run_integration_tests(args.verbose, args.coverage)
        else:
            success = runner.run_all_tests(args.verbose, args.coverage, args.scenarios)
        
        # 生成報告
        runner.generate_report(args.output)
        
        if success:
            print("\n🎉 所有測試完成！")
            sys.exit(0)
        else:
            print("\n❌ 部分測試失敗")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n⚠️ 測試被使用者中斷")
        sys.exit(1)
    except Exception as e:
        print(f"\n💥 測試執行過程中發生未預期錯誤: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()