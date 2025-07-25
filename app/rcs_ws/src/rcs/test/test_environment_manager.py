#!/usr/bin/env python3
"""
測試環境管理器
提供測試環境配置、管理和監控功能
"""
import os
import json
import yaml
import argparse
import subprocess
from pathlib import Path
from datetime import datetime, timezone
from typing import Dict, List, Any, Optional

from mock_environment import MockTestEnvironment


class TestEnvironmentManager:
    """測試環境管理器"""
    
    def __init__(self):
        self.test_dir = Path(__file__).parent
        self.config_dir = self.test_dir / "configs"
        self.config_dir.mkdir(exist_ok=True)
        
        self.scenarios_dir = self.test_dir / "scenarios"
        self.scenarios_dir.mkdir(exist_ok=True)
        
        self.logs_dir = self.test_dir / "logs"
        self.logs_dir.mkdir(exist_ok=True)
        
        self.mock_env = MockTestEnvironment()
    
    def create_scenario(self, name: str, description: str = "", 
                       agvs: List[Dict] = None, tasks: List[Dict] = None, 
                       racks: List[Dict] = None) -> bool:
        """創建新的測試場景"""
        try:
            scenario_data = {
                "name": name,
                "description": description,
                "created_at": datetime.now(timezone.utc).isoformat(),
                "agvs": agvs or [],
                "tasks": tasks or [],
                "racks": racks or []
            }
            
            scenario_file = self.scenarios_dir / f"{name}.json"
            with open(scenario_file, 'w', encoding='utf-8') as f:
                json.dump(scenario_data, f, indent=2, default=str)
            
            print(f"✅ 場景 '{name}' 已創建: {scenario_file}")
            return True
            
        except Exception as e:
            print(f"❌ 創建場景失敗: {e}")
            return False
    
    def list_scenarios(self) -> List[Dict[str, Any]]:
        """列出所有可用場景"""
        scenarios = []
        
        for scenario_file in self.scenarios_dir.glob("*.json"):
            try:
                with open(scenario_file, 'r', encoding='utf-8') as f:
                    scenario_data = json.load(f)
                    scenarios.append({
                        "name": scenario_data.get("name", scenario_file.stem),
                        "description": scenario_data.get("description", ""),
                        "file": str(scenario_file),
                        "agv_count": len(scenario_data.get("agvs", [])),
                        "task_count": len(scenario_data.get("tasks", [])),
                        "rack_count": len(scenario_data.get("racks", []))
                    })
            except Exception as e:
                print(f"⚠️ 讀取場景檔案 {scenario_file} 失敗: {e}")
        
        return scenarios
    
    def load_scenario(self, name: str) -> bool:
        """載入指定場景"""
        try:
            scenario_file = self.scenarios_dir / f"{name}.json"
            if not scenario_file.exists():
                print(f"❌ 場景檔案不存在: {scenario_file}")
                return False
            
            with open(scenario_file, 'r', encoding='utf-8') as f:
                scenario_data = json.load(f)
            
            self.mock_env.add_test_scenario(name, scenario_data)
            self.mock_env.load_scenario(name)
            
            print(f"✅ 場景 '{name}' 已載入")
            return True
            
        except Exception as e:
            print(f"❌ 載入場景失敗: {e}")
            return False
    
    def export_scenario(self, name: str, output_file: str = None) -> bool:
        """匯出場景到檔案"""
        try:
            if not output_file:
                output_file = f"scenario_{name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            
            # 載入場景
            if not self.load_scenario(name):
                return False
            
            # 匯出狀態
            self.mock_env.export_state(output_file)
            
            print(f"✅ 場景 '{name}' 已匯出至: {output_file}")
            return True
            
        except Exception as e:
            print(f"❌ 匯出場景失敗: {e}")
            return False
    
    def import_scenario(self, file_path: str, name: str = None) -> bool:
        """從檔案匯入場景"""
        try:
            if not Path(file_path).exists():
                print(f"❌ 檔案不存在: {file_path}")
                return False
            
            with open(file_path, 'r', encoding='utf-8') as f:
                state_data = json.load(f)
            
            scenario_name = name or Path(file_path).stem
            
            # 轉換狀態數據為場景格式
            scenario_data = {
                "name": scenario_name,
                "description": f"從 {file_path} 匯入",
                "created_at": datetime.now(timezone.utc).isoformat(),
                "agvs": state_data.get("agvs", []),
                "tasks": state_data.get("tasks", []),
                "racks": state_data.get("racks", [])
            }
            
            # 儲存場景
            scenario_file = self.scenarios_dir / f"{scenario_name}.json"
            with open(scenario_file, 'w', encoding='utf-8') as f:
                json.dump(scenario_data, f, indent=2, default=str)
            
            print(f"✅ 場景 '{scenario_name}' 已從 {file_path} 匯入")
            return True
            
        except Exception as e:
            print(f"❌ 匯入場景失敗: {e}")
            return False
    
    def validate_scenario(self, name: str) -> bool:
        """驗證場景資料完整性"""
        try:
            scenario_file = self.scenarios_dir / f"{name}.json"
            if not scenario_file.exists():
                print(f"❌ 場景檔案不存在: {scenario_file}")
                return False
            
            with open(scenario_file, 'r', encoding='utf-8') as f:
                scenario_data = json.load(f)
            
            errors = []
            
            # 驗證必要欄位
            required_fields = ["name", "agvs", "tasks", "racks"]
            for field in required_fields:
                if field not in scenario_data:
                    errors.append(f"缺少必要欄位: {field}")
            
            # 驗證 AGV 資料
            for i, agv in enumerate(scenario_data.get("agvs", [])):
                agv_required = ["id", "name", "model", "enable", "status_id"]
                for field in agv_required:
                    if field not in agv:
                        errors.append(f"AGV {i} 缺少欄位: {field}")
            
            # 驗證任務資料
            for i, task in enumerate(scenario_data.get("tasks", [])):
                task_required = ["id", "name", "work_id", "status_id", "parameters"]
                for field in task_required:
                    if field not in task:
                        errors.append(f"Task {i} 缺少欄位: {field}")
            
            # 驗證 Rack 資料
            for i, rack in enumerate(scenario_data.get("racks", [])):
                rack_required = ["id", "name", "location_id", "is_carry", "is_in_map"]
                for field in rack_required:
                    if field not in rack:
                        errors.append(f"Rack {i} 缺少欄位: {field}")
            
            if errors:
                print(f"❌ 場景 '{name}' 驗證失敗:")
                for error in errors:
                    print(f"  - {error}")
                return False
            else:
                print(f"✅ 場景 '{name}' 驗證通過")
                return True
                
        except Exception as e:
            print(f"❌ 驗證場景失敗: {e}")
            return False
    
    def run_test_with_scenario(self, scenario_name: str, test_file: str = None, 
                              verbose: bool = False) -> bool:
        """使用指定場景運行測試"""
        try:
            # 載入場景
            if not self.load_scenario(scenario_name):
                return False
            
            # 匯出場景狀態供測試使用
            temp_scenario_file = self.test_dir / f"temp_scenario_{scenario_name}.json"
            self.mock_env.export_state(str(temp_scenario_file))
            
            # 設置環境變數
            env = os.environ.copy()
            env['KUKA_TEST_SCENARIO'] = scenario_name
            env['KUKA_TEST_SCENARIO_FILE'] = str(temp_scenario_file)
            
            # 建立測試命令
            cmd = ["python", "-m", "pytest"]
            if verbose:
                cmd.append("-v")
            
            if test_file:
                cmd.append(test_file)
            else:
                cmd.extend([
                    str(self.test_dir / "test_kuka_integration.py"),
                    str(self.test_dir / "test_kuka_manager.py")
                ])
            
            print(f"🚀 使用場景 '{scenario_name}' 運行測試...")
            
            # 執行測試
            result = subprocess.run(
                cmd,
                cwd=str(self.test_dir),
                env=env,
                capture_output=True,
                text=True
            )
            
            # 清理臨時檔案
            if temp_scenario_file.exists():
                temp_scenario_file.unlink()
            
            if result.returncode == 0:
                print("✅ 測試通過")
                if verbose:
                    print(result.stdout)
                return True
            else:
                print("❌ 測試失敗")
                print(result.stdout)
                print(result.stderr)
                return False
                
        except Exception as e:
            print(f"❌ 運行測試失敗: {e}")
            return False
    
    def start_mock_server(self, host: str = "localhost", port: int = 8080, 
                         scenario: str = None) -> bool:
        """啟動模擬服務器"""
        try:
            # 如果指定了場景，先載入
            if scenario:
                if not self.load_scenario(scenario):
                    return False
                print(f"📋 使用場景: {scenario}")
            
            # 啟動服務器
            cmd = [
                "python", str(self.test_dir / "offline_test_server.py"),
                "--host", host,
                "--port", str(port)
            ]
            
            print(f"🚀 啟動模擬服務器於 http://{host}:{port}")
            subprocess.run(cmd, cwd=str(self.test_dir))
            
            return True
            
        except KeyboardInterrupt:
            print("\n⚠️ 服務器已停止")
            return True
        except Exception as e:
            print(f"❌ 啟動模擬服務器失敗: {e}")
            return False
    
    def generate_test_report(self, output_dir: str = None) -> bool:
        """生成測試環境報告"""
        try:
            if not output_dir:
                output_dir = str(self.logs_dir)
            
            output_path = Path(output_dir)
            output_path.mkdir(exist_ok=True)
            
            # 生成場景報告
            scenarios = self.list_scenarios()
            
            report = {
                "report_type": "test_environment",
                "generated_at": datetime.now(timezone.utc).isoformat(),
                "scenarios": {
                    "total_count": len(scenarios),
                    "scenarios": scenarios
                },
                "environment_info": {
                    "test_dir": str(self.test_dir),
                    "config_dir": str(self.config_dir),
                    "scenarios_dir": str(self.scenarios_dir),
                    "logs_dir": str(self.logs_dir)
                }
            }
            
            # 儲存 JSON 報告
            report_file = output_path / f"test_environment_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(report_file, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2, default=str)
            
            print(f"✅ 測試環境報告已生成: {report_file}")
            return True
            
        except Exception as e:
            print(f"❌ 生成報告失敗: {e}")
            return False


def main():
    """主函數"""
    parser = argparse.ArgumentParser(description="KUKA 測試環境管理器")
    subparsers = parser.add_subparsers(dest='command', help='可用命令')
    
    # 創建場景
    create_parser = subparsers.add_parser('create', help='創建新場景')
    create_parser.add_argument('name', help='場景名稱')
    create_parser.add_argument('--description', help='場景描述')
    
    # 列出場景
    subparsers.add_parser('list', help='列出所有場景')
    
    # 載入場景
    load_parser = subparsers.add_parser('load', help='載入場景')
    load_parser.add_argument('name', help='場景名稱')
    
    # 驗證場景
    validate_parser = subparsers.add_parser('validate', help='驗證場景')
    validate_parser.add_argument('name', help='場景名稱')
    
    # 匯出場景
    export_parser = subparsers.add_parser('export', help='匯出場景')
    export_parser.add_argument('name', help='場景名稱')
    export_parser.add_argument('--output', help='輸出檔案路徑')
    
    # 匯入場景
    import_parser = subparsers.add_parser('import', help='匯入場景')
    import_parser.add_argument('file', help='場景檔案路徑')
    import_parser.add_argument('--name', help='場景名稱 (預設使用檔案名)')
    
    # 運行測試
    test_parser = subparsers.add_parser('test', help='使用場景運行測試')
    test_parser.add_argument('scenario', help='場景名稱')
    test_parser.add_argument('--test-file', help='特定測試檔案')
    test_parser.add_argument('-v', '--verbose', action='store_true', help='詳細輸出')
    
    # 啟動服務器
    server_parser = subparsers.add_parser('server', help='啟動模擬服務器')
    server_parser.add_argument('--host', default='localhost', help='服務器主機')
    server_parser.add_argument('--port', type=int, default=8080, help='服務器端口')
    server_parser.add_argument('--scenario', help='使用的場景')
    
    # 生成報告
    report_parser = subparsers.add_parser('report', help='生成測試環境報告')
    report_parser.add_argument('--output-dir', help='輸出目錄')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return 1
    
    manager = TestEnvironmentManager()
    
    try:
        if args.command == 'create':
            success = manager.create_scenario(args.name, args.description or "")
        elif args.command == 'list':
            scenarios = manager.list_scenarios()
            if scenarios:
                print(f"\n📋 可用場景 ({len(scenarios)} 個):")
                for scenario in scenarios:
                    print(f"  - {scenario['name']}: {scenario['description']}")
                    print(f"    AGV: {scenario['agv_count']}, 任務: {scenario['task_count']}, Rack: {scenario['rack_count']}")
            else:
                print("📋 無可用場景")
            success = True
        elif args.command == 'load':
            success = manager.load_scenario(args.name)
        elif args.command == 'validate':
            success = manager.validate_scenario(args.name)
        elif args.command == 'export':
            success = manager.export_scenario(args.name, args.output)
        elif args.command == 'import':
            success = manager.import_scenario(args.file, args.name)
        elif args.command == 'test':
            success = manager.run_test_with_scenario(args.scenario, args.test_file, args.verbose)
        elif args.command == 'server':
            success = manager.start_mock_server(args.host, args.port, args.scenario)
        elif args.command == 'report':
            success = manager.generate_test_report(args.output_dir)
        else:
            print(f"❌ 未知命令: {args.command}")
            success = False
        
        return 0 if success else 1
        
    except KeyboardInterrupt:
        print("\n⚠️ 操作被使用者中斷")
        return 1
    except Exception as e:
        print(f"❌ 執行命令時發生錯誤: {e}")
        return 1


if __name__ == "__main__":
    exit(main())