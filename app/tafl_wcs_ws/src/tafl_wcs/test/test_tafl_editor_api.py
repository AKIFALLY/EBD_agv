#!/usr/bin/env python3
"""
TAFL Editor API 執行測試

測試 8 個核心業務流程在 TAFL Editor API 上的執行結果
驗證 API 回應格式和執行正確性
"""

import urllib.request
import urllib.error
import json
import yaml
import time
from pathlib import Path
from datetime import datetime

# 測試配置
API_BASE_URL = "http://localhost:8001"
TAFL_EXECUTE_URL = f"{API_BASE_URL}/tafl/execute"
FLOWS_DIR = Path("/app/config/tafl/flows")

# 資料庫配置
DATABASE_URL = "postgresql://agvc:password@192.168.100.254:5432/agvc"

# 測試流程列表
TEST_FLOWS = [
    {
        "file": "empty_rack_inlet_to_outlet.yaml",
        "name": "空料架入口→出口",
        "test_data": {
            "rack_id": 701,
            "location_id": 21,  # 房間入口
            "outlet_location_id": 22  # 房間出口
        }
    },
    {
        "file": "empty_rack_inlet_to_parking.yaml",
        "name": "空料架入口→停車區",
        "test_data": {
            "rack_id": 702,
            "location_id": 21,
            "parking_location_id": 31
        }
    },
    {
        "file": "parking_to_outlet.yaml",
        "name": "停車區→出口",
        "test_data": {
            "rack_id": 703,
            "location_id": 31,
            "outlet_location_id": 22
        }
    },
    {
        "file": "machine_to_prepare.yaml",
        "name": "射出機停車格→準備區",
        "test_data": {
            "rack_id": 704,
            "location_id": 1,  # 停車格
            "prepare_location_id": 11,  # 準備區
            "room_id": 2
        }
    },
    {
        "file": "full_rack_outlet_to_manual_collection.yaml",
        "name": "完成料架→收料區",
        "test_data": {
            "rack_id": 705,
            "location_id": 22,  # 房間出口
            "collection_location_id": 51  # 收料區
        }
    },
    {
        "file": "rack_rotation_room_inlet_aempty_bwork.yaml",
        "name": "房間入口翻轉（A空B工作）",
        "test_data": {
            "rack_id": 801,
            "location_id": 21,  # 房間入口
            "room_id": 2
        }
    },
    {
        "file": "rack_rotation_room_outlet_afull_bempty.yaml",
        "name": "房間出口翻轉（A滿B空）",
        "test_data": {
            "rack_id": 802,
            "location_id": 22,  # 房間出口
            "room_id": 2
        }
    },
    {
        "file": "room_dispatch_simple.yaml",
        "name": "房間投料調度",
        "test_data": {
            "rack_id": 901,
            "location_id": 11,  # 準備區
            "room_id": 2,
            "product_id": 1
        }
    }
]


class Colors:
    """終端顏色"""
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'


def print_header(text):
    """打印標題"""
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'=' * 80}{Colors.ENDC}")
    print(f"{Colors.BOLD}{Colors.BLUE}{text:^80}{Colors.ENDC}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'=' * 80}{Colors.ENDC}\n")


def print_success(text):
    """打印成功訊息"""
    print(f"{Colors.GREEN}✓ {text}{Colors.ENDC}")


def print_error(text):
    """打印錯誤訊息"""
    print(f"{Colors.RED}✗ {text}{Colors.ENDC}")


def print_warning(text):
    """打印警告訊息"""
    print(f"{Colors.YELLOW}⚠ {text}{Colors.ENDC}")


def print_info(text):
    """打印資訊"""
    print(f"{Colors.BLUE}ℹ {text}{Colors.ENDC}")


def load_flow_yaml(flow_file):
    """載入流程 YAML 檔案"""
    flow_path = FLOWS_DIR / flow_file
    if not flow_path.exists():
        raise FileNotFoundError(f"Flow file not found: {flow_path}")

    with open(flow_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def execute_flow_via_api(flow_data, mode="simulation"):
    """透過 API 執行流程

    Args:
        flow_data: 完整的 TAFL 流程數據（包含 metadata, flow 等）
        mode: "simulation" 或 "real"

    Returns:
        API 回應的 JSON 數據
    """
    try:
        # 準備請求數據
        request_data = {
            "metadata": flow_data.get("metadata", {}),
            "settings": flow_data.get("settings", {}),
            "preload": flow_data.get("preload", {}),
            "rules": flow_data.get("rules", {}),
            "variables": flow_data.get("variables", {}),
            "flow": flow_data.get("flow", []),
            "mode": mode
        }

        # 發送請求
        print_info(f"發送請求到 {TAFL_EXECUTE_URL}")
        print_info(f"模式: {mode}")

        # 使用 urllib 發送 POST 請求
        json_data = json.dumps(request_data).encode('utf-8')
        req = urllib.request.Request(
            TAFL_EXECUTE_URL,
            data=json_data,
            headers={'Content-Type': 'application/json'}
        )

        with urllib.request.urlopen(req, timeout=30) as response:
            response_data = response.read().decode('utf-8')
            result = json.loads(response_data)
            return result

    except urllib.error.HTTPError as e:
        return {
            "success": False,
            "message": f"HTTP {e.code}: {e.reason}",
            "http_error": True
        }
    except urllib.error.URLError as e:
        return {
            "success": False,
            "message": "無法連接到 API 服務器。請確認 AGVCUI 服務運行中。",
            "connection_error": True
        }
    except TimeoutError:
        return {
            "success": False,
            "message": "API 請求超時",
            "timeout_error": True
        }
    except Exception as e:
        return {
            "success": False,
            "message": f"執行失敗: {str(e)}",
            "exception": str(e)
        }


def verify_response_format(response, mode):
    """驗證 API 回應格式"""
    errors = []

    # 基本欄位檢查
    if "success" not in response:
        errors.append("缺少 'success' 欄位")

    if "message" not in response and "execution_log" not in response:
        errors.append("缺少 'message' 或 'execution_log' 欄位")

    # 模擬模式特定檢查
    if mode == "simulation":
        if response.get("success") and "execution_log" not in response:
            errors.append("模擬模式應返回 'execution_log'")

    # 真實模式特定檢查
    if mode == "real":
        if response.get("success") and "result" not in response:
            # 真實模式可能沒有result欄位（取決於流程是否創建任務）
            pass

    return errors


def test_flow_simulation(flow_config):
    """測試流程（模擬模式）"""
    flow_name = flow_config["name"]
    flow_file = flow_config["file"]

    print(f"\n{Colors.BOLD}測試: {flow_name}{Colors.ENDC}")
    print(f"檔案: {flow_file}")
    print(f"模式: 模擬執行")

    try:
        # 載入流程
        flow_data = load_flow_yaml(flow_file)

        # 執行模擬
        result = execute_flow_via_api(flow_data, mode="simulation")

        # 檢查連接錯誤
        if result.get("connection_error"):
            print_error(result["message"])
            print_warning("請確認 AGVCUI 服務運行中：")
            print(f"  curl http://localhost:8001/health")
            return False

        # 驗證回應格式
        format_errors = verify_response_format(result, "simulation")
        if format_errors:
            print_error("API 回應格式錯誤:")
            for error in format_errors:
                print(f"  - {error}")
            return False

        # 檢查執行結果
        if result.get("success"):
            print_success("模擬執行成功")

            # 顯示執行日誌
            if "execution_log" in result:
                log = result["execution_log"]
                print_info(f"執行步驟: {len(log)} 步")
                for step in log[:3]:  # 只顯示前3步
                    verb = step.get("verb", "unknown")
                    action = step.get("action", "")
                    print(f"  步驟 {step.get('step')}: {verb} - {action}")
                if len(log) > 3:
                    print(f"  ... 還有 {len(log) - 3} 步")

            return True
        else:
            print_error(f"模擬執行失敗: {result.get('message', '未知錯誤')}")
            return False

    except FileNotFoundError as e:
        print_error(str(e))
        return False
    except Exception as e:
        print_error(f"測試失敗: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def test_all_flows():
    """測試所有流程"""
    print_header("TAFL Editor API 執行測試")

    print_info(f"測試時間: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print_info(f"API 端點: {TAFL_EXECUTE_URL}")
    print_info(f"流程目錄: {FLOWS_DIR}")
    print_info(f"測試流程數量: {len(TEST_FLOWS)}")

    # 檢查 API 可用性
    print("\n" + "=" * 80)
    print("檢查 API 服務...")
    try:
        req = urllib.request.Request(f"{API_BASE_URL}/health")
        with urllib.request.urlopen(req, timeout=5) as response:
            health_data = json.loads(response.read().decode('utf-8'))
            if health_data.get("status") == "healthy":
                print_success("API 服務運行正常")
            else:
                print_warning(f"API 健康檢查返回: {health_data}")
    except:
        print_error("無法連接到 API 服務")
        print_warning("請啟動 AGVCUI 服務:")
        print(f"  docker compose -f docker-compose.agvc.yml exec agvc_server bash")
        print(f"  python3 /app/web_api_ws/src/agvcui/agvcui/agvc_ui_server.py")
        return

    # 執行測試
    results = {
        "total": len(TEST_FLOWS),
        "passed": 0,
        "failed": 0,
        "details": []
    }

    for i, flow_config in enumerate(TEST_FLOWS, 1):
        print(f"\n{Colors.BOLD}[{i}/{len(TEST_FLOWS)}] {Colors.ENDC}", end="")

        # 測試模擬模式
        success = test_flow_simulation(flow_config)

        if success:
            results["passed"] += 1
            status = "✓ PASSED"
        else:
            results["failed"] += 1
            status = "✗ FAILED"

        results["details"].append({
            "name": flow_config["name"],
            "file": flow_config["file"],
            "status": status,
            "success": success
        })

        # 延遲避免過快請求
        time.sleep(0.5)

    # 打印結果摘要
    print_header("測試結果摘要")

    print(f"總測試數: {results['total']}")
    print_success(f"通過: {results['passed']}")
    if results['failed'] > 0:
        print_error(f"失敗: {results['failed']}")
    else:
        print_success(f"失敗: {results['failed']}")

    print(f"\n{Colors.BOLD}詳細結果:{Colors.ENDC}")
    for detail in results["details"]:
        status_color = Colors.GREEN if detail["success"] else Colors.RED
        print(f"  {status_color}{detail['status']}{Colors.ENDC} {detail['name']}")
        print(f"         檔案: {detail['file']}")

    # 成功率
    success_rate = (results['passed'] / results['total']) * 100 if results['total'] > 0 else 0
    print(f"\n{Colors.BOLD}成功率: {success_rate:.1f}%{Colors.ENDC}")

    if results['passed'] == results['total']:
        print_success("\n所有測試通過！✨")
    else:
        print_warning(f"\n{results['failed']} 個測試失敗")


if __name__ == "__main__":
    test_all_flows()