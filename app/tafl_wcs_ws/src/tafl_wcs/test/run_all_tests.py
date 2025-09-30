#!/usr/bin/env python3
"""執行所有 TAFL 業務流程測試"""

import sys
import asyncio
import subprocess
from datetime import datetime

# 測試腳本列表
TEST_SCRIPTS = [
    ("空料架停車區管理（3個流程）", "test_parking_flows.py"),
    ("射出機停車格→系統準備區", "test_machine_to_prepare.py"),
    ("完成料架出口→人工收料區", "test_full_rack_to_collection.py"),
    ("架台翻轉（入口+出口）", "test_rack_rotation.py"),
    ("房間投料調度", "test_room_dispatch.py"),
    ("重複執行防護", "test_duplicate_prevention.py"),
]

def run_test(script_name):
    """執行單個測試腳本"""
    try:
        result = subprocess.run(
            ["python3", script_name],
            capture_output=True,
            text=True,
            timeout=120
        )
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "測試超時（120秒）"
    except Exception as e:
        return False, "", str(e)

def main():
    """執行所有測試並生成報告"""
    print("=" * 80)
    print("RosAGV TAFL 業務流程完整測試套件")
    print("=" * 80)
    print(f"開始時間: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()

    results = []
    passed_count = 0
    failed_count = 0

    for idx, (test_name, script_name) in enumerate(TEST_SCRIPTS, 1):
        print(f"\n[{idx}/{len(TEST_SCRIPTS)}] 執行測試: {test_name}")
        print("-" * 80)

        success, stdout, stderr = run_test(script_name)

        if success:
            print(f"✅ {test_name} - 通過")
            passed_count += 1
            results.append((test_name, True, None))
        else:
            print(f"❌ {test_name} - 失敗")
            failed_count += 1
            results.append((test_name, False, stderr or stdout))

        # 顯示簡要輸出（最後10行）
        output_lines = (stdout or stderr).split('\n')
        if len(output_lines) > 10:
            print("\n最後輸出:")
            for line in output_lines[-10:]:
                if line.strip():
                    print(f"  {line}")

    # 總結報告
    print("\n" + "=" * 80)
    print("測試總結")
    print("=" * 80)
    print(f"總測試數: {len(TEST_SCRIPTS)}")
    print(f"✅ 通過: {passed_count}")
    print(f"❌ 失敗: {failed_count}")
    print()

    # 詳細結果
    for test_name, success, error in results:
        status = "✅ 通過" if success else "❌ 失敗"
        print(f"{status} - {test_name}")
        if error and not success:
            print(f"     錯誤: {error[:200]}...")

    print()
    print(f"完成時間: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 80)

    # 測試覆蓋的流程清單
    print("\n📋 測試覆蓋的業務流程:")
    flows = [
        "1. 空料架入口→出口（出口空閒）",
        "2. 空料架入口→停車區（出口佔用）",
        "3. 停車區→出口（房間有carrier等待）",
        "4. 射出機停車格→系統準備區（已派車料架）",
        "5. 完成料架出口→人工收料區（滿載）",
        "6. 完成料架出口→人工收料區（尾批）",
        "7. 房間入口架台翻轉（A空B工作）",
        "8. 房間出口架台翻轉（A滿B空）",
        "9. 房間投料調度（準備區→房間入口）",
        "10. 重複執行防護機制驗證",
    ]
    for flow in flows:
        print(f"  {flow}")

    print("\n🔧 測試的關鍵機制:")
    mechanisms = [
        "✓ 重複任務防護（所有流程）",
        "✓ 條件判斷邏輯（滿載、尾批、空閒檢查）",
        "✓ 產品與房間匹配（process_settings_id）",
        "✓ 派車狀態檢查（room_id != null）",
        "✓ 位置佔用狀態管理",
        "✓ 載具計數與側面判斷（A面、B面）",
    ]
    for mechanism in mechanisms:
        print(f"  {mechanism}")

    # 返回退出碼
    return 0 if failed_count == 0 else 1

if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)