#!/usr/bin/env python3
"""
ROS2 OPUI 測試運行腳本
專門為 ROS2 環境設計的測試執行工具
"""

import sys
import subprocess
import argparse
from pathlib import Path


def run_colcon_test():
    """使用 colcon 運行測試"""
    print("🚀 使用 colcon 運行 OPUI 測試...")

    # 切換到工作空間根目錄
    ws_root = Path(__file__).parent.parent.parent.parent

    try:
        # 運行 colcon test
        cmd = [
            "colcon", "test",
            "--packages-select", "opui",
            "--event-handlers", "console_direct+"
        ]

        print(f"執行命令: {' '.join(cmd)}")
        print(f"工作目錄: {ws_root}")

        result = subprocess.run(
            cmd,
            cwd=ws_root,
            check=True,
            text=True
        )

        print("✅ 測試執行完成")

        # 顯示測試結果
        print("\n📊 顯示測試結果...")
        result_cmd = ["colcon", "test-result", "--all", "--verbose"]
        subprocess.run(result_cmd, cwd=ws_root)

        return True

    except subprocess.CalledProcessError as e:
        print(f"❌ 測試執行失敗: {e}")
        return False
    except FileNotFoundError:
        print("❌ 找不到 colcon 命令，請確保 ROS2 環境已正確設定")
        return False


def run_pytest_direct(test_type="all"):
    """直接使用 pytest 運行測試"""
    print(f"🧪 直接使用 pytest 運行 {test_type} 測試...")

    test_dir = Path(__file__).parent

    # 根據測試類型選擇檔案
    test_files = {
        "unit": [
            "test_db.py",
            "test_op_ui_socket.py",
            "test_op_ui_server.py",
            "test_routers.py"
        ],
        "integration": ["test_integration.py"],
        "performance": ["test_performance.py"],
        "all": []  # 空列表表示所有測試
    }

    files = test_files.get(test_type, [])

    try:
        cmd = ["python3", "-m", "pytest", "-v", "--tb=short"]

        if files:
            cmd.extend(files)

        print(f"執行命令: {' '.join(cmd)}")
        print(f"測試目錄: {test_dir}")

        result = subprocess.run(
            cmd,
            cwd=test_dir,
            check=True,
            text=True
        )

        print("✅ 測試執行完成")
        return True

    except subprocess.CalledProcessError as e:
        print(f"❌ 測試執行失敗: {e}")
        return False


def check_ros2_environment():
    """檢查 ROS2 環境"""
    print("🔍 檢查 ROS2 環境...")

    # 檢查 ROS_DISTRO
    import os
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        print(f"✅ ROS_DISTRO: {ros_distro}")
    else:
        print("⚠️  ROS_DISTRO 未設定，請執行 all_source 或 source install/setup.bash")

    # 檢查 colcon
    try:
        result = subprocess.run(
            ["colcon", "version-check"],
            capture_output=True,
            text=True,
            check=True,
            env=os.environ.copy()  # 確保使用完整環境變數
        )
        print("✅ colcon 可用")
    except subprocess.CalledProcessError as e:
        # colcon version-check 可能會失敗，但只要 colcon 存在就行
        try:
            subprocess.run(["colcon", "-h"], capture_output=True, check=True)
            print("✅ colcon 可用")
        except:
            print(f"❌ colcon 執行錯誤: {e}")
            return False
    except FileNotFoundError:
        print("❌ colcon 未安裝或不在 PATH 中")
        print(f"當前 PATH: {os.environ.get('PATH', 'N/A')}")
        return False

    # 檢查 pytest
    try:
        result = subprocess.run(
            ["python3", "-m", "pytest", "--version"],
            capture_output=True,
            text=True,
            check=True
        )
        print(f"✅ pytest 版本: {result.stdout.strip()}")
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("❌ pytest 未安裝")
        return False

    # 檢查 OPUI 套件是否已建置
    ws_root = Path(__file__).parent.parent.parent.parent
    install_dir = ws_root / "install" / "opui"

    if install_dir.exists():
        print("✅ OPUI 套件已建置")
    else:
        print("⚠️  OPUI 套件尚未建置，請先執行 colcon build --packages-select opui")

    return True


def show_test_summary():
    """顯示測試摘要"""
    print("""
📋 OPUI 測試摘要
================

測試類型:
- unit: 單元測試 (資料庫、Socket、伺服器、路由)
- integration: 整合測試 (模組間互動)
- performance: 效能測試 (並發、壓力測試)

建議執行順序:
1. 檢查環境: python ros2_test.py check
2. 運行測試: python ros2_test.py colcon
3. 查看結果: colcon test-result --all --verbose

注意事項:
- 執行前請確保已執行 all_source
- 建議使用 colcon 方式運行測試
- 測試使用 mock 資料，不需要真實資料庫連線
""")


def main():
    """主函數"""
    parser = argparse.ArgumentParser(
        description="ROS2 OPUI 測試運行工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
範例:
  python ros2_test.py check          # 檢查環境
  python ros2_test.py colcon         # 使用 colcon 運行測試
  python ros2_test.py pytest unit   # 直接運行單元測試
  python ros2_test.py summary        # 顯示測試摘要
        """
    )

    parser.add_argument(
        "command",
        choices=["check", "colcon", "pytest", "summary"],
        help="執行的命令"
    )

    parser.add_argument(
        "test_type",
        nargs="?",
        choices=["unit", "integration", "performance", "all"],
        default="all",
        help="測試類型 (僅用於 pytest 命令)"
    )

    args = parser.parse_args()

    if args.command == "check":
        success = check_ros2_environment()
    elif args.command == "colcon":
        success = run_colcon_test()
    elif args.command == "pytest":
        success = run_pytest_direct(args.test_type)
    elif args.command == "summary":
        show_test_summary()
        success = True
    else:
        parser.print_help()
        success = False

    if success:
        print("\n🎉 操作完成")
        sys.exit(0)
    else:
        print("\n💥 操作失敗")
        sys.exit(1)


if __name__ == "__main__":
    main()
