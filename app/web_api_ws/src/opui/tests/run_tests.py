#!/usr/bin/env python33
"""
OPUI 測試運行腳本
提供不同類型的測試運行選項
"""

import sys
import subprocess
import argparse
from pathlib import Path


def run_command(cmd, description):
    """運行命令並處理結果"""
    print(f"\n{'='*60}")
    print(f"執行: {description}")
    print(f"命令: {' '.join(cmd)}")
    print(f"{'='*60}")

    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print(result.stdout)
        if result.stderr:
            print("警告:", result.stderr)
        return True
    except subprocess.CalledProcessError as e:
        print(f"錯誤: {e}")
        print(f"標準輸出: {e.stdout}")
        print(f"錯誤輸出: {e.stderr}")
        return False


def run_unit_tests():
    """運行單元測試"""
    cmd = [
        "python3", "-m", "pytest",
        "test_db.py",
        "test_op_ui_socket.py",
        "test_op_ui_server.py",
        "test_routers.py",
        "-v",
        "--tb=short"
    ]
    return run_command(cmd, "單元測試")


def run_integration_tests():
    """運行整合測試"""
    cmd = [
        "python3", "-m", "pytest",
        "test_integration.py",
        "-v",
        "--tb=short"
    ]
    return run_command(cmd, "整合測試")


def run_performance_tests():
    """運行效能測試"""
    cmd = [
        "python3", "-m", "pytest",
        "test_performance.py",
        "-v",
        "--tb=short",
        "-m", "not stress"  # 排除壓力測試
    ]
    return run_command(cmd, "效能測試")


def run_stress_tests():
    """運行壓力測試"""
    cmd = [
        "python3", "-m", "pytest",
        "test_performance.py",
        "-v",
        "--tb=short",
        "-m", "stress"
    ]
    return run_command(cmd, "壓力測試")


def run_all_tests():
    """運行所有測試"""
    cmd = [
        "python3", "-m", "pytest",
        "-v",
        "--tb=short",
        "--durations=10"
    ]
    return run_command(cmd, "所有測試")


def run_coverage_tests():
    """運行測試覆蓋率分析"""
    cmd = [
        "python3", "-m", "pytest",
        "--cov=opui",
        "--cov-report=html",
        "--cov-report=term-missing",
        "--cov-report=xml",
        "-v"
    ]
    return run_command(cmd, "測試覆蓋率分析")


def run_specific_test(test_file, test_function=None):
    """運行特定測試"""
    cmd = ["python3", "-m", "pytest", test_file, "-v"]
    if test_function:
        cmd.extend(["-k", test_function])

    description = f"特定測試: {test_file}"
    if test_function:
        description += f"::{test_function}"

    return run_command(cmd, description)


def check_test_environment():
    """檢查測試環境"""
    print("檢查測試環境...")

    # 檢查 Python3 版本
    python3_version = sys.version_info
    print(f"Python3 版本: {python3_version.major}.{python3_version.minor}.{python3_version.micro}")

    if python3_version < (3, 8):
        print("警告: 建議使用 Python3 3.8 或更高版本")

    # 檢查必要的套件
    required_packages = [
        "pytest",
        "pytest-asyncio",
        "fastapi",
        "sqlmodel",
        "socketio"
    ]

    missing_packages = []
    for package in required_packages:
        try:
            __import__(package.replace("-", "_"))
            print(f"✓ {package}")
        except ImportError:
            missing_packages.append(package)
            print(f"✗ {package} (缺少)")

    if missing_packages:
        print(f"\n缺少套件: {', '.join(missing_packages)}")
        print("請運行: pip install -r test/requirements-test.txt")
        return False

    print("\n測試環境檢查完成 ✓")
    return True


def generate_test_report():
    """生成測試報告"""
    cmd = [
        "python3", "-m", "pytest",
        "--html=test_report.html",
        "--self-contained-html",
        "--json-report",
        "--json-report-file=test_report.json",
        "-v"
    ]
    return run_command(cmd, "生成測試報告")


def main():
    """主函數"""
    parser = argparse.ArgumentParser(description="OPUI 測試運行腳本")
    parser.add_argument(
        "test_type",
        choices=[
            "unit", "integration", "performance", "stress",
            "all", "coverage", "report", "check", "specific"
        ],
        help="測試類型"
    )
    parser.add_argument(
        "--file",
        help="特定測試檔案 (用於 specific 類型)"
    )
    parser.add_argument(
        "--function",
        help="特定測試函數 (用於 specific 類型)"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="詳細輸出"
    )

    args = parser.parse_args()

    # 切換到測試目錄
    test_dir = Path(__file__).parent
    original_dir = Path.cwd()

    try:
        import os
        os.chdir(test_dir)

        success = True

        if args.test_type == "check":
            success = check_test_environment()
        elif args.test_type == "unit":
            success = run_unit_tests()
        elif args.test_type == "integration":
            success = run_integration_tests()
        elif args.test_type == "performance":
            success = run_performance_tests()
        elif args.test_type == "stress":
            success = run_stress_tests()
        elif args.test_type == "all":
            success = run_all_tests()
        elif args.test_type == "coverage":
            success = run_coverage_tests()
        elif args.test_type == "report":
            success = generate_test_report()
        elif args.test_type == "specific":
            if not args.file:
                print("錯誤: 使用 specific 類型時必須指定 --file")
                sys.exit(1)
            success = run_specific_test(args.file, args.function)

        if success:
            print(f"\n✓ {args.test_type} 測試完成")
            sys.exit(0)
        else:
            print(f"\n✗ {args.test_type} 測試失敗")
            sys.exit(1)

    finally:
        os.chdir(original_dir)


if __name__ == "__main__":
    main()
