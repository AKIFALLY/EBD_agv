#!/usr/bin/env python3
"""
KUKA 相關模組測試運行腳本
執行所有 KUKA 相關的單元測試並生成報告
"""
import os
import sys
import subprocess
import argparse
from pathlib import Path

def run_tests(test_files=None, verbose=False, coverage=False):
    """
    運行測試
    
    Args:
        test_files: 要運行的測試檔案列表，None 表示運行所有測試
        verbose: 是否顯示詳細輸出
        coverage: 是否生成覆蓋率報告
    """
    # 設置環境
    test_dir = Path(__file__).parent
    rcs_dir = test_dir.parent
    src_dir = rcs_dir.parent
    
    # 添加路徑到 Python 路徑
    sys.path.insert(0, str(src_dir))
    sys.path.insert(0, str(rcs_dir))
    
    # 建立 pytest 命令
    cmd = ["python", "-m", "pytest"]
    
    if verbose:
        cmd.append("-v")
    
    if coverage:
        cmd.extend([
            "--cov=rcs.kuka_manager",
            "--cov=rcs.kuka_robot", 
            "--cov=rcs.kuka_container",
            "--cov-report=html",
            "--cov-report=term-missing"
        ])
    
    # 指定測試檔案
    if test_files:
        for test_file in test_files:
            cmd.append(str(test_dir / test_file))
    else:
        # 運行所有 KUKA 測試
        cmd.extend([
            str(test_dir / "test_kuka_robot.py"),
            str(test_dir / "test_kuka_container.py"), 
            str(test_dir / "test_kuka_manager.py")
        ])
    
    print(f"執行命令: {' '.join(cmd)}")
    print("=" * 60)
    
    # 執行測試
    try:
        result = subprocess.run(cmd, cwd=str(test_dir), check=True)
        print("\n✅ 所有測試通過！")
        return True
    except subprocess.CalledProcessError as e:
        print(f"\n❌ 測試失敗，返回碼: {e.returncode}")
        return False
    except FileNotFoundError:
        print("❌ 錯誤：找不到 pytest。請確保已安裝 pytest。")
        print("安裝命令: pip install pytest pytest-cov")
        return False

def main():
    """主函數"""
    parser = argparse.ArgumentParser(description="運行 KUKA 相關模組測試")
    parser.add_argument(
        "--test-files", 
        nargs="*", 
        help="指定要運行的測試檔案（不指定則運行所有測試）"
    )
    parser.add_argument(
        "-v", "--verbose", 
        action="store_true", 
        help="顯示詳細輸出"
    )
    parser.add_argument(
        "--coverage", 
        action="store_true", 
        help="生成覆蓋率報告"
    )
    parser.add_argument(
        "--robot-only", 
        action="store_true", 
        help="只運行 KukaRobot 測試"
    )
    parser.add_argument(
        "--container-only", 
        action="store_true", 
        help="只運行 KukaContainer 測試"
    )
    parser.add_argument(
        "--manager-only", 
        action="store_true", 
        help="只運行 KukaManager 測試"
    )
    
    args = parser.parse_args()
    
    # 根據參數決定要運行的測試檔案
    test_files = args.test_files
    
    if args.robot_only:
        test_files = ["test_kuka_robot.py"]
    elif args.container_only:
        test_files = ["test_kuka_container.py"]
    elif args.manager_only:
        test_files = ["test_kuka_manager.py"]
    
    print("🚀 KUKA 模組測試套件")
    print("=" * 60)
    
    if test_files:
        print(f"測試檔案: {', '.join(test_files)}")
    else:
        print("測試檔案: 所有 KUKA 相關測試")
    
    print(f"詳細輸出: {'是' if args.verbose else '否'}")
    print(f"覆蓋率報告: {'是' if args.coverage else '否'}")
    print()
    
    # 執行測試
    success = run_tests(test_files, args.verbose, args.coverage)
    
    if args.coverage and success:
        print("\n📊 覆蓋率報告已生成：")
        print("- HTML 報告: htmlcov/index.html")
        print("- 終端報告: 已顯示在上方")
    
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()