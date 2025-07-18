#!/usr/bin/env python3
"""
統一測試運行腳本

執行所有地圖匯入和清除功能的測試，提供完整的測試報告
"""

import sys
import os
import subprocess
import time
from pathlib import Path


def run_test_script(script_path, test_name):
    """運行單個測試腳本"""
    print(f"\n{'='*60}")
    print(f"🧪 執行測試: {test_name}")
    print(f"📄 腳本: {script_path}")
    print(f"{'='*60}")
    
    try:
        # 執行測試腳本
        result = subprocess.run(
            [sys.executable, script_path],
            capture_output=True,
            text=True,
            timeout=300  # 5分鐘超時
        )
        
        # 顯示輸出
        if result.stdout:
            print(result.stdout)
        
        if result.stderr:
            print("⚠️  錯誤輸出:")
            print(result.stderr)
        
        # 檢查返回碼
        if result.returncode == 0:
            print(f"✅ {test_name} 測試通過")
            return True
        else:
            print(f"❌ {test_name} 測試失敗 (返回碼: {result.returncode})")
            return False
            
    except subprocess.TimeoutExpired:
        print(f"⏰ {test_name} 測試超時")
        return False
    except Exception as e:
        print(f"❌ {test_name} 測試執行異常: {e}")
        return False


def main():
    """主測試函數"""
    print("🚀 DB Proxy SQL 模組完整測試套件")
    print("=" * 60)
    print(f"⏰ 開始時間: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    
    # 測試腳本列表
    test_scripts = [
        ("test_kuka_map_import.py", "KUKA 地圖匯入功能"),
        ("test_ct_map_import.py", "CT 地圖匯入功能"),
        ("test_both_maps.py", "地圖整合功能"),
        ("test_smart_clear.py", "智能清除功能"),
    ]
    
    # 獲取測試目錄
    test_dir = Path(__file__).parent
    
    # 執行所有測試
    results = []
    start_time = time.time()
    
    for script_name, test_name in test_scripts:
        script_path = test_dir / script_name
        
        if not script_path.exists():
            print(f"⚠️  測試腳本不存在: {script_path}")
            results.append((test_name, False))
            continue
        
        result = run_test_script(str(script_path), test_name)
        results.append((test_name, result))
        
        # 測試間隔
        time.sleep(1)
    
    end_time = time.time()
    duration = end_time - start_time
    
    # 生成測試報告
    print("\n" + "=" * 60)
    print("📋 完整測試報告")
    print("=" * 60)
    print(f"⏰ 測試時間: {duration:.2f} 秒")
    print(f"📅 完成時間: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    
    passed = 0
    total = len(results)
    
    print(f"\n📊 測試結果詳情:")
    for test_name, result in results:
        status = "✅ 通過" if result else "❌ 失敗"
        print(f"   {status} {test_name}")
        if result:
            passed += 1
    
    print(f"\n📈 總體統計:")
    print(f"   - 總測試數: {total}")
    print(f"   - 通過數量: {passed}")
    print(f"   - 失敗數量: {total - passed}")
    print(f"   - 成功率: {(passed/total*100):.1f}%")
    
    # 最終結果
    if passed == total:
        print("\n🎉 所有測試通過！")
        print("✅ DB Proxy SQL 模組功能正常")
        print("\n💡 後續步驟:")
        print("   1. 可以安全執行 db_install")
        print("   2. 地圖匯入功能已就緒")
        print("   3. 智能清除功能可用")
        return True
    else:
        print(f"\n⚠️  {total - passed} 個測試失敗")
        print("❌ 請檢查失敗的測試並修復問題")
        print("\n🔧 故障排除建議:")
        print("   1. 檢查資料庫連接")
        print("   2. 確認地圖檔案存在")
        print("   3. 查看詳細錯誤訊息")
        return False


def run_quick_test():
    """快速測試模式"""
    print("⚡ 快速測試模式")
    print("只執行基本功能驗證，跳過詳細測試")
    
    # 只執行整合測試
    test_dir = Path(__file__).parent
    script_path = test_dir / "test_both_maps.py"
    
    if script_path.exists():
        return run_test_script(str(script_path), "地圖整合功能 (快速)")
    else:
        print("❌ 快速測試腳本不存在")
        return False


if __name__ == "__main__":
    # 檢查命令行參數
    if len(sys.argv) > 1 and sys.argv[1] == "--quick":
        success = run_quick_test()
    else:
        success = main()
    
    if not success:
        sys.exit(1)
    
    print(f"\n🏁 測試完成於 {time.strftime('%Y-%m-%d %H:%M:%S')}")
