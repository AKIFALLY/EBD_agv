#!/usr/bin/env python3
"""
智能清除功能測試腳本

測試 KUKA 和 CT 地圖的智能清除功能，驗證：
1. 外鍵約束檢測
2. 部分清除能力
3. 詳細的統計報告
4. 錯誤處理機制
"""

from db_proxy.models import Node, Edge, KukaNode, KukaEdge
from db_proxy.connection_pool_manager import ConnectionPoolManager
import sys
import os
import importlib.util
from pathlib import Path
from sqlalchemy import text
from sqlmodel import select

# 加入專案路徑
sys.path.append(str(Path(__file__).parent.parent.parent.parent))


def import_clear_functions():
    """動態匯入清除函數"""
    # 匯入 KUKA 清除函數
    kuka_module_path = os.path.join(
        os.path.dirname(__file__), '..', '..', 'init_data', '18_kuka_map.py')
    spec = importlib.util.spec_from_file_location("kuka_map", kuka_module_path)
    kuka_map_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(kuka_map_module)

    # 匯入 CT 清除函數
    ct_module_path = os.path.join(
        os.path.dirname(__file__), '..', '..', 'init_data', '19_ct_map.py')
    spec = importlib.util.spec_from_file_location("ct_map", ct_module_path)
    ct_map_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ct_map_module)

    return kuka_map_module.clear_kuka_map, ct_map_module.clear_ct_map


def test_kuka_smart_clear():
    """測試 KUKA 地圖智能清除功能"""
    print("🧪 測試 KUKA 地圖智能清除功能...")
    print("=" * 60)

    # 資料庫連接設定
    db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'

    try:
        # 匯入清除函數
        clear_kuka_map, _ = import_clear_functions()

        # 建立連接池
        pool_agvc = ConnectionPoolManager(db_url_agvc, 1)

        with pool_agvc.get_session() as session:
            # 測試前統計
            print("📊 測試前 KUKA 地圖統計:")
            kuka_nodes_before = session.exec(select(KukaNode)).all()
            kuka_edges_before = session.exec(select(KukaEdge)).all()
            print(f"   節點: {len(kuka_nodes_before)}")
            print(f"   邊: {len(kuka_edges_before)}")

            if len(kuka_nodes_before) == 0 and len(kuka_edges_before) == 0:
                print("⚠️  沒有 KUKA 地圖資料可清除")
                return True

            # 執行智能清除
            print("\n🗑️ 執行 KUKA 地圖智能清除...")
            result = clear_kuka_map(session)

            # 驗證結果
            print(f"\n✅ KUKA 清除結果驗證:")
            print(f"   節點刪除: {result['nodes_deleted']}")
            print(f"   節點跳過: {result['nodes_skipped']}")
            print(f"   邊刪除: {result['edges_deleted']}")
            print(f"   邊跳過: {result['edges_skipped']}")

            if result['skipped_reasons']:
                print(f"   跳過原因: {list(result['skipped_reasons'].keys())}")

            return True

        # 關閉連接池
        pool_agvc.shutdown()

    except Exception as e:
        print(f"❌ KUKA 智能清除測試失敗: {e}")
        return False


def test_ct_smart_clear():
    """測試 CT 地圖智能清除功能"""
    print("\n🧪 測試 CT 地圖智能清除功能...")
    print("=" * 60)

    # 資料庫連接設定
    db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'

    try:
        # 匯入清除函數
        _, clear_ct_map = import_clear_functions()

        # 建立連接池
        pool_agvc = ConnectionPoolManager(db_url_agvc, 1)

        with pool_agvc.get_session() as session:
            # 測試前統計
            print("📊 測試前 CT 地圖統計:")
            ct_nodes_before = session.exec(select(Node)).all()
            ct_edges_before = session.exec(select(Edge)).all()
            print(f"   節點: {len(ct_nodes_before)}")
            print(f"   邊: {len(ct_edges_before)}")

            if len(ct_nodes_before) == 0 and len(ct_edges_before) == 0:
                print("⚠️  沒有 CT 地圖資料可清除")
                return True

            # 執行智能清除
            print("\n🗑️ 執行 CT 地圖智能清除...")
            result = clear_ct_map(session)

            # 驗證結果
            print(f"\n✅ CT 清除結果驗證:")
            print(f"   節點刪除: {result['nodes_deleted']}")
            print(f"   節點跳過: {result['nodes_skipped']}")
            print(f"   邊刪除: {result['edges_deleted']}")
            print(f"   邊跳過: {result['edges_skipped']}")

            if result['skipped_reasons']:
                print(f"   跳過原因: {list(result['skipped_reasons'].keys())}")

            return True

        # 關閉連接池
        pool_agvc.shutdown()

    except Exception as e:
        print(f"❌ CT 智能清除測試失敗: {e}")
        return False


def test_foreign_key_detection():
    """測試外鍵約束檢測功能"""
    print("\n🧪 測試外鍵約束檢測功能...")
    print("=" * 60)

    # 資料庫連接設定
    db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'

    try:
        # 建立連接池
        pool_agvc = ConnectionPoolManager(db_url_agvc, 1)

        with pool_agvc.get_session() as session:
            # 檢查哪些節點被其他表參考
            print("🔍 檢查節點外鍵參考情況...")

            # 檢查 machine 表對 node 的參考
            machine_refs = session.exec(text("""
                SELECT DISTINCT parking_space_1, parking_space_2 
                FROM machine 
                WHERE parking_space_1 IS NOT NULL OR parking_space_2 IS NOT NULL
            """)).all()

            referenced_nodes = set()
            for ref in machine_refs:
                if ref[0]:
                    referenced_nodes.add(ref[0])
                if ref[1]:
                    referenced_nodes.add(ref[1])

            print(f"   被 machine 表參考的節點數量: {len(referenced_nodes)}")
            if referenced_nodes:
                sample_nodes = list(referenced_nodes)[:5]
                print(f"   範例節點 ID: {sample_nodes}")

            # 檢查其他可能的外鍵參考
            other_refs = session.exec(text("""
                SELECT kcu.table_name, kcu.column_name, kcu.constraint_name
                FROM information_schema.key_column_usage kcu
                JOIN information_schema.referential_constraints rc
                    ON kcu.constraint_name = rc.constraint_name
                JOIN information_schema.key_column_usage kcu2
                    ON rc.unique_constraint_name = kcu2.constraint_name
                WHERE kcu2.table_name IN ('node', 'kuka_node')
                    AND kcu.table_name != kcu2.table_name
            """)).all()

            print(f"   發現的外鍵約束數量: {len(other_refs)}")
            for ref in other_refs[:5]:  # 只顯示前5個
                print(f"   - {ref[0]}.{ref[1]} -> {ref[2]}")

            return True

        # 關閉連接池
        pool_agvc.shutdown()

    except Exception as e:
        print(f"❌ 外鍵約束檢測測試失敗: {e}")
        return False


def main():
    """主測試函數"""
    print("🚀 智能清除功能測試套件")
    print("=" * 60)

    tests = [
        ("外鍵約束檢測", test_foreign_key_detection),
        ("KUKA 地圖智能清除", test_kuka_smart_clear),
        ("CT 地圖智能清除", test_ct_smart_clear),
    ]

    results = []
    for test_name, test_func in tests:
        try:
            print(f"\n🔍 執行測試: {test_name}")
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"❌ 測試 '{test_name}' 發生異常: {e}")
            results.append((test_name, False))

    # 總結報告
    print("\n" + "=" * 60)
    print("📋 測試結果總結")
    print("=" * 60)

    passed = 0
    total = len(results)

    for test_name, result in results:
        status = "✅ 通過" if result else "❌ 失敗"
        print(f"{status} {test_name}")
        if result:
            passed += 1

    print(f"\n📊 總體結果: {passed}/{total} 測試通過")

    if passed == total:
        print("🎉 所有智能清除功能測試通過！")
        print("\n💡 使用建議:")
        print("   - 智能清除會跳過有外鍵約束的節點")
        print("   - 這是正常行為，保護資料完整性")
        print("   - 查看詳細統計了解跳過原因")
    else:
        print("⚠️  部分測試失敗，請檢查錯誤訊息")

    return passed == total


if __name__ == "__main__":
    success = main()
    if not success:
        sys.exit(1)
