#!/usr/bin/env python3
"""
CT 地圖匯入功能測試腳本

這個腳本用於測試和驗證 CT 地圖匯入功能是否正常運作
"""

import os
import importlib.util
from db_proxy.models import Node, Edge
from db_proxy.connection_pool_manager import ConnectionPoolManager
import sys
import os
import importlib.util
from pathlib import Path
from sqlalchemy import text
from sqlmodel import select

# 加入專案路徑
sys.path.append(str(Path(__file__).parent.parent.parent))

# 動態匯入 CT 地圖初始化函數


def import_ct_map_function():
    """動態匯入 CT 地圖初始化函數"""
    module_path = os.path.join(os.path.dirname(__file__), 'init_data', '19_ct_map.py')
    spec = importlib.util.spec_from_file_location("ct_map", module_path)
    ct_map_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ct_map_module)
    return ct_map_module.initialize_ct_map


initialize_ct_map = import_ct_map_function()


def test_ct_map_import():
    """測試 CT 地圖匯入功能"""
    print("🧪 開始測試 CT 地圖匯入功能...")
    print("=" * 60)

    # 資料庫連接設定
    db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'

    try:
        # 建立連接池
        pool_agvc = ConnectionPoolManager(db_url_agvc, 1)

        with pool_agvc.get_session() as session:
            # 測試前先檢查現有資料
            print("📊 測試前資料統計:")
            existing_nodes = session.exec(select(Node)).all()
            existing_edges = session.exec(select(Edge)).all()
            print(f"   現有節點數量: {len(existing_nodes)}")
            print(f"   現有邊數量: {len(existing_edges)}")
            print()

            # 執行地圖匯入
            initialize_ct_map(session)

            # 測試後檢查資料
            print("\n📊 測試後資料統計:")
            new_nodes = session.exec(select(Node)).all()
            new_edges = session.exec(select(Edge)).all()
            print(f"   節點數量: {len(new_nodes)}")
            print(f"   邊數量: {len(new_edges)}")

            # 顯示一些範例資料
            if new_nodes:
                print("\n📋 節點範例資料 (前5個):")
                for i, node in enumerate(new_nodes[:5]):
                    print(f"   {i+1}. ID: {node.id}, Name: {node.name}, "
                          f"X: {node.x:.2f}, Y: {node.y:.2f}")

            if new_edges:
                print("\n📋 邊範例資料 (前5個):")
                for i, edge in enumerate(new_edges[:5]):
                    print(f"   {i+1}. {edge.name}, From: {edge.from_id}, "
                          f"To: {edge.to_id}, Weight: {edge.weight}")

            # 驗證資料完整性
            print("\n🔍 資料完整性檢查:")

            # 檢查是否有孤立的邊（指向不存在的節點）
            orphaned_edges = session.exec(text("""
                SELECT e.name, e.from_id, e.to_id
                FROM edge e
                LEFT JOIN node n1 ON e.from_id = n1.id
                LEFT JOIN node n2 ON e.to_id = n2.id
                WHERE n1.id IS NULL OR n2.id IS NULL
            """)).all()

            if orphaned_edges:
                print(f"   ⚠️  發現 {len(orphaned_edges)} 個孤立的邊:")
                for edge in orphaned_edges[:3]:  # 只顯示前3個
                    print(f"      - {edge[0]}: {edge[1]} -> {edge[2]}")
            else:
                print("   ✅ 沒有發現孤立的邊")

            # 檢查節點 ID 範圍
            node_id_stats = session.exec(text("""
                SELECT MIN(id) as min_id, MAX(id) as max_id, COUNT(*) as count
                FROM node
            """)).first()

            if node_id_stats:
                print(
                    f"   📊 節點 ID 範圍: {node_id_stats[0]} - {node_id_stats[1]} (共 {node_id_stats[2]} 個)")

            # 檢查邊的連接統計
            edge_stats = session.exec(text("""
                SELECT COUNT(DISTINCT from_id) as unique_from,
                       COUNT(DISTINCT to_id) as unique_to,
                       COUNT(*) as total_edges
                FROM edge
            """)).first()

            if edge_stats:
                print(f"   📊 邊統計: {edge_stats[2]} 條邊，連接 {edge_stats[0]} 個起點到 {edge_stats[1]} 個終點")

            print("\n✅ 測試完成！")

        # 關閉連接池
        pool_agvc.shutdown()

    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        raise


def test_clear_ct_map():
    """測試清除 CT 地圖資料功能"""
    print("🧪 測試清除 CT 地圖資料功能...")

    # 資料庫連接設定
    db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'

    try:
        # 建立連接池
        pool_agvc = ConnectionPoolManager(db_url_agvc, 1)

        with pool_agvc.get_session() as session:
            # 動態匯入清除函數
            def import_clear_function():
                module_path = os.path.join(os.path.dirname(__file__), 'init_data', '19_ct_map.py')
                spec = importlib.util.spec_from_file_location("ct_map", module_path)
                ct_map_module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(ct_map_module)
                return ct_map_module.clear_ct_map

            clear_ct_map = import_clear_function()

            # 清除資料
            clear_ct_map(session)

            # 驗證清除結果
            remaining_nodes = session.exec(select(Node)).all()
            remaining_edges = session.exec(select(Edge)).all()

            print(f"清除後剩餘節點: {len(remaining_nodes)}")
            print(f"清除後剩餘邊: {len(remaining_edges)}")

            if len(remaining_nodes) == 0 and len(remaining_edges) == 0:
                print("✅ 清除功能正常")
            else:
                print("⚠️  清除可能不完整")

        # 關閉連接池
        pool_agvc.shutdown()

    except Exception as e:
        print(f"❌ 清除測試失敗: {e}")


def main():
    """主函數"""
    print("🚀 CT 地圖匯入功能測試套件")
    print("=" * 60)

    try:
        # 測試匯入功能
        test_ct_map_import()

        # 詢問是否要測試清除功能
        print("\n" + "=" * 60)
        response = input("是否要測試清除功能？(y/N): ").strip().lower()
        if response in ['y', 'yes']:
            test_clear_ct_map()

        print("\n🎉 所有測試完成！")

    except KeyboardInterrupt:
        print("\n⏹️  測試被使用者中斷")
    except Exception as e:
        print(f"\n❌ 測試過程中發生錯誤: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
