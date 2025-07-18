#!/usr/bin/env python3
"""
KUKA 地圖匯入功能測試腳本

這個腳本用於測試和驗證 KUKA 地圖匯入功能是否正常運作
"""

import sys
from pathlib import Path
from sqlalchemy import text
from sqlmodel import select

# 加入專案路徑
sys.path.append(str(Path(__file__).parent.parent.parent))

from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import KukaNode, KukaEdge
from db_proxy.sql.init_data.init_manager import initialize_kuka_map


def test_kuka_map_import():
    """測試 KUKA 地圖匯入功能"""
    print("🧪 開始測試 KUKA 地圖匯入功能...")
    print("=" * 60)
    
    # 資料庫連接設定
    db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    
    try:
        # 建立連接池
        pool_agvc = ConnectionPoolManager(db_url_agvc, 1)
        
        with pool_agvc.get_session() as session:
            # 測試前先檢查現有資料
            print("📊 測試前資料統計:")
            existing_nodes = session.exec(select(KukaNode)).all()
            existing_edges = session.exec(select(KukaEdge)).all()
            print(f"   現有節點數量: {len(existing_nodes)}")
            print(f"   現有邊數量: {len(existing_edges)}")
            print()
            
            # 執行地圖匯入
            initialize_kuka_map(session)
            
            # 測試後檢查資料
            print("\n📊 測試後資料統計:")
            new_nodes = session.exec(select(KukaNode)).all()
            new_edges = session.exec(select(KukaEdge)).all()
            print(f"   節點數量: {len(new_nodes)}")
            print(f"   邊數量: {len(new_edges)}")
            
            # 顯示一些範例資料
            if new_nodes:
                print("\n📋 節點範例資料 (前5個):")
                for i, node in enumerate(new_nodes[:5]):
                    print(f"   {i+1}. ID: {node.id}, UUID: {node.uuid}, "
                          f"Type: {node.node_type_id}, X: {node.x:.2f}, Y: {node.y:.2f}")
            
            if new_edges:
                print("\n📋 邊範例資料 (前5個):")
                for i, edge in enumerate(new_edges[:5]):
                    print(f"   {i+1}. {edge.name}, From: {edge.from_id}, "
                          f"To: {edge.to_id}, Weight: {edge.weight}")
            
            # 驗證資料完整性
            print("\n🔍 資料完整性檢查:")
            
            # 檢查是否有孤立的邊（指向不存在的節點）
            orphaned_edges = session.exec(text("""
                SELECT ke.name, ke.from_id, ke.to_id
                FROM kuka_edge ke
                LEFT JOIN kuka_node kn1 ON ke.from_id = kn1.id
                LEFT JOIN kuka_node kn2 ON ke.to_id = kn2.id
                WHERE kn1.id IS NULL OR kn2.id IS NULL
            """)).all()
            
            if orphaned_edges:
                print(f"   ⚠️  發現 {len(orphaned_edges)} 個孤立的邊:")
                for edge in orphaned_edges[:3]:  # 只顯示前3個
                    print(f"      - {edge[0]}: {edge[1]} -> {edge[2]}")
            else:
                print("   ✅ 沒有發現孤立的邊")
            
            # 檢查節點類型分佈
            node_type_stats = session.exec(text("""
                SELECT node_type_id, COUNT(*) as count
                FROM kuka_node
                WHERE node_type_id IS NOT NULL
                GROUP BY node_type_id
                ORDER BY node_type_id
            """)).all()
            
            if node_type_stats:
                print("   📊 節點類型分佈:")
                for stat in node_type_stats:
                    print(f"      - 類型 {stat[0]}: {stat[1]} 個節點")
            
            print("\n✅ 測試完成！")
        
        # 關閉連接池
        pool_agvc.shutdown()
        
    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        raise


def test_clear_kuka_map():
    """測試清除 KUKA 地圖資料功能"""
    print("🧪 測試清除 KUKA 地圖資料功能...")
    
    # 資料庫連接設定
    db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    
    try:
        # 建立連接池
        pool_agvc = ConnectionPoolManager(db_url_agvc, 1)
        
        with pool_agvc.get_session() as session:
            from db_proxy.sql.init_data.init_data.kuka_map import clear_kuka_map
            
            # 清除資料
            clear_kuka_map(session)
            
            # 驗證清除結果
            remaining_nodes = session.exec(select(KukaNode)).all()
            remaining_edges = session.exec(select(KukaEdge)).all()
            
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
    print("🚀 KUKA 地圖匯入功能測試套件")
    print("=" * 60)
    
    try:
        # 測試匯入功能
        test_kuka_map_import()
        
        # 詢問是否要測試清除功能
        print("\n" + "=" * 60)
        response = input("是否要測試清除功能？(y/N): ").strip().lower()
        if response in ['y', 'yes']:
            test_clear_kuka_map()
        
        print("\n🎉 所有測試完成！")
        
    except KeyboardInterrupt:
        print("\n⏹️  測試被使用者中斷")
    except Exception as e:
        print(f"\n❌ 測試過程中發生錯誤: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
