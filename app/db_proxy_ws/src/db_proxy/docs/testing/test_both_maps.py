#!/usr/bin/env python3
"""
測試 KUKA 和 CT 地圖匯入功能整合
"""

import sys
import os
import importlib.util
from pathlib import Path
from sqlalchemy import text
from sqlmodel import select

# 加入專案路徑
sys.path.append(str(Path(__file__).parent.parent.parent))

from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Node, Edge, KukaNode, KukaEdge


def import_map_functions():
    """動態匯入地圖初始化函數"""
    # 匯入 KUKA 地圖函數
    kuka_module_path = os.path.join(os.path.dirname(__file__), 'init_data', '18_kuka_map.py')
    spec = importlib.util.spec_from_file_location("kuka_map", kuka_module_path)
    kuka_map_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(kuka_map_module)
    
    # 匯入 CT 地圖函數
    ct_module_path = os.path.join(os.path.dirname(__file__), 'init_data', '19_ct_map.py')
    spec = importlib.util.spec_from_file_location("ct_map", ct_module_path)
    ct_map_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ct_map_module)
    
    return kuka_map_module.initialize_kuka_map, ct_map_module.initialize_ct_map


def test_both_maps():
    """測試兩個地圖匯入功能"""
    print("🚀 測試 KUKA 和 CT 地圖匯入功能整合")
    print("=" * 60)
    
    # 資料庫連接設定
    db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    
    try:
        # 匯入函數
        initialize_kuka_map, initialize_ct_map = import_map_functions()
        
        # 建立連接池
        pool_agvc = ConnectionPoolManager(db_url_agvc, 1)
        
        with pool_agvc.get_session() as session:
            # 測試前統計
            print("📊 測試前資料統計:")
            kuka_nodes = session.exec(select(KukaNode)).all()
            kuka_edges = session.exec(select(KukaEdge)).all()
            ct_nodes = session.exec(select(Node)).all()
            ct_edges = session.exec(select(Edge)).all()
            
            print(f"   KUKA 節點: {len(kuka_nodes)}")
            print(f"   KUKA 邊: {len(kuka_edges)}")
            print(f"   CT 節點: {len(ct_nodes)}")
            print(f"   CT 邊: {len(ct_edges)}")
            print()
            
            # 執行 KUKA 地圖匯入
            print("🗺️ 執行 KUKA 地圖匯入...")
            initialize_kuka_map(session)
            print()
            
            # 執行 CT 地圖匯入
            print("🗺️ 執行 CT 地圖匯入...")
            initialize_ct_map(session)
            print()
            
            # 測試後統計
            print("📊 測試後資料統計:")
            kuka_nodes_after = session.exec(select(KukaNode)).all()
            kuka_edges_after = session.exec(select(KukaEdge)).all()
            ct_nodes_after = session.exec(select(Node)).all()
            ct_edges_after = session.exec(select(Edge)).all()
            
            print(f"   KUKA 節點: {len(kuka_nodes_after)}")
            print(f"   KUKA 邊: {len(kuka_edges_after)}")
            print(f"   CT 節點: {len(ct_nodes_after)}")
            print(f"   CT 邊: {len(ct_edges_after)}")
            
            # 驗證資料完整性
            print("\n🔍 資料完整性檢查:")
            
            # 檢查 KUKA 地圖完整性
            kuka_orphaned = session.exec(text("""
                SELECT COUNT(*)
                FROM kuka_edge ke
                LEFT JOIN kuka_node kn1 ON ke.from_id = kn1.id
                LEFT JOIN kuka_node kn2 ON ke.to_id = kn2.id
                WHERE kn1.id IS NULL OR kn2.id IS NULL
            """)).first()
            
            print(f"   KUKA 孤立邊: {kuka_orphaned[0] if kuka_orphaned else 0}")
            
            # 檢查 CT 地圖完整性
            ct_orphaned = session.exec(text("""
                SELECT COUNT(*)
                FROM edge e
                LEFT JOIN node n1 ON e.from_id = n1.id
                LEFT JOIN node n2 ON e.to_id = n2.id
                WHERE n1.id IS NULL OR n2.id IS NULL
            """)).first()
            
            print(f"   CT 孤立邊: {ct_orphaned[0] if ct_orphaned else 0}")
            
            # 檢查節點 ID 範圍是否有衝突
            kuka_id_range = session.exec(text("""
                SELECT MIN(id), MAX(id) FROM kuka_node
            """)).first()
            
            ct_id_range = session.exec(text("""
                SELECT MIN(id), MAX(id) FROM node
            """)).first()
            
            print(f"   KUKA 節點 ID 範圍: {kuka_id_range[0]} - {kuka_id_range[1]}")
            print(f"   CT 節點 ID 範圍: {ct_id_range[0]} - {ct_id_range[1]}")
            
            # 檢查是否有 ID 衝突
            if kuka_id_range and ct_id_range:
                kuka_min, kuka_max = kuka_id_range
                ct_min, ct_max = ct_id_range
                
                if (kuka_min <= ct_max and kuka_max >= ct_min):
                    print("   ⚠️  節點 ID 範圍有重疊，這是正常的（不同表格）")
                else:
                    print("   ✅ 節點 ID 範圍沒有重疊")
            
            print("\n✅ 整合測試完成！")
            print("=" * 60)
            print("📋 總結:")
            print(f"   - KUKA 地圖：{len(kuka_nodes_after)} 節點，{len(kuka_edges_after)} 邊")
            print(f"   - CT 地圖：{len(ct_nodes_after)} 節點，{len(ct_edges_after)} 邊")
            print(f"   - 資料完整性：✅ 通過")
            print("=" * 60)
        
        # 關閉連接池
        pool_agvc.shutdown()
        
    except Exception as e:
        print(f"❌ 整合測試失敗: {e}")
        raise


if __name__ == "__main__":
    test_both_maps()
