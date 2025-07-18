#!/usr/bin/env python3
"""
測試 KukaEdge 屬性設置修復
"""

import sys
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo

# 加入專案路徑
sys.path.append(str(Path(__file__).parent.parent.parent))

from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import KukaNode, KukaEdge
from sqlmodel import select


def test_edge_creation():
    """測試邊的建立和更新"""
    print("🧪 測試 KukaEdge 屬性設置修復...")
    
    # 資料庫連接設定
    db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    
    try:
        # 建立連接池
        pool_agvc = ConnectionPoolManager(db_url_agvc, 1)
        
        with pool_agvc.get_session() as session:
            taipei_timezone = ZoneInfo("Asia/Taipei")
            
            # 清除測試資料
            print("🗑️ 清除現有測試資料...")
            session.exec(select(KukaEdge).where(KukaEdge.name.like("test-%"))).all()
            for edge in session.exec(select(KukaEdge).where(KukaEdge.name.like("test-%"))).all():
                session.delete(edge)
            session.commit()
            
            # 確保有測試節點
            test_node_1 = session.get(KukaNode, 1)
            test_node_2 = session.get(KukaNode, 2)
            
            if not test_node_1:
                test_node_1 = KukaNode(id=1, x=0.0, y=0.0)
                test_node_1.updated_at = datetime.now(taipei_timezone)
                session.add(test_node_1)
            
            if not test_node_2:
                test_node_2 = KukaNode(id=2, x=10.0, y=10.0)
                test_node_2.updated_at = datetime.now(taipei_timezone)
                session.add(test_node_2)
            
            session.commit()
            
            # 測試建立新邊
            print("✅ 測試建立新邊...")
            edge_name = "test-1-2"
            
            try:
                new_edge = KukaEdge(
                    from_id=1,
                    to_id=2,
                    weight=1.5,
                    name=edge_name
                )
                # 手動設置 updated_at
                new_edge.updated_at = datetime.now(taipei_timezone)
                session.add(new_edge)
                session.commit()
                print("   ✅ 新邊建立成功")
                
            except Exception as e:
                print(f"   ❌ 新邊建立失敗: {e}")
                session.rollback()
                return False
            
            # 測試更新現有邊
            print("✅ 測試更新現有邊...")
            
            try:
                existing_edge = session.exec(select(KukaEdge).where(
                    KukaEdge.name == edge_name)).first()
                
                if existing_edge:
                    # 更新現有邊 - 只更新可變的欄位
                    existing_edge.from_id = 2
                    existing_edge.to_id = 1
                    existing_edge.weight = 2.0
                    existing_edge.updated_at = datetime.now(taipei_timezone)
                    session.commit()
                    print("   ✅ 現有邊更新成功")
                else:
                    print("   ⚠️  找不到測試邊")
                    
            except Exception as e:
                print(f"   ❌ 現有邊更新失敗: {e}")
                session.rollback()
                return False
            
            # 驗證結果
            print("🔍 驗證結果...")
            final_edge = session.exec(select(KukaEdge).where(
                KukaEdge.name == edge_name)).first()
            
            if final_edge:
                print(f"   邊名稱: {final_edge.name}")
                print(f"   從節點: {final_edge.from_id}")
                print(f"   到節點: {final_edge.to_id}")
                print(f"   權重: {final_edge.weight}")
                print(f"   建立時間: {final_edge.created_at}")
                print(f"   更新時間: {final_edge.updated_at}")
                print("   ✅ 驗證通過")
            else:
                print("   ❌ 找不到測試邊")
                return False
            
            # 清除測試資料
            print("🗑️ 清除測試資料...")
            session.delete(final_edge)
            session.commit()
            
            print("✅ 所有測試通過！")
            return True
        
        # 關閉連接池
        pool_agvc.shutdown()
        
    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        return False


if __name__ == "__main__":
    success = test_edge_creation()
    if success:
        print("🎉 修復驗證成功！")
    else:
        print("💥 修復驗證失敗！")
        sys.exit(1)
