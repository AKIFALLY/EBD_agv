#!/usr/bin/env python3
"""
簡單的邊更新測試
"""

import sys
import traceback
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo

# 加入專案路徑
sys.path.append(str(Path(__file__).parent.parent.parent))

from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import KukaNode, KukaEdge
from sqlmodel import select


def simple_edge_test():
    """簡單的邊更新測試"""
    print("🔍 簡單的邊更新測試...")
    
    # 資料庫連接設定
    db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    
    try:
        # 建立連接池
        pool_agvc = ConnectionPoolManager(db_url_agvc, 1)
        
        with pool_agvc.get_session() as session:
            taipei_timezone = ZoneInfo("Asia/Taipei")
            
            # 找一個現有的邊來測試 - 使用名稱 "51-50"
            edge_name = "51-50"
            existing_edge = session.exec(select(KukaEdge).where(
                KukaEdge.name == edge_name)).first()
            
            if not existing_edge:
                print(f"❌ 沒有找到邊: {edge_name}")
                return
            
            print(f"📋 找到現有邊: {existing_edge.name}")
            print(f"   ID: {existing_edge.id}")
            print(f"   From: {existing_edge.from_id}")
            print(f"   To: {existing_edge.to_id}")
            print(f"   Weight: {existing_edge.weight}")
            
            # 嘗試更新邊 - 模擬 18_kuka_map.py 中的邏輯
            try:
                print("\n🔄 嘗試更新邊...")
                now = datetime.now(taipei_timezone)
                
                # 這是 18_kuka_map.py 中的邏輯
                existing_edge.from_id = 51
                existing_edge.to_id = 50
                existing_edge.weight = 1.0
                existing_edge.updated_at = now
                
                print("   ✅ 屬性設置成功")
                
                # 嘗試提交
                session.commit()
                print("   ✅ 提交成功")
                
            except Exception as e:
                print(f"   ❌ 更新失敗: {e}")
                print(f"   錯誤類型: {type(e).__name__}")
                traceback.print_exc()
                session.rollback()
        
        # 關閉連接池
        pool_agvc.shutdown()
        
    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    simple_edge_test()
