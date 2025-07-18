#!/usr/bin/env python3
"""
調試 KukaEdge 更新問題
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


def debug_edge_update():
    """調試邊更新問題"""
    print("🔍 調試 KukaEdge 更新問題...")
    
    # 資料庫連接設定
    db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    
    try:
        # 建立連接池
        pool_agvc = ConnectionPoolManager(db_url_agvc, 1)
        
        with pool_agvc.get_session() as session:
            taipei_timezone = ZoneInfo("Asia/Taipei")
            
            # 找一個現有的邊來測試
            existing_edge = session.exec(select(KukaEdge)).first()
            
            if not existing_edge:
                print("❌ 沒有找到現有的邊")
                return
            
            print(f"📋 找到現有邊: {existing_edge.name}")
            print(f"   ID: {existing_edge.id}")
            print(f"   From: {existing_edge.from_id}")
            print(f"   To: {existing_edge.to_id}")
            print(f"   Weight: {existing_edge.weight}")
            print(f"   Created: {existing_edge.created_at}")
            print(f"   Updated: {existing_edge.updated_at}")
            
            # 檢查每個屬性是否可以設置
            print("\n🔍 檢查屬性設置能力:")
            
            # 測試各個屬性
            test_attributes = [
                ('from_id', 999),
                ('to_id', 998),
                ('weight', 5.5),
                ('updated_at', datetime.now(taipei_timezone))
            ]
            
            for attr_name, test_value in test_attributes:
                try:
                    print(f"   測試設置 {attr_name}...")
                    setattr(existing_edge, attr_name, test_value)
                    print(f"   ✅ {attr_name} 設置成功")
                except Exception as e:
                    print(f"   ❌ {attr_name} 設置失敗: {e}")
                    print(f"      錯誤類型: {type(e).__name__}")
                    traceback.print_exc()
            
            # 檢查模型的屬性定義
            print(f"\n🔍 檢查模型屬性定義:")
            print(f"   模型類型: {type(existing_edge)}")
            print(f"   模型字段: {existing_edge.__fields__.keys() if hasattr(existing_edge, '__fields__') else 'N/A'}")
            
            # 檢查 SQLModel 的特殊屬性
            if hasattr(existing_edge, '__table__'):
                print(f"   表名: {existing_edge.__table__.name}")
                print(f"   列: {[col.name for col in existing_edge.__table__.columns]}")
            
            # 嘗試直接使用原始的 map_importer 方式
            print(f"\n🔍 嘗試原始 map_importer 方式:")
            try:
                now = datetime.now(taipei_timezone)
                existing_edge.from_id = 1
                existing_edge.to_id = 2
                existing_edge.weight = 1.0
                existing_edge.updated_at = now
                print("   ✅ 原始方式設置成功")
                
                # 嘗試提交
                session.commit()
                print("   ✅ 提交成功")
                
            except Exception as e:
                print(f"   ❌ 原始方式失敗: {e}")
                print(f"      錯誤類型: {type(e).__name__}")
                traceback.print_exc()
                session.rollback()
        
        # 關閉連接池
        pool_agvc.shutdown()
        
    except Exception as e:
        print(f"❌ 調試失敗: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    debug_edge_update()
