#!/usr/bin/env python3
"""
檢查資料庫中的 client 記錄
"""
import sys
sys.path.append('/app/db_proxy_ws/src/db_proxy')

from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Client
from sqlmodel import select

# 資料庫連線
db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc?client_encoding=utf8'
connection_pool = ConnectionPoolManager(db_url)

def check_clients():
    """檢查 client 記錄"""
    with connection_pool.get_session() as session:
        # 查詢所有 client 記錄
        statement = select(Client).order_by(Client.created_at.desc()).limit(20)
        clients = session.exec(statement).all()
        
        print(f"📊 資料庫中共有 {len(clients)} 個 client 記錄（最新 20 筆）:")
        print("-" * 100)
        print(f"{'ID':<30} {'Machine ID':<12} {'Created At':<25} {'Updated At':<25}")
        print("-" * 100)
        
        for client in clients:
            created_str = client.created_at.strftime('%Y-%m-%d %H:%M:%S') if client.created_at else 'None'
            updated_str = client.updated_at.strftime('%Y-%m-%d %H:%M:%S') if client.updated_at else 'None'
            print(f"{client.id:<30} {client.machine_id:<12} {created_str:<25} {updated_str:<25}")
        
        # 檢查是否有重複的 clientId 模式
        print("\n🔍 檢查 clientId 模式:")
        opui_clients = [c for c in clients if c.id and c.id.startswith('opui_')]
        if opui_clients:
            print(f"找到 {len(opui_clients)} 個 OPUI 客戶端記錄:")
            for client in opui_clients:
                print(f"  - {client.id} (created: {client.created_at})")
        else:
            print("沒有找到 OPUI 客戶端記錄")
        
        # 檢查是否有 sid 格式的記錄
        sid_clients = [c for c in clients if c.id and len(c.id) == 20 and not c.id.startswith('opui_')]
        if sid_clients:
            print(f"\n⚠️ 找到 {len(sid_clients)} 個可能是 SID 的記錄:")
            for client in sid_clients:
                print(f"  - {client.id} (created: {client.created_at})")

if __name__ == "__main__":
    check_clients()
