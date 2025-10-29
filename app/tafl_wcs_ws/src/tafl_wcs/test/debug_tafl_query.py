#!/usr/bin/env python3
"""調試 TAFL 流程查詢邏輯"""

import sys
sys.path.insert(0, '/app/db_proxy_ws/src')

from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Carrier, AGV, Room
from sqlmodel import select

# 測試參數
TEST_ROOM_ID = 2
TEST_CARRIER_ID = 92001
STATION_PORTS = [2011, 2012]

# 連接資料庫
db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
pool_manager = ConnectionPoolManager(db_url)

print("\n🔍 模擬 TAFL 流程查詢邏輯")
print("="*60)

# 1. 查詢房間
print("\n1️⃣ 查詢房間 (enable=1)")
with pool_manager.get_session() as session:
    rooms = session.exec(select(Room).where(Room.enable == 1)).all()
    print(f"   結果: 找到 {len(rooms)} 個房間")
    for room in rooms:
        print(f"     - Room {room.id}: enable={room.enable}")

# 2. 查詢 Loader AGV
print("\n2️⃣ 查詢 Loader AGV (model='LOADER', enable=1)")
with pool_manager.get_session() as session:
    agvs = session.exec(select(AGV).where(
        AGV.model == "LOADER",
        AGV.enable == 1
    )).all()
    print(f"   結果: 找到 {len(agvs)} 個 Loader AGV")
    for agv in agvs:
        print(f"     - AGV {agv.id}: model={agv.model}, enable={agv.enable}")

# 3. 查詢 AGV Port 1、3 的載具
print("\n3️⃣ 查詢 AGV Port 1、3 的載具 (port_in=[2101, 2103])")
with pool_manager.get_session() as session:
    port13_carriers = session.exec(select(Carrier).where(
        Carrier.port_id.in_([2101, 2103])
    )).all()
    print(f"   結果: 找到 {len(port13_carriers)} 個載具")
    if port13_carriers:
        for carrier in port13_carriers:
            print(f"     - Carrier {carrier.id}: port_id={carrier.port_id}, status_id={carrier.status_id}")
    else:
        print("     ✅ Port 1、3 為空（有空位）")

# 4. 計算空位
port13_carrier_count = len(port13_carriers)
port13_capacity = 2
required_space = 1
available_space_port13 = port13_capacity - port13_carrier_count
agv_has_space = available_space_port13 >= required_space

print(f"\n4️⃣ 計算 Port 1、3 空位")
print(f"   port13_carrier_count = {port13_carrier_count}")
print(f"   port13_capacity = {port13_capacity}")
print(f"   required_space = {required_space}")
print(f"   available_space_port13 = {available_space_port13}")
print(f"   agv_has_space = {agv_has_space}")

# 5. 如果有空位，查詢入口傳送箱載具
if agv_has_space:
    print(f"\n5️⃣ AGV 有空位，查詢入口傳送箱 Station 01 載具")
    print(f"   查詢條件: room_id={TEST_ROOM_ID}, port_in={STATION_PORTS}, status_id=201")

    with pool_manager.get_session() as session:
        ready_carriers = session.exec(select(Carrier).where(
            Carrier.room_id == TEST_ROOM_ID,
            Carrier.port_id.in_(STATION_PORTS),
            Carrier.status_id == 201
        )).all()

        print(f"   結果: 找到 {len(ready_carriers)} 個載具")
        if ready_carriers:
            for carrier in ready_carriers:
                print(f"     - Carrier {carrier.id}: room_id={carrier.room_id}, port_id={carrier.port_id}, status_id={carrier.status_id}")

        # 檢查載具數量
        carrier_count = len(ready_carriers)
        required_count = 1
        has_enough_carriers = carrier_count >= required_count

        print(f"\n6️⃣ 檢查載具數量")
        print(f"   carrier_count = {carrier_count}")
        print(f"   required_count = {required_count}")
        print(f"   has_enough_carriers = {has_enough_carriers}")

        if has_enough_carriers:
            print("\n✅ 條件滿足，應該創建任務！")
            print(f"   - AGV 有空位: {agv_has_space}")
            print(f"   - 入口箱有載具: {has_enough_carriers}")
        else:
            print("\n❌ 載具數量不足，不創建任務")
else:
    print("\n❌ AGV 沒有空位，不執行後續查詢")

print("\n" + "="*60)
