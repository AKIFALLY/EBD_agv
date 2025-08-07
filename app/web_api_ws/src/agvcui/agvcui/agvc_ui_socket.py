import asyncio
import time
from datetime import datetime, timedelta, timezone
from fastapi.encoders import jsonable_encoder
from agvcui.db import node_all, edge_all, kuka_node_all, kuka_edge_all
from agvcui.db import signal_all, carrier_all, rack_all, task_all
from agvcui.db import machine_all, room_all
from agvcui.db import get_all_agvs, get_all_locations
from agvcui.db import modify_log_all_objects


class AgvcUiSocket:
    def __init__(self, sio):
        self.sio = sio
        self.connected_sids = set()  # 🆕 存目前連線中的 sid
        self.init_socketio()

        # 每項通知更新步率不同
        now = time.perf_counter()
        self.tasks = [
            {"func": self.notify_agvs,     "interval": 5, "last_time": now},
            {"func": self.notify_carriers, "interval": 10, "last_time": now},
            {"func": self.notify_signals,  "interval": 10, "last_time": now},
            {"func": self.notify_racks,    "interval": 10, "last_time": now},
            {"func": self.notify_tasks,    "interval": 15, "last_time": now},
            {"func": self.notify_by_modifylog, "interval": 0.1, "last_time": now},
        ]

        self._task = asyncio.create_task(self._periodic_notify())

    def init_socketio(self):
        self.sio.on('connect')(self.connect)
        self.sio.on('disconnect')(self.disconnect)
        self.sio.on('user_login')(self.user_login)
        self.sio.on('user_logout')(self.user_logout)
        
        # Flow Designer 事件
        self.sio.on('flow_save')(self.flow_save)
        self.sio.on('flow_load')(self.flow_load)
        self.sio.on('flow_validate')(self.flow_validate)

        # 如果還有其他事件，這裡可以繼續綁定

    async def connect(self, sid, environ):
        self.connected_sids.add(sid)  # 🆕 加入連線清單
        print("🔌 使用者連線:", sid)
        # print("🔌 使用者連線environ:", environ)
        await self.notify_map(sid)  # first create map node edge
        await self.notify_locations(sid)  # 新增 locations 資料傳送
        await self.notify_machines(sid)
        await self.notify_rooms(sid)
        # 初次連線傳送一次必要資訊
        await self.notify_agvs(sid)
        await self.notify_carriers(sid)
        await self.notify_signals(sid)
        await self.notify_racks(sid)  # 依賴 map, agv 及 carrier 的資訊 最後載入
        await self.notify_tasks(sid)
        # await self.notify_client_data(sid)

    async def disconnect(self, sid):
        self.connected_sids.discard(sid)  # 🆕 移除 sid
        print("❌ 使用者離線:", sid)

    async def user_login(self, sid, data):
        """處理用戶登入"""
        from agvcui.auth import authenticate_user
        from fastapi.encoders import jsonable_encoder

        username = data.get('username')
        password = data.get('password')

        if not username or not password:
            return {
                "success": False,
                "message": "用戶名和密碼不能為空"
            }

        # 驗證用戶
        user = authenticate_user(username, password)
        if not user:
            return {
                "success": False,
                "message": "用戶名或密碼錯誤"
            }

        # 更新最後登入時間
        from agvcui.db import update_user_last_login
        update_user_last_login(user.id)

        # 創建 JWT token
        from agvcui.auth import create_access_token
        from datetime import timedelta

        access_token_expires = timedelta(days=7)  # 7天有效期
        access_token = create_access_token(
            data={"sub": user.username}, expires_delta=access_token_expires
        )

        # 登入成功，回傳用戶資訊
        user_data = {
            "id": user.id,
            "username": user.username,
            "role": user.role,
            "full_name": user.full_name,
            "is_active": user.is_active,
            "isLoggedIn": True,
            "isConnected": True
        }

        print(f"✅ Socket 登入成功: {user.username} (sid: {sid})")

        return {
            "success": True,
            "message": f"登入成功，歡迎 {user.full_name or user.username}",
            "user": jsonable_encoder(user_data),
            "access_token": access_token
        }

    async def user_logout(self, sid, data):
        """處理用戶登出"""
        print(f"🚪 Socket 登出請求 (sid: {sid})")

        return {
            "success": True,
            "message": "登出成功"
        }

    async def flow_save(self, sid, data):
        """處理流程保存事件"""
        print(f"💾 Flow Designer 保存請求 (sid: {sid}): {data.get('name', 'Unknown')}")
        
        # 這裡可以添加保存邏輯，或者只是廣播事件
        await self.sio.emit('flow_saved', {
            'name': data.get('name', 'Unknown'),
            'timestamp': time.time()
        }, room=sid)
        
        return {
            "success": True,
            "message": f"流程 '{data.get('name', 'Unknown')}' 已保存"
        }

    async def flow_load(self, sid, data):
        """處理流程載入事件"""
        print(f"📂 Flow Designer 載入請求 (sid: {sid}): {data.get('name', 'Unknown')}")
        
        await self.sio.emit('flow_loaded', {
            'name': data.get('name', 'Unknown'),
            'timestamp': time.time()
        }, room=sid)
        
        return {
            "success": True,
            "message": f"流程 '{data.get('name', 'Unknown')}' 已載入"
        }

    async def flow_validate(self, sid, data):
        """處理流程驗證事件"""
        print(f"✅ Flow Designer 驗證請求 (sid: {sid})")
        
        # Simple validation logic - can be enhanced
        valid = True
        errors = []
        
        if not data.get('nodes'):
            valid = False
            errors.append("流程必須包含至少一個節點")
        
        await self.sio.emit('flow_validation_result', {
            'valid': valid,
            'errors': errors,
            'timestamp': time.time()
        }, room=sid)
        
        return {
            "success": True,
            "valid": valid,
            "errors": errors
        }

    async def notify_machines(self, sid):
        machines = machine_all()
        payload = {"machines": machines}
        # print(machines)
        await self.sio.emit("machine_list", jsonable_encoder(payload), room=sid)

    async def notify_rooms(self, sid):
        rooms = room_all()
        payload = {"rooms": rooms}
        # print(rooms)
        await self.sio.emit("room_list", jsonable_encoder(payload), room=sid)

    async def notify_signals(self, sid):
        signals = signal_all()
        payload = {"signals": signals}
        # print(signals)
        await self.sio.emit("signal_list", jsonable_encoder(payload), room=sid)

    async def notify_carriers(self, sid):
        carriers = carrier_all()
        payload = {"carriers": carriers}
        # print(carriers)
        await self.sio.emit("carrier_list", jsonable_encoder(payload), room=sid)

    async def notify_racks(self, sid):
        racks = rack_all()
        payload = {"racks": racks}
        await self.sio.emit("rack_list", jsonable_encoder(payload), room=sid)

    async def notify_tasks(self, sid):
        tasks = task_all()
        payload = {"tasks": tasks}
        await self.sio.emit("task_list", jsonable_encoder(payload), room=sid)

    async def notify_map(self, sid):
        nodes = node_all()
        edges = edge_all()
        kuka_nodes = kuka_node_all()
        kuka_edges = kuka_edge_all()
        agvs = get_all_agvs()  # 新增 AGV 資料
        # nodes.extend(kuka_nodes)
        # edges.extend(kuka_edges)
        payload = {"nodes": nodes, "edges": edges,
                   "kukaNodes": kuka_nodes, "kukaEdges": kuka_edges,
                   "agvs": agvs}  # 包含 AGV 資料
        await self.sio.emit("map_info", jsonable_encoder(payload), room=sid)

    async def notify_locations(self, sid):
        locations = get_all_locations()
        payload = {"locations": locations}
        await self.sio.emit("location_list", jsonable_encoder(payload), room=sid)

    async def notify_agvs(self, sid):
        agvs = get_all_agvs()
        payload = {"agvs": agvs}
        await self.sio.emit("agv_list", jsonable_encoder(payload), room=sid)

    async def notify_by_modifylog(self, sid):
        now = datetime.now(timezone.utc)
        # The check interval is 0.1s. We use a slightly larger window to be safe.
        check_since = now - timedelta(seconds=0.2)

        logs = modify_log_all_objects()

        recent_updates = [log for log in logs if log.modified_at > check_since]

        updated_tables = {log.table_name for log in recent_updates}

        if not updated_tables:
            return

        # Mapping table names to notification functions
        notify_map = {
            "agv": self.notify_agvs,
            "rack": self.notify_racks,
            "carrier": self.notify_carriers,
            "signal": self.notify_signals,
            "task": self.notify_tasks,
            # Add other mappings as needed
        }

        for table_name in updated_tables:
            if table_name in notify_map:
                # print(f"Notify {table_name} updated by modify_log")
                await notify_map[table_name](sid)

                # 如果是 AGV 更新，同時更新地圖資料
                if table_name == "agv":
                    await self.notify_map(sid)

    async def _periodic_notify(self):
        while True:
            now = time.perf_counter()
            for task in self.tasks:
                if now - task["last_time"] >= task["interval"]:
                    for sid in list(self.connected_sids):  # ⏺️ 遍歷所有活躍 sid
                        try:
                            await task["func"](sid)
                        except Exception as e:
                            print(
                                f"❌ Error in task {task['func'].__name__} for sid {sid}: {e}")
                    task["last_time"] = now
            await asyncio.sleep(0.05)

    async def close(self):
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                print("Periodic broadcast task cancelled.")
