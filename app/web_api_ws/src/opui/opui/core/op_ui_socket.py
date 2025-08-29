import asyncio
from datetime import datetime
from fastapi.encoders import jsonable_encoder
from typing import Dict, Set
from opui.database.operations import get_or_create_or_update_client, get_client, create_or_update_product, product_all, machine_all, room_all
from opui.monitoring.task_monitor import TaskMonitor
from opui.core.task_service import TaskService


class OpUiSocket:
    def __init__(self, sio):
        self.sio = sio
        self.user_sid_map = {}  # clientId -> sid
        self.task_monitor = TaskMonitor()
        self.task_monitor.set_completion_callback(self._handle_task_completion)
        self.task_service = TaskService()  # 新增任務服務
        self.init_socketio()

    def init_socketio(self):
        """初始化 Socket.IO 事件處理器"""
        self.sio.on('connect')(self.connect)
        self.sio.on('disconnect')(self.disconnect)
        self.sio.on('login')(self.login)
        self.sio.on('client_update')(self.client_update)
        self.sio.on('restore_client_by_id')(self.restore_client_by_id)  # 新增：恢復客戶端資料
        # self.sio.on('add_product')(self.add_product)
        self.sio.on('add_rack')(self.add_rack)
        self.sio.on('del_rack')(self.del_rack)
        self.sio.on('call_empty')(self.call_empty)
        self.sio.on('dispatch_full')(self.dispatch_full)
        self.sio.on('cancel_task')(self.cancel_task)  # 新增取消任務事件
        self.sio.on('confirm_delivery')(self.confirm_delivery)  # 新增確認送達事件
        self.sio.on('test_complete_task')(self.test_complete_task)  # 測試用：手動完成任務
        self.sio.on('get_task_status')(self.get_task_status)  # 新增：查詢任務狀態
        self.sio.on('get_active_tasks')(self.get_active_tasks)  # 新增：查詢活躍任務
        # HMI 相關事件
        self.sio.on('request_hmi_data')(self.request_hmi_data)  # HMI 請求資料

    async def connect(self, sid, environ):
        """處理客戶端連線 - 優化版：不在連線時發送所有資料"""
        print("🔌 使用者連線:", sid)
        # 確保任務監聽已啟動
        if not self.task_monitor.task_monitoring_started:
            self.task_monitor.start_monitoring()
            # 從資料庫恢復進行中的任務
            await self.task_monitor.restore_from_database()

        # 只發送當前監聽的任務狀態給前端（這是必要的）
        await self._sync_active_tasks_to_client(sid)

        # 嘗試恢復客戶端資料（如果存在的話）
        # 注意：靜態資料（產品、機台、房間等）將在登入或恢復時發送，而不是在連線時
        await self._restore_client_data(sid)

    async def disconnect(self, sid):
        """處理客戶端斷線"""
        print("❌ 使用者離線:", sid)
        for clientId, s in list(self.user_sid_map.items()):
            if s == sid:
                del self.user_sid_map[clientId]

    # ==================== 共用方法 ====================
    
    async def _send_client_notifications(self, sid):
        """統一的客戶端通知方法 - 發送所有靜態資料（只在首次登入時使用）"""
        print(f"📤 發送所有靜態資料給客戶端 (sid: {sid})")
        await self.notify_products(sid)
        await self.notify_machines(sid)
        await self.notify_rooms(sid)
        await self.notify_parking_list(sid)

    async def _broadcast_data_update(self, data_type, data=None):
        """向所有連線的客戶端廣播資料更新（當資料變更時使用）"""
        if not self.user_sid_map:
            print(f"⚠️ 沒有連線的客戶端，跳過廣播 {data_type}")
            return

        print(f"📢 廣播 {data_type} 資料更新給 {len(self.user_sid_map)} 個客戶端")

        try:
            if data_type == "products":
                for sid in self.user_sid_map.values():
                    await self.notify_products(sid)
            elif data_type == "machines":
                for sid in self.user_sid_map.values():
                    await self.notify_machines(sid)
            elif data_type == "rooms":
                for sid in self.user_sid_map.values():
                    await self.notify_rooms(sid)
            elif data_type == "parking":
                for sid in self.user_sid_map.values():
                    await self.notify_parking_list(sid)
            else:
                print(f"⚠️ 未知的資料類型: {data_type}")
        except Exception as e:
            print(f"❌ 廣播資料更新失敗: {e}")

    def _format_client_data(self, client_data):
        """統一的客戶端資料格式化"""
        client_dict = dict(client_data)
        # 把 datetime 轉成 ISO 字串
        if client_dict.get("created_at"):
            client_dict["created_at"] = client_dict["created_at"].isoformat()
        if client_dict.get("updated_at"):
            client_dict["updated_at"] = client_dict["updated_at"].isoformat()
        return client_dict

    async def _handle_client_session(self, sid, client_data, clientId, send_static_data=True):
        """處理客戶端會話的共用邏輯"""
        # 更新 sid 映射
        self.user_sid_map[clientId] = sid

        # 格式化客戶端資料
        formatted_client = self._format_client_data(client_data)

        # 只在需要時發送靜態資料（避免重複發送）
        if send_static_data:
            await self._send_client_notifications(sid)

        return formatted_client

    # ==================== 客戶端管理 ====================

    async def login(self, sid, client):
        """處理客戶端登入（統一使用扁平化格式）"""
        print(f"🔐 login 收到資料:")
        print(f"  sid: {sid}")
        print(f"  client: {client}")

        # 直接使用扁平化格式提取資料
        clientId = client.get("clientId")

        # 如果沒有提供 clientId，使用預設值（應該由前端確保總是提供）
        if not clientId:
            clientId = "device_undefined"
            print(f"⚠️ login 沒有提供 clientId，使用預設值: {clientId}")

        client['clientId'] = clientId

        print(f"  最終使用的 clientId: {clientId}")

        # 查詢或建立 client 記錄
        db_client = get_or_create_or_update_client(client)

        # 使用共用邏輯處理會話（首次登入需要發送靜態資料）
        formatted_client = await self._handle_client_session(sid, db_client, clientId, send_static_data=True)

        print(f"🔗 登入後建立映射: clientId={clientId} -> sid={sid}")
        print(f"🔍 當前 user_sid_map: {self.user_sid_map}")

        print(f"✅ 登入成功，返回 clientId: {formatted_client.get('clientId')}")

        # 登入成功，回傳 client 資訊（包含 clientId 供前端儲存）
        return {
            "success": True,
            "message": f"登入成功，clientId: {formatted_client.get('clientId')}",
            "client": jsonable_encoder(db_client),
            "clientId": formatted_client.get('clientId')  # 重要：返回 clientId 給前端
        }

    async def _restore_client_data(self, sid):
        """嘗試從localStorage的clientId恢復客戶端資料"""
        try:
            # 發送一個特殊事件，請求前端提供clientId
            await self.sio.emit("request_client_id", {}, room=sid)
        except Exception as e:
            print(f"❌ 恢復客戶端資料失敗: {e}")

    async def restore_client_by_id(self, sid, data):
        """根據clientId恢復客戶端資料"""
        try:
            clientId = data.get("clientId")
            if not clientId:
                return {"success": False, "message": "缺少clientId"}

            # 從資料庫獲取客戶端資料
            client_data = get_client({"clientId": clientId})
            if not client_data:
                return {"success": False, "message": "找不到客戶端資料"}

            print(f"🔄 恢復客戶端資料: clientId={clientId}")
            print(f"🔄 恢復的OP資料: {client_data.get('op', {})}")

            # 使用共用邏輯處理會話（恢復時不需要重複發送靜態資料）
            formatted_client = await self._handle_client_session(sid, client_data, clientId, send_static_data=False)

            # 發送恢復的資料給前端
            await self.sio.emit("client_data_restored", {
                "success": True,
                "client": jsonable_encoder(client_data)
            }, room=sid)

            return {"success": True, "message": "客戶端資料已恢復"}

        except Exception as e:
            print(f"❌ 恢復客戶端資料失敗: {e}")
            return {"success": False, "message": f"恢復失敗: {str(e)}"}

    # ==================== 通知方法 ====================

    async def notify_products(self, sid):
        """發送產品列表通知"""
        products = product_all()
        payload = {"products": products}
        await self.sio.emit("product_list", jsonable_encoder(payload), room=sid)

    async def notify_machines(self, sid):
        """發送機台列表通知"""
        machines = machine_all()
        payload = {"machines": machines}
        await self.sio.emit("machine_list", jsonable_encoder(payload), room=sid)

    async def notify_rooms(self, sid):
        """發送房間列表通知"""
        rooms = room_all()
        payload = {"rooms": rooms}
        await self.sio.emit("room_list", jsonable_encoder(payload), room=sid)

    async def notify_parking_list(self, sid):
        """發送停車格列表通知"""
        clientId, machine_id, err = self._require_client_and_machine(sid)
        if err:
            return err
        parking_list = self.get_parking_list_by_machineId(machine_id)
        await self.sio.emit("parking_list", parking_list, room=sid)
        return {"success": True}

    async def notify_message(self, sid, message):
        """發送通知訊息"""
        payload = jsonable_encoder({"message": message})
        await self.sio.emit("notify_message", payload, room=sid)

    async def error_message(self, sid, message):
        """發送錯誤訊息"""
        payload = jsonable_encoder({"message": message})
        await self.sio.emit("error_message", payload, room=sid)

    # ==================== 客戶端更新 ====================

    async def client_update(self, sid, data):
        """處理客戶端資料更新（統一使用扁平化格式）"""
        # 直接使用扁平化格式提取資料
        clientId = data.get("clientId")

        # 如果沒有提供 clientId，使用預設值（應該由前端確保總是提供）
        if not clientId:
            clientId = "device_undefined"
            print(f"⚠️ 沒有提供 clientId，使用預設值: {clientId}")

        userAgent = data.get("userAgent") or ""
        machineId = data.get("machineId")

        # 確保 machineId 是有效的整數
        if machineId is not None:
            try:
                machineId = int(machineId)
            except (ValueError, TypeError):
                print(f"⚠️ 無效的 machineId 格式: {machineId}，設為預設值 1")
                machineId = 1
        else:
            print(f"⚠️ machineId 為 None，設為預設值 1")
            machineId = 1

        isConnected = data.get("isConnected") or False

        # 處理操作資料，直接使用 op 欄位
        op = data.get("op") or {}

        # 向後相容性：將舊的 product 欄位遷移為 products
        op = self._migrate_product_to_products(op)

        # 詳細日誌記錄
        print(f"🔄 client_update 收到資料:")
        print(f"  clientId: {clientId}")
        print(f"  machineId: {machineId}")
        print(f"  isConnected: {isConnected}")
        print(f"  userAgent: {userAgent}")

        # 計算資料大小
        import sys, json
        data_size = sys.getsizeof(json.dumps(data)) / 1024
        op_size = sys.getsizeof(json.dumps(op)) / 1024
        print(f"  資料大小: {data_size:.2f} KB")
        print(f"  op 欄位大小: {op_size:.2f} KB")

        # 只顯示 op 欄位的摘要，避免日誌過大
        print(f"  op 欄位: {str(op)[:100]}..." if len(str(op)) > 100 else f"  op 欄位: {op}")

        db_client = get_or_create_or_update_client({
            "clientId": clientId,
            "userAgent": userAgent,
            "op": op,
            "machineId": machineId,
        })

        print(f"✅ 資料庫更新完成，client.op: {db_client.get('op', {})}")

        # 建立 clientId 到 sid 的映射
        print(f"🔗 建立映射: clientId={clientId} -> sid={sid}")
        self.user_sid_map[clientId] = sid
        print(f"🔍 當前 user_sid_map: {self.user_sid_map}")

        client_dict = dict(db_client)  # 確保是 dict

        # 只在機台變更時更新停車列表，避免不必要的通知
        current_machine = getattr(self, '_last_machine_id', {}).get(clientId)
        if current_machine != machineId:
            await self.notify_parking_list(sid)
            # 記錄當前機台ID
            if not hasattr(self, '_last_machine_id'):
                self._last_machine_id = {}
            self._last_machine_id[clientId] = machineId

        return {"success": True, "message": "設定已儲存",
                "client": jsonable_encoder(db_client),
                "clientId": client_dict.get('clientId')}

    # ==================== 輔助方法 ====================

    def _migrate_product_to_products(self, op_data):
        """向後相容性：將舊的 product 欄位遷移為 products 欄位"""
        if not op_data or not isinstance(op_data, dict):
            return op_data

        migrated_op = {}

        for side in ['left', 'right']:
            if side in op_data and isinstance(op_data[side], dict):
                side_data = op_data[side].copy()

                # 如果存在舊的 product 欄位，遷移為 products
                if 'product' in side_data and 'products' not in side_data:
                    side_data['products'] = side_data.pop('product')
                    print(f"🔄 遷移 {side} 側的 product → products")

                # 確保 products 欄位存在且為陣列
                if 'products' not in side_data:
                    side_data['products'] = []
                elif not isinstance(side_data['products'], list):
                    side_data['products'] = []

                # 確保 productSelected 欄位存在
                if 'productSelected' not in side_data:
                    side_data['productSelected'] = 0

                migrated_op[side] = side_data
            else:
                # 如果側邊資料不存在，創建預設結構
                migrated_op[side] = {
                    'productSelected': 0,
                    'products': []
                }

        return migrated_op

    def _get_client_and_machine_id(self, sid):
        """獲取客戶端ID和機台ID"""
        clientId = None
        for cid, s in self.user_sid_map.items():
            if s == sid:
                clientId = cid
                break

        print(f"🔍 _get_client_and_machine_id: sid={sid}, clientId={clientId}")

        if not clientId:
            print(f"❌ 找不到 clientId for sid={sid}")
            return None, None

        try:
            client = get_client({"clientId": clientId})
            print(f"🔍 從資料庫獲取的客戶端資料: {client}")

            # 如果客戶端資料為空或不包含 machineId，使用預設值 1
            if not client or not isinstance(client, dict):
                print(f"⚠️ 客戶端資料無效，使用預設 machineId=1")
                return clientId, 1

            machine_id = client.get("machineId")
            print(f"🔍 machineId from client: {machine_id}")

            if not machine_id:
                print(f"⚠️ machineId 為空或 None，使用預設值 1")
                return clientId, 1

            print(f"✅ 成功獲取 machineId: {machine_id}")
            return clientId, int(machine_id)
        except Exception as e:
            print(f"❌ 獲取 machineId 時發生錯誤: {e}")
            print(f"⚠️ 使用預設 machineId=1")
            return clientId, 1

    def _require_client_and_machine(self, sid):
        """要求客戶端和機台資訊，返回錯誤訊息如果缺少"""
        print(f"🔍 _require_client_and_machine: 開始驗證 sid={sid}")

        clientId, machine_id = self._get_client_and_machine_id(sid)

        print(f"🔍 驗證結果: clientId={clientId}, machine_id={machine_id}")

        if not clientId:
            print(f"❌ 客戶端資訊驗證失敗: clientId 為空")
            return None, None, {"success": False, "message": "找不到客戶端資訊"}
        if not machine_id:
            print(f"❌ 機台驗證失敗: machine_id 為空，clientId={clientId}")
            print(f"🔍 當前 user_sid_map: {self.user_sid_map}")
            return clientId, None, {"success": False, "message": "請先選擇機台"}

        print(f"✅ 驗證成功: clientId={clientId}, machine_id={machine_id}")
        return clientId, machine_id, None

    def _update_machine_parking_status(self, machine_id, node_id, status=1):
        """更新機台停車格狀態"""
        from opui.database.operations import machine_crud, connection_pool
        with connection_pool.get_session() as session:
            machine = machine_crud.get_by_id(session, machine_id)
            if machine:
                updated = False
                if getattr(machine, 'parking_space_1', None) == node_id:
                    machine.parking_space_1_status = status
                    updated = True
                if getattr(machine, 'parking_space_2', None) == node_id:
                    machine.parking_space_2_status = status
                    updated = True
                if updated:
                    machine_crud.update(session, machine.id, machine)
                    print(f"🔄 更新停車格狀態: machine_id={machine_id}, node_id={node_id}, status={status}")
                return updated
        return False

    def get_parking_list_by_machineId(self, machine_id):
        """根據機台ID獲取停車格列表"""
        from opui.database.operations import connection_pool, rack_crud, machine_crud
        parking_list = {"left": [], "right": []}

        print(f"🔍 獲取機台 {machine_id} 的停車格資料")

        with connection_pool.get_session() as session:
            machine = machine_crud.get_by_id(session, int(machine_id))

            if not machine:
                print(f"❌ 找不到機台 {machine_id}")
                return parking_list

            print(
                f"🔍 機台配置: parking_space_1={machine.parking_space_1}, parking_space_2={machine.parking_space_2}")

            if getattr(machine, 'parking_space_1', None):
                left_racks = [r for r in rack_crud.get_all(
                    session) if r.location_id == machine.parking_space_1]
                parking_list["left"] = [
                    {"id": r.id, "name": r.name} for r in left_racks]
                print(f"🔍 左側停車格 {machine.parking_space_1} 找到 {len(left_racks)} 個 rack")
            else:
                print(f"❌ 機台 {machine_id} 沒有配置 parking_space_1")

            if getattr(machine, 'parking_space_2', None):
                right_racks = [r for r in rack_crud.get_all(
                    session) if r.location_id == machine.parking_space_2]
                parking_list["right"] = [
                    {"id": r.id, "name": r.name} for r in right_racks]
                print(f"🔍 右側停車格 {machine.parking_space_2} 找到 {len(right_racks)} 個 rack")
            else:
                print(f"❌ 機台 {machine_id} 沒有配置 parking_space_2")

        print(f"🔍 停車格資料結果: {parking_list}")
        return parking_list

    async def _sync_active_tasks_to_client(self, sid):
        """同步當前活躍任務狀態給前端客戶端"""
        try:
            # 獲取客戶端和機台資訊
            clientId, machine_id = self._get_client_and_machine_id(sid)
            if not machine_id:
                print(f"⚠️ 客戶端 {sid} 尚未選擇機台，跳過任務狀態同步")
                return

            # 獲取當前監聽的任務
            monitored_tasks = self.task_monitor.get_monitored_tasks()
            active_tasks = {}

            for task_id, task_info in monitored_tasks.items():
                # 只同步該機台的任務
                if task_info.get('machine_id') == machine_id:
                    node_id = task_info.get('node_id')
                    task_type = task_info.get('task_type')

                    # 判斷是左側還是右側停車格
                    side = self._get_side_by_node_id(machine_id, node_id)
                    if side:
                        # 🔧 獲取詳細的任務狀態
                        detailed_status = self._get_detailed_task_status(task_id, task_type, machine_id, node_id)

                        active_tasks[side] = {
                            'task_id': task_id,
                            'task_type': task_type,
                            'node_id': node_id,
                            'status': detailed_status,
                            'createdAt': task_info.get('created_at', 0)
                        }

            # 發送活躍任務狀態給前端
            if active_tasks:
                print(f"🔄 同步活躍任務狀態給客戶端 {sid}: {active_tasks}")
                await self.sio.emit("active_tasks", active_tasks, room=sid)
            else:
                print(f"🔄 客戶端 {sid} 沒有活躍任務")

        except Exception as e:
            print(f"❌ 同步活躍任務狀態失敗: {e}")

    def _get_detailed_task_status(self, task_id, task_type, machine_id, node_id):
        """獲取詳細的任務狀態"""
        try:
            from opui.database.operations import task_crud, connection_pool
            from db_proxy.models.machine import Machine

            with connection_pool.get_session() as session:
                # 獲取任務詳細資訊
                task = task_crud.get_by_id(session, task_id)
                if not task:
                    return 'pending'

                # 獲取停車格狀態
                parking_status = self._get_machine_parking_status(machine_id, node_id)

                if task_type == 'call_empty':
                    # 叫空車任務狀態判斷
                    if parking_status == Machine.PARKING_TASK_COMPLETED:
                        return 'delivered'  # 已送達，等待確認
                    else:
                        return 'pending'    # 進行中

                elif task_type == 'dispatch_full':
                    # 派車任務狀態判斷（派車任務不需要確認送達）
                    return 'pending'  # 派車任務只有進行中狀態

                else:
                    return 'pending'

        except Exception as e:
            print(f"❌ 獲取詳細任務狀態失敗: {e}")
            return 'pending'

    def _get_side_by_node_id(self, machine_id, node_id):
        """根據機台ID和node_id判斷是左側還是右側停車格"""
        try:
            from opui.database.operations import machine_crud, connection_pool
            with connection_pool.get_session() as session:
                machine = machine_crud.get_by_id(session, machine_id)
                if machine:
                    if machine.parking_space_1 == node_id:
                        return "left"
                    elif machine.parking_space_2 == node_id:
                        return "right"
                return None
        except Exception as e:
            print(f"❌ 判斷停車格側邊失敗: {e}")
            return None

    # ==================== 料架管理 ====================

    async def add_rack(self, sid, data):
        """新增料架到停車格"""
        try:
            clientId, machine_id, err = self._require_client_and_machine(sid)
            if err:
                return err

            # 支援兩種參數名稱：rack (新) 和 rackName (舊)，確保向後相容
            rack_name = data.get("rack") or data.get("rackName")
            side = data.get("side")  # "left" 或 "right"

            if not rack_name or not side:
                return {"success": False, "message": "缺少料架名稱或停車格位置"}

            from opui.database.operations import connection_pool, rack_crud, machine_crud

            session = connection_pool.get_session()
            try:
                machine = machine_crud.get_by_id(session, machine_id)
                if not machine:
                    return {"success": False, "message": f"找不到機台 {machine_id}"}

                print(f"🏭 找到機台: ID={machine_id}, Name={machine.name}")

                # 根據 side 決定停車格位置
                if side == "left":
                    location_id = machine.parking_space_1
                elif side == "right":
                    location_id = machine.parking_space_2
                else:
                    return {"success": False, "message": "無效的停車格位置"}

                if not location_id:
                    return {"success": False, "message": f"機台 {machine_id} 沒有配置 {side} 停車格"}

                print(f"📍 停車格位置: {side} -> location_id={location_id}")

                exist_rack = rack_crud.get_by_field(session, "name", rack_name)
                if exist_rack:
                    print(f"📦 找到現有料架: {rack_name}")
                    # 檢查料架是否已經分配到其他停車格
                    if exist_rack.location_id and exist_rack.location_id != location_id:
                        return {"success": False, "message": f"料架 {rack_name} 已分配到其他停車格"}

                    # 更新料架的停車格位置
                    exist_rack.location_id = location_id
                    rack_crud.update(session, exist_rack.id, exist_rack)
                    rack_id = exist_rack.id
                    action = "分配到停車格"
                    print(f"✅ 更新現有料架成功: {rack_name}")
                else:
                    # 料架不存在於資料表中，不允許新增
                    print(f"❌ 料架不存在: {rack_name}")
                    return {"success": False, "message": f"料架 {rack_name} 不存在於系統中，請先在料架管理中新增此料架"}

                await self.notify_parking_list(sid)
                return {"success": True, "message": f"料架 {rack_name} [{rack_id}] 已{action}成功"}
            except Exception as e:
                print(f"❌ 新增料架時發生錯誤: {e}")
                raise e
            finally:
                session.close()
        except Exception as e:
            return {"success": False, "message": f"料架新增失敗: {str(e)}"}

    async def del_rack(self, sid, data):
        """刪除料架"""
        try:
            # 添加機台驗證，確保用戶已選擇機台
            clientId, machine_id, err = self._require_client_and_machine(sid)
            if err:
                return err

            rack_id = data.get("rackId")
            if not rack_id:
                return {"success": False, "message": "缺少料架ID"}

            from opui.database.operations import connection_pool, rack_crud

            session = connection_pool.get_session()
            try:
                rack = rack_crud.get_by_id(session, rack_id)
                if not rack:
                    return {"success": False, "message": f"找不到料架 {rack_id}"}

                print(f"🗑️ 準備刪除料架: ID={rack_id}, Name={rack.name}")

                # 移除料架的停車格分配
                rack.location_id = None
                rack_crud.update(session, rack.id, rack)

                print(f"✅ 料架刪除成功: {rack.name}")

                await self.notify_parking_list(sid)
                return {"success": True, "message": f"料架 {rack.name} 已從停車格移除"}
            except Exception as e:
                print(f"❌ 刪除料架時發生錯誤: {e}")
                raise e
            finally:
                session.close()
        except Exception as e:
            return {"success": False, "message": f"料架刪除失敗: {str(e)}"}

    # ==================== 任務管理 ====================

    async def call_empty(self, sid, data):
        """叫空車任務"""
        try:
            from opui.database.operations import create_task, get_call_empty_work_id
            from shared_constants.task_status import TaskStatus

            # 獲取側邊和機台資訊
            side = data.get("side")  # "left" 或 "right"
            clientId, machine_id, err = self._require_client_and_machine(sid)
            if err:
                return err

            # 優先使用前端傳來的 parkingSpace 參數（向後相容）
            node_id = data.get("parkingSpace")
            if node_id:
                print(f"🔍 使用前端傳來的停車格 ID: {node_id}")
            else:
                # 如果前端沒有傳 parkingSpace，則根據機台和側邊獲取正確的 node_id
                node_id = self._get_parking_space_node_id(machine_id, side)
                if not node_id:
                    return {"success": False, "message": f"找不到機台 {machine_id} 的 {side} 側停車格"}

            ok, msg = self._check_parking_space_status(machine_id, node_id)
            if not ok:
                return {"success": False, "message": msg}

            # 準備任務資料
            task_data = {
                "name": f"叫空車 - 停車位 [{node_id}]",
                "description": f"操作員從機台 {machine_id} 叫空車到停車位 [{node_id}]",
                "work_id": get_call_empty_work_id(),
                "status_id": TaskStatus.REQUESTING,
                "node_id": node_id,
                "priority": 1,
                "parameters": {
                    "node_id": node_id,
                    "machine_id": machine_id,
                    "client_id": clientId,
                    "task_type": "call_empty",
                    "side": side  # 🔧 新增：保存側邊資訊
                }
            }
            created_task = create_task(task_data)
            task_id = created_task['id']
            print(f"[callEmpty] 任務已創建: ID={task_id}, 名稱={created_task['name']}")

            # 開始監聽這個叫車任務
            self.task_monitor.add_task(task_id, machine_id, node_id,
                                       TaskStatus.REQUESTING, "call_empty")

            # 更新停車格狀態
            self._update_machine_parking_status(machine_id, node_id, 1)

            # 發送詳細的任務狀態更新
            await self._notify_task_creation(task_id, machine_id, {
                'task_id': task_id,
                'side': side,
                'task_type': 'call_empty',
                'node_id': node_id,
                'machine_id': machine_id,
                'client_id': clientId
            })

            await self.notify_machines(sid)
            return {
                "success": True,
                "message": f"叫車成功，任務 ID: {task_id}",
                "task_id": task_id,
                "status": TaskStatus.REQUESTING,
                "status_name": TaskStatus.get_name(TaskStatus.REQUESTING)
            }
        except Exception as e:
            print(f"[callEmpty] 錯誤: {str(e)}")
            return {"success": False, "message": f"叫車失敗: {str(e)}"}

    async def dispatch_full(self, sid, data):
        """派滿車任務"""
        try:
            from opui.database.operations import create_task, get_dispatch_full_work_id
            from shared_constants.task_status import TaskStatus

            # 獲取任務參數
            side = data.get("side")  # "left" 或 "right"
            # 支援兩種參數名稱：productName (新) 和 name (舊)，確保向後相容
            product_name = data.get("productName") or data.get("name")
            count = data.get("count")
            rack_id = data.get("rackId")
            room = data.get("room")

            clientId, machine_id, err = self._require_client_and_machine(sid)
            if err:
                return err

            if not all([side, product_name, count, rack_id, room]):
                return {"success": False, "message": "缺少必要參數"}

            # 根據機台和側邊獲取正確的 node_id
            node_id = self._get_parking_space_node_id(machine_id, side)
            if not node_id:
                return {"success": False, "message": f"找不到機台 {machine_id} 的 {side} 側停車格"}

            ok, msg = self._check_parking_space_status(machine_id, node_id)
            if not ok:
                return {"success": False, "message": msg}

            # 準備任務資料
            task_data = {
                "name": f"派滿車 - {product_name} x{count} 從停車位 [{node_id}]",
                "description": f"操作員從機台 {machine_id} 派滿車，產品: {product_name}，數量: {count}，來源停車位: [{node_id}]",
                "work_id": get_dispatch_full_work_id(),
                "status_id": TaskStatus.REQUESTING,
                "priority": 2,
                "node_id": node_id,
                "parameters": {
                    "node_id": node_id,
                    "product_name": product_name,
                    "count": count,
                    "rack_id": rack_id,
                    "room": room,
                    "side": side,
                    "machine_id": machine_id,
                    "client_id": clientId,
                    "task_type": "dispatch_full"
                }
            }
            created_task = create_task(task_data)
            task_id = created_task['id']
            print(f"[dispatchFull] 任務已創建: ID={task_id}, 名稱={created_task['name']}")

            # 開始監聽這個派車任務
            self.task_monitor.add_task(task_id, machine_id, node_id,
                                       TaskStatus.REQUESTING, "dispatch_full")

            self._update_machine_parking_status(machine_id, node_id, 1)
            await self.notify_machines(sid)
            return {"success": True, "message": f"派車成功，任務 ID: {task_id}"}
        except Exception as e:
            print(f"[dispatchFull] 錯誤: {str(e)}")
            return {"success": False, "message": f"派車失敗: {str(e)}"}

    async def cancel_task(self, sid, data):
        """取消任務"""
        from opui.database.operations import delete_task_by_parking, connection_pool, machine_crud

        # 獲取側邊和機台資訊
        side = data.get("side")  # "left" 或 "right"
        clientId, machine_id, err = self._require_client_and_machine(sid)
        if err:
            return err

        # 根據機台和側邊獲取正確的 node_id
        node_id = self._get_parking_space_node_id(machine_id, side)
        if not node_id:
            return {"success": False, "message": f"找不到機台 {machine_id} 的 {side} 側停車格"}

        # 刪除任務
        deleted = delete_task_by_parking(node_id)
        if not deleted:
            print(f"[op_ui_socket.py] delete_task_by_parking({node_id}) failed")

        # 重設機台停車格狀態
        print(f"🔄 嘗試更新機台停車格狀態: machine_id={machine_id}, node_id={node_id}, status=0")
        updated = self._update_machine_parking_status(machine_id, node_id, 0)
        print(f"🔄 機台停車格狀態更新結果: {updated}")

        if updated:
            print(f"✅ 機台狀態更新成功，通知前端")
            await self.notify_machines(sid)
            await self.notify_parking_list(sid)
        else:
            print(f"❌ 機台狀態更新失敗")

        await self.notify_message(sid, f"已取消停車位 [{node_id}] 的任務")
        return {"success": True, "message": f"已取消停車位 [{node_id}] 的任務"}

    async def confirm_delivery(self, sid, data):
        """確認rack架已送達並搬移至作業區"""
        try:
            # 獲取側邊和機台資訊
            side = data.get("side")  # "left" 或 "right"
            clientId, machine_id, err = self._require_client_and_machine(sid)
            if err:
                return err

            # 根據機台和側邊獲取正確的 node_id
            node_id = self._get_parking_space_node_id(machine_id, side)
            if not node_id:
                return {"success": False, "message": f"找不到機台 {machine_id} 的 {side} 側停車格"}

            # 停止監聽該停車格的任務
            # 🔧 臨時修復：手動查找並移除任務
            tasks_to_remove = []
            monitored_tasks = self.task_monitor.get_monitored_tasks()

            for task_id, task_info in monitored_tasks.items():
                if task_info.get('node_id') == node_id:
                    tasks_to_remove.append(task_id)

            # 移除找到的任務
            for task_id in tasks_to_remove:
                self.task_monitor.remove_task(task_id)
                print(f"🗑️ 確認送達：停止監聽停車格 {node_id} 的任務 {task_id}")

            if tasks_to_remove:
                print(f"✅ 確認送達：已移除停車格 {node_id} 的 {len(tasks_to_remove)} 個任務監聽")
            else:
                print(f"⚠️ 確認送達：未找到停車格 {node_id} 的監聽任務")

            # 重設機台停車格狀態為未佔用(0)
            updated = self._update_machine_parking_status(machine_id, node_id, 0)
            if updated:
                await self.notify_machines(sid)
                await self.notify_parking_list(sid)

                # 🔧 推送任務狀態變更：任務已確認完成
                task_info = {
                    'side': side,
                    'task_type': 'call_empty'
                }
                await self._notify_task_status_change(machine_id, task_info, 'confirmed')

            await self.notify_message(sid, f"已確認停車位 [{node_id}] 的rack架已搬移至作業區")
            return {"success": True, "message": f"已確認停車位 [{node_id}] 的rack架已搬移至作業區"}
        except Exception as e:
            print(f"[confirm_delivery] 錯誤: {str(e)}")
            return {"success": False, "message": f"確認送達失敗: {str(e)}"}

    async def test_complete_task(self, sid, data):
        """測試用：手動完成任務"""
        try:
            task_id = int(data.get("taskId"))
            if not task_id:
                return {"success": False, "message": "缺少任務ID"}

            from opui.database.operations import task_crud, connection_pool
            with connection_pool.get_session() as session:
                task = task_crud.get_by_id(session, task_id)
                if not task:
                    return {"success": False, "message": f"找不到任務 ID {task_id}"}

                # 更新任務狀態為已完成
                from shared_constants.task_status import TaskStatus
                task.status_id = TaskStatus.EXECUTING  # 執行中 (AGV-任務正在執行)
                task_crud.update(session, task.id, task)

                print(f"🧪 測試：手動完成任務 {task_id}")
                return {"success": True, "message": f"任務 {task_id} 已標記為完成"}

        except Exception as e:
            print(f"❌ 測試完成任務失敗: {e}")
            return {"success": False, "message": f"測試失敗: {str(e)}"}

    # ==================== 任務完成處理 ====================

    async def _handle_task_completion(self, task, task_info):
        """處理任務完成"""
        try:
            node_id = task.node_id
            machine_id = task_info['machine_id']
            task_type = task_info.get('task_type', 'call_empty')

            # 🔧 從任務參數中提取 side 資訊
            side = None
            try:
                import json
                if hasattr(task, 'parameters') and task.parameters:
                    if isinstance(task.parameters, str):
                        parameters = json.loads(task.parameters)
                    else:
                        parameters = task.parameters
                    side = parameters.get('side')
            except Exception as e:
                print(f"⚠️ 提取任務 side 資訊失敗: {e}")

            # 將 side 資訊添加到 task_info 中
            if side:
                task_info['side'] = side

            if task_type == 'call_empty':
                # 叫車任務完成：更新停車格狀態為已送達(2)
                updated = self._update_machine_parking_status(machine_id, node_id, 2)
                if updated:
                    await self._notify_clients_for_machine(machine_id)
                    # 叫車完成後發送最新的parking list，因為rack已送到停車格
                    await self._notify_parking_list_for_machine(machine_id)

                    # 🔧 新增：推送任務狀態變更給前端
                    await self._notify_task_status_change(machine_id, task_info, 'delivered')

                    print(
                        f"✅ 叫車任務完成處理完畢: task_id={task.id}, node_id={node_id}, machine_id={machine_id}")
                else:
                    print(f"⚠️ 叫車任務完成但停車格狀態未更新: task_id={task.id}, node_id={node_id}")

            elif task_type == 'dispatch_full':
                # 派車任務完成：更新停車格狀態為未佔用(0)
                updated = self._update_machine_parking_status(machine_id, node_id, 0)
                if updated:
                    await self._notify_clients_for_machine(machine_id)
                    # 派車完成後發送最新的parking list，因為rack已被搬走
                    await self._notify_parking_list_for_machine(machine_id)

                    # 🔧 新增：推送派車任務狀態變更給前端
                    await self._notify_task_status_change(machine_id, task_info, 'completed')

                    # 發送派車完成訊息給相關客戶端
                    rack_name = self._get_rack_name_from_task(task)
                    message = f"AGV已將rack{rack_name}搬離停車位[{node_id}]"
                    await self._notify_message_for_machine(machine_id, message)

                    print(
                        f"✅ 派車任務完成處理完畢: task_id={task.id}, node_id={node_id}, machine_id={machine_id}")
                else:
                    print(f"⚠️ 派車任務完成但停車格狀態未更新: task_id={task.id}, node_id={node_id}")

        except Exception as e:
            print(f"❌ 處理任務完成失敗: {e}")

    async def _notify_task_status_change(self, machine_id, task_info, new_status):
        """推送任務狀態變更給前端"""
        try:
            from shared_constants.task_status import TaskStatus

            # 構建任務狀態更新資料
            task_update = {
                'task_id': task_info.get('task_id'),
                'side': task_info.get('side', 'unknown'),
                'type': task_info.get('task_type', 'call_empty'),
                'status': new_status,
                'status_name': TaskStatus.get_name(new_status),
                'status_description': TaskStatus.get_description(new_status),
                'machine_id': machine_id,
                'node_id': task_info.get('node_id'),
                'updatedAt': datetime.now().isoformat()
            }

            # 推送給使用該機台的所有客戶端
            for client_id, sid in self.user_sid_map.items():
                if await self._client_uses_machine(client_id, machine_id):
                    await self.sio.emit("task_status_update", task_update, room=sid)
                    print(f"📤 推送任務狀態變更給客戶端 {client_id}: {task_update}")

        except Exception as e:
            print(f"❌ 推送任務狀態變更失敗: {e}")

    async def notify_task_progress(self, task_id: int, status: int, message: str = ""):
        """通知任務進度更新（新增方法）"""
        try:
            from opui.database.operations import connection_pool
            from db_proxy.crud.task_crud import task_crud
            from shared_constants.task_status import TaskStatus

            with connection_pool.get_session() as session:
                task = task_crud.get_by_id(session, task_id)
                if not task:
                    print(f"⚠️ 任務 {task_id} 不存在，無法發送進度通知")
                    return

                # 解析任務參數
                params = task.parameters or {}
                machine_id = params.get('machine_id')

                if not machine_id:
                    print(f"⚠️ 任務 {task_id} 缺少 machine_id，無法發送進度通知")
                    return

                # 構建進度更新資料
                progress_update = {
                    'task_id': task_id,
                    'side': params.get('side', 'unknown'),
                    'type': params.get('task_type', 'call_empty'),
                    'status': status,
                    'status_name': TaskStatus.get_name(status),
                    'status_description': TaskStatus.get_description(status),
                    'machine_id': machine_id,
                    'node_id': params.get('node_id'),
                    'message': message,
                    'updatedAt': datetime.now().isoformat()
                }

                # 推送給使用該機台的所有客戶端
                for client_id, sid in self.user_sid_map.items():
                    if await self._client_uses_machine(client_id, machine_id):
                        await self.sio.emit("task_progress_update", progress_update, room=sid)
                        print(f"📤 推送任務進度更新給客戶端 {client_id}: {progress_update}")

        except Exception as e:
            print(f"❌ 推送任務進度更新失敗: {e}")

    async def _notify_task_creation(self, task_id: int, machine_id: int, task_info: dict):
        """通知任務創建（新增方法）"""
        try:
            from shared_constants.task_status import TaskStatus

            # 構建任務創建通知資料
            creation_notification = {
                'event_type': 'task_created',
                'task_id': task_id,
                'side': task_info.get('side', 'unknown'),
                'type': task_info.get('task_type', 'call_empty'),
                'status': TaskStatus.REQUESTING,
                'status_name': TaskStatus.get_name(TaskStatus.REQUESTING),
                'status_description': TaskStatus.get_description(TaskStatus.REQUESTING),
                'machine_id': machine_id,
                'node_id': task_info.get('node_id'),
                'client_id': task_info.get('client_id'),
                'createdAt': datetime.now().isoformat(),
                'message': f"叫空車任務已創建，等待系統處理"
            }

            # 推送給使用該機台的所有客戶端
            for client_id, sid in self.user_sid_map.items():
                if await self._client_uses_machine(client_id, machine_id):
                    await self.sio.emit("task_created", creation_notification, room=sid)
                    print(f"📤 推送任務創建通知給客戶端 {client_id}: {creation_notification}")

        except Exception as e:
            print(f"❌ 推送任務創建通知失敗: {e}")

    async def _notify_clients_for_machine(self, machine_id):
        """通知使用該機台的所有客戶端"""
        try:
            for client_id, sid in self.user_sid_map.items():
                if await self._client_uses_machine(client_id, machine_id):
                    await self.notify_machines(sid)
                    await self.notify_message(sid, "AGV已送達，請確認rack架已搬移至作業區")
        except Exception as e:
            print(f"❌ 通知客戶端失敗: {e}")

    async def _client_uses_machine(self, client_id, machine_id) -> bool:
        """檢查客戶端是否使用指定的機台"""
        try:
            from opui.database.operations import client_crud, connection_pool
            with connection_pool.get_session() as session:
                client = client_crud.get_by_id(session, client_id)
                return client and getattr(client, 'machine_id', None) == machine_id
        except Exception as e:
            print(f"❌ 檢查客戶端機台失敗: {e}")
            return False

    async def _notify_parking_list_for_machine(self, machine_id):
        """通知使用該機台的所有客戶端更新parking list"""
        try:
            notified_count = 0
            for client_id, sid in self.user_sid_map.items():
                if await self._client_uses_machine(client_id, machine_id):
                    await self.notify_parking_list(sid)
                    notified_count += 1

            print(f"📋 已通知 {notified_count} 個客戶端更新parking list (machine_id: {machine_id})")

        except Exception as e:
            print(f"❌ 通知parking list失敗: {e}")

    async def _notify_message_for_machine(self, machine_id, message):
        """通知使用該機台的所有客戶端顯示訊息"""
        try:
            notified_count = 0
            for client_id, sid in self.user_sid_map.items():
                if await self._client_uses_machine(client_id, machine_id):
                    await self.notify_message(sid, message)
                    notified_count += 1

            print(f"💬 已通知 {notified_count} 個客戶端顯示訊息: {message} (machine_id: {machine_id})")

        except Exception as e:
            print(f"❌ 通知訊息失敗: {e}")

    # ==================== 更多輔助方法 ====================

    def _get_parking_space_node_id(self, machine_id, side):
        """根據機台ID和側邊獲取停車格的 node_id"""
        from opui.database.operations import machine_crud, connection_pool

        print(f"🔍 查找停車格: machine_id={machine_id}, side={side}")

        with connection_pool.get_session() as session:
            machine = machine_crud.get_by_id(session, machine_id)
            if not machine:
                print(f"❌ 找不到機台 ID {machine_id}")
                return None

            print(f"🔍 機台資料: id={machine.id}, name={machine.name}")
            print(
                f"🔍 停車格配置: parking_space_1={machine.parking_space_1}, parking_space_2={machine.parking_space_2}")

            if side == "left":
                node_id = machine.parking_space_1
                print(f"🔍 左側停車格: {node_id}")
                return node_id
            elif side == "right":
                node_id = machine.parking_space_2
                print(f"🔍 右側停車格: {node_id}")
                return node_id
            else:
                print(f"❌ 無效的側邊參數: {side}")
                return None

    def _check_parking_space_status(self, machine_id, node_id):
        """檢查停車格狀態"""
        from opui.database.operations import machine_crud, connection_pool
        with connection_pool.get_session() as session:
            machine = machine_crud.get_by_id(session, machine_id)
            if not machine:
                return False, "找不到機台資訊"
            if getattr(machine, 'parking_space_1', None) == node_id:
                if machine.parking_space_1_status != 0:
                    status_msg = self._get_parking_status_message(
                        machine.parking_space_1_status, node_id)
                    return False, status_msg
            if getattr(machine, 'parking_space_2', None) == node_id:
                if machine.parking_space_2_status != 0:
                    status_msg = self._get_parking_status_message(
                        machine.parking_space_2_status, node_id)
                    return False, status_msg
        return True, None

    def _get_parking_status_message(self, status, node_id):
        """根據停車格狀態返回對應的錯誤訊息"""
        from db_proxy.models.machine import Machine

        if status == Machine.PARKING_TASK_ACTIVE:
            return f"停車位 [{node_id}] 已叫車，請先取消"
        elif status == Machine.PARKING_TASK_COMPLETED:
            return f"停車位 [{node_id}] 已送達，請先確認rack架已搬移"
        else:
            return f"停車位 [{node_id}] 狀態異常"

    def _get_rack_name_from_task(self, task):
        """從任務參數中獲取rack名稱"""
        try:
            import json
            if task.parameters:
                params = json.loads(task.parameters) if isinstance(
                    task.parameters, str) else task.parameters

                # 優先使用 rack_name
                if 'rack_name' in params:
                    return params['rack_name']

                # 如果沒有 rack_name，嘗試從 rack_id 獲取rack名稱
                if 'rack_id' in params:
                    rack_id = params['rack_id']
                    rack_name = self._get_rack_name_by_id(rack_id)
                    if rack_name:
                        return rack_name

                return '未知'
            return '未知'
        except Exception as e:
            print(f"❌ 解析任務參數失敗: {e}")
            return '未知'

    def _get_rack_name_by_id(self, rack_id):
        """根據rack_id從資料庫獲取rack名稱"""
        try:
            from opui.database.operations import connection_pool
            from sqlmodel import select
            from db_proxy.models import Rack

            with connection_pool.get_session() as session:
                statement = select(Rack).where(Rack.id == rack_id)
                rack = session.exec(statement).first()
                if rack:
                    return rack.name
                return None
        except Exception as e:
            print(f"❌ 根據rack_id獲取rack名稱失敗: {e}")
            return None

    async def get_task_status(self, sid, data):
        """查詢任務狀態 Socket.IO 事件"""
        try:
            from opui.database.operations import connection_pool
            from db_proxy.crud.task_crud import task_crud
            from shared_constants.task_status import TaskStatus

            task_id = data.get("task_id")
            if not task_id:
                return {"success": False, "message": "缺少 task_id 參數"}

            with connection_pool.get_session() as session:
                task = task_crud.get_by_id(session, task_id)

                if not task:
                    return {"success": False, "message": "任務不存在"}

                # 解析任務參數
                params = task.parameters or {}

                return {
                    "success": True,
                    "task": {
                        "id": task.id,
                        "name": task.name,
                        "description": task.description,
                        "status": task.status_id,
                        "status_name": TaskStatus.get_name(task.status_id) if task.status_id else "未知",
                        "status_description": TaskStatus.get_description(task.status_id) if task.status_id else "未知狀態",
                        "node_id": task.node_id,
                        "priority": task.priority,
                        "created_at": task.created_at.isoformat() if task.created_at else None,
                        "updated_at": task.updated_at.isoformat() if task.updated_at else None,
                        "parameters": params
                    }
                }

        except Exception as e:
            print(f"❌ 查詢任務狀態失敗: {e}")
            return {"success": False, "message": f"查詢失敗: {str(e)}"}

    async def get_active_tasks(self, sid, data):
        """查詢活躍任務 Socket.IO 事件"""
        try:
            from opui.database.operations import connection_pool
            from db_proxy.crud.task_crud import task_crud
            from shared_constants.task_status import TaskStatus
            from sqlmodel import select

            clientId, machine_id, err = self._require_client_and_machine(sid)
            if err:
                return err

            with connection_pool.get_session() as session:
                # 查詢該機台相關的活躍任務
                from db_proxy.models import Task

                active_tasks = session.exec(
                    select(Task).where(
                        Task.parameters["machine_id"].as_integer() == machine_id,
                        Task.status_id.in_([
                            TaskStatus.REQUESTING,
                            TaskStatus.PENDING,
                            TaskStatus.READY_TO_EXECUTE,
                            TaskStatus.EXECUTING
                        ])
                    ).order_by(Task.created_at.desc())
                ).all()

                task_list = []
                for task in active_tasks:
                    params = task.parameters or {}
                    task_list.append({
                        "id": task.id,
                        "name": task.name,
                        "description": task.description,
                        "status": task.status_id,
                        "status_name": TaskStatus.get_name(task.status_id) if task.status_id else "未知",
                        "status_description": TaskStatus.get_description(task.status_id) if task.status_id else "未知狀態",
                        "node_id": task.node_id,
                        "priority": task.priority,
                        "created_at": task.created_at.isoformat() if task.created_at else None,
                        "updated_at": task.updated_at.isoformat() if task.updated_at else None,
                        "task_type": params.get("task_type"),
                        "side": params.get("side")
                    })

                return {
                    "success": True,
                    "machine_id": machine_id,
                    "tasks": task_list,
                    "total": len(task_list)
                }

        except Exception as e:
            print(f"❌ 查詢活躍任務失敗: {e}")
            return {"success": False, "message": f"查詢失敗: {str(e)}"}
    
    async def request_hmi_data(self, sid, data):
        """HMI 請求資料 - 發送 HMI 顯示所需的位置和料架資料"""
        try:
            device_id = data.get('device_id')
            if not device_id:
                print("❌ HMI 請求缺少 device_id")
                await self.sio.emit('hmi_data_update', {
                    'success': False,
                    'message': 'Missing device_id'
                }, to=sid)
                return
            
            print(f"📡 HMI 請求資料: device_id={device_id}")
            
            # 從資料庫獲取 HMI 資料
            from opui.database.operations import connection_pool
            from sqlmodel import select
            import json
            
            with connection_pool.get_session() as session:
                # 1. 查詢 license 獲取權限配置
                from db_proxy.models import License
                license_data = session.exec(
                    select(License).where(License.device_id == device_id)
                ).first()
                
                if not license_data:
                    print(f"❌ 找不到 device_id {device_id} 的授權資料")
                    await self.sio.emit('hmi_data_update', {
                        'success': False,
                        'message': 'Device not authorized'
                    }, to=sid)
                    return
                
                if license_data.device_type != "hmi_terminal":
                    print(f"❌ Device {device_id} 不是 HMI 終端")
                    await self.sio.emit('hmi_data_update', {
                        'success': False,
                        'message': 'Not an HMI terminal'
                    }, to=sid)
                    return
                
                # 2. 解析權限配置
                permissions = license_data.permissions or {}
                location_names = permissions.get("locations", [])
                layout = permissions.get("layout", "2x2")
                
                # 3. 查詢位置資料
                from db_proxy.models import Location, Rack, Product, Carrier
                locations_data = []
                
                for location_name in location_names:
                    location = session.exec(
                        select(Location).where(Location.name == location_name)
                    ).first()
                    
                    if location:
                        location_info = {
                            "location": {
                                "id": location.id,
                                "name": location.name
                            },
                            "rack": None,
                            "product": None,
                            "carriers": []
                        }
                        
                        # 查詢該位置的料架
                        rack = session.exec(
                            select(Rack).where(Rack.location_id == location.id)
                        ).first()
                        
                        if rack:
                            location_info["rack"] = {
                                "id": rack.id,
                                "name": rack.name
                            }
                            
                            # 查詢產品資訊
                            if rack.product_id:
                                product = session.exec(
                                    select(Product).where(Product.id == rack.product_id)
                                ).first()
                                if product:
                                    location_info["product"] = {
                                        "id": product.id,
                                        "name": product.name,
                                        "size": product.size
                                    }
                            
                            # 查詢載具數量
                            carriers = session.exec(
                                select(Carrier).where(Carrier.rack_id == rack.id)
                            ).all()
                            location_info["carriers"] = [
                                {"id": c.id, "index": c.rack_index} for c in carriers
                            ]
                        
                        locations_data.append(location_info)
                
                # 4. 發送資料給 HMI
                response_data = {
                    'success': True,
                    'device_id': device_id,
                    'layout': layout,
                    'locations': locations_data
                }
                
                print(f"✅ 發送 HMI 資料: {len(locations_data)} 個位置")
                await self.sio.emit('hmi_data_update', response_data, to=sid)
                
        except Exception as e:
            print(f"❌ HMI 資料請求處理失敗: {e}")
            import traceback
            traceback.print_exc()
            await self.sio.emit('hmi_data_update', {
                'success': False,
                'message': f'Error: {str(e)}'
            }, to=sid)
