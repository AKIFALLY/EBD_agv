import asyncio
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
        self.sio.on('connect')(self.connect)
        self.sio.on('disconnect')(self.disconnect)
        self.sio.on('login')(self.login)
        self.sio.on('client_update')(self.client_update)
        # self.sio.on('add_product')(self.add_product)
        self.sio.on('add_rack')(self.add_rack)
        self.sio.on('del_rack')(self.del_rack)
        self.sio.on('call_empty')(self.call_empty)
        self.sio.on('dispatch_full')(self.dispatch_full)
        self.sio.on('cancel_task')(self.cancel_task)  # 新增取消任務事件
        self.sio.on('confirm_delivery')(self.confirm_delivery)  # 新增確認送達事件
        self.sio.on('test_complete_task')(self.test_complete_task)  # 測試用：手動完成任務
        # 如果還有其他事件，這裡可以繼續綁定

    async def connect(self, sid, environ):
        print("🔌 使用者連線:", sid)
        # 確保任務監聽已啟動
        if not self.task_monitor.task_monitoring_started:
            self.task_monitor.start_monitoring()
            # 從資料庫恢復進行中的任務
            await self.task_monitor.restore_from_database()

        # 發送當前監聽的任務狀態給前端
        await self._sync_active_tasks_to_client(sid)

        # 發送當前監聽的任務狀態給前端
        await self._sync_active_tasks_to_client(sid)

    async def disconnect(self, sid):
        print("❌ 使用者離線:", sid)
        for clientId, s in list(self.user_sid_map.items()):
            if s == sid:
                del self.user_sid_map[clientId]

    async def notify_products(self, sid):
        products = product_all()
        payload = {"products": products}
        # print(products)
        await self.sio.emit("product_list", jsonable_encoder(payload), room=sid)

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

    async def login(self, sid, client):
        print(f"🔐 login 收到資料:")
        print(f"  sid: {sid}")
        print(f"  client: {client}")

        # 新的邏輯：優先使用前端提供的 clientId，如果沒有則使用 sid
        clientId = client.get("clientId") or sid
        client['clientId'] = clientId

        print(f"  最終使用的 clientId: {clientId}")

        # 查詢或建立 client 記錄
        db_client = get_or_create_or_update_client(client)

        self.user_sid_map[clientId] = sid
        client_dict = dict(db_client)  # 確保是 dict

        # 把 datetime 轉成 ISO 字串
        if client_dict.get("created_at"):
            client_dict["created_at"] = client_dict["created_at"].isoformat()
        if client_dict.get("updated_at"):
            client_dict["updated_at"] = client_dict["updated_at"].isoformat()

        await self.notify_products(sid)
        await self.notify_machines(sid)
        await self.notify_rooms(sid)
        await self.notify_parking_list(sid)

        print(f"✅ 登入成功，返回 clientId: {client_dict.get('clientId')}")

        # 登入成功，回傳 client 資訊（包含 clientId 供前端儲存）
        return {
            "success": True,
            "message": f"登入成功，clientId: {client_dict.get('clientId')}",
            "client": jsonable_encoder(db_client),
            "clientId": client_dict.get('clientId')  # 重要：返回 clientId 給前端
        }

    async def client_update(self, sid, data):
        clientId = data.get("clientId") or sid
        userAgent = data.get("userAgent") or ""
        op = data.get("op") or []
        machineId = data.get("machineId") or 1

        # 臨時啟用詳細日誌來除錯料架選擇問題
        print(f"🔄 client_update 收到資料:")
        print(f"  clientId: {clientId}")
        print(f"  machineId: {machineId}")
        print(f"  op: {op}")

        db_client = get_or_create_or_update_client({
            "clientId": clientId,
            "userAgent": userAgent,
            "op": op,
            "machineId": machineId,
        })

        print(f"✅ 資料庫更新完成，client.op: {db_client.get('op', {})}")

        self.user_sid_map[clientId] = sid
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

    def _get_client_and_machine_id(self, sid):
        clientId = None
        for cid, s in self.user_sid_map.items():
            if s == sid:
                clientId = cid
                break
        if not clientId:
            return None, None
        client = get_client({"clientId": clientId})
        machine_id = client.get("machineId")
        if not machine_id:
            return clientId, None
        return clientId, int(machine_id)

    def _update_machine_parking_status(self, machine_id, node_id, status=1):
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
        from opui.constants.parking_status import ParkingStatus

        if status == ParkingStatus.TASK_ACTIVE:
            return f"停車位 [{node_id}] 已叫車，請先取消"
        elif status == ParkingStatus.TASK_COMPLETED:
            return f"停車位 [{node_id}] 已送達，請先確認rack架已搬移"
        else:
            return f"停車位 [{node_id}] 狀態異常"

    def _require_client_and_machine(self, sid):
        clientId, machine_id = self._get_client_and_machine_id(sid)
        if not clientId:
            return None, None, {"success": False, "message": "找不到客戶端資訊"}
        if not machine_id:
            return clientId, None, {"success": False, "message": "請先選擇機台（machine_id 無效）"}
        return clientId, machine_id, None

    async def call_empty(self, sid, data):
        try:
            from opui.database.operations import create_task, get_call_empty_work_id, get_default_task_status_id

            # 獲取側邊和機台資訊
            side = data.get("side")  # "left" 或 "right"
            clientId, machine_id, err = self._require_client_and_machine(sid)
            if err:
                return err

            # 根據機台和側邊獲取正確的 node_id
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
                "status_id": get_default_task_status_id(),
                "node_id": node_id,
                "priority": 1,
                "parameters": {
                    "node_id": node_id,
                    "machine_id": machine_id,
                    "client_id": clientId,
                    "task_type": "call_empty"
                }
            }
            created_task = create_task(task_data)
            task_id = created_task['id']
            print(f"[callEmpty] 任務已創建: ID={task_id}, 名稱={created_task['name']}")

            # 開始監聽這個叫車任務
            self.task_monitor.add_task(task_id, machine_id, node_id,
                                       get_default_task_status_id(), "call_empty")

            self._update_machine_parking_status(machine_id, node_id, 1)
            await self.notify_machines(sid)
            return {"success": True, "message": f"叫車成功，任務 ID: {task_id}"}
        except Exception as e:
            print(f"[callEmpty] 錯誤: {str(e)}")
            return {"success": False, "message": f"叫車失敗: {str(e)}"}

    async def dispatch_full(self, sid, data):
        try:
            from opui.database.operations import create_task, get_dispatch_full_work_id, get_default_task_status_id

            # 獲取參數
            product_name = data.get("name")
            count = data.get("count")
            rack_id = data.get("rackId")
            room = data.get("room")
            side = data.get("side")
            clientId, machine_id, err = self._require_client_and_machine(sid)
            if err:
                return err

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
                "status_id": get_default_task_status_id(),
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
                                       get_default_task_status_id(), "dispatch_full")

            self._update_machine_parking_status(machine_id, node_id, 1)
            await self.notify_machines(sid)
            return {"success": True, "message": f"派車成功，任務 ID: {task_id}"}
        except Exception as e:
            print(f"[dispatchFull] 錯誤: {str(e)}")
            return {"success": False, "message": f"派車失敗: {str(e)}"}

    # async def add_product(self, sid, data):
    #    try:
    #        db_product = create_or_update_product(data)
    #        await self.notify_products(sid)
    #        return {
    #            "success": True,
    #            "message": "產品新增成功",
    #            "product": jsonable_encoder(db_product)
    #        }
    #    except Exception as e:
    #        return {"success": False, "message": f"產品新增失敗: {str(e)}"}

    def get_parking_list_by_machineId(self, machine_id):
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

    async def add_rack(self, sid, data):
        try:
            from opui.database.operations import connection_pool, rack_crud, location_crud, machine_crud
            from db_proxy.models import Rack
            side = data.get("side")
            rack_name = data.get("rack")
            if side is None or not rack_name:
                return {"success": False, "message": "缺少必要參數"}

            clientId, machine_id, err = self._require_client_and_machine(sid)
            if err:
                return err

            with connection_pool.get_session() as session:
                machine = machine_crud.get_by_id(session, machine_id)
                if not machine:
                    return {"success": False, "message": "找不到機台資訊"}

                location_id = machine.parking_space_1 if side == "left" else machine.parking_space_2
                if not location_id:
                    return {"success": False, "message": "該機台未設定對應停車格 location_id"}

                location = location_crud.get_by_id(session, location_id)
                if not location:
                    return {"success": False, "message": f"location_id {location_id} 不存在，請先建立 location"}

                exist_rack = rack_crud.get_by_field(session, "name", rack_name)
                if exist_rack:
                    # 檢查料架是否已經分配到其他停車格
                    if exist_rack.location_id and exist_rack.location_id != location_id:
                        return {"success": False, "message": f"料架 {rack_name} 已分配到其他停車格"}

                    # 更新料架的停車格位置
                    exist_rack.location_id = location_id
                    rack_crud.update(session, exist_rack.id, exist_rack)
                    rack_id = exist_rack.id
                    action = "分配到停車格"
                else:
                    # 料架不存在於資料表中，不允許新增
                    return {"success": False, "message": f"料架 {rack_name} 不存在於系統中，請先在料架管理中新增此料架"}

            await self.notify_parking_list(sid)
            return {"success": True, "message": f"料架 {rack_name} [{rack_id}] 已{action}成功"}
        except Exception as e:
            return {"success": False, "message": f"料架新增失敗: {str(e)}"}

    async def del_rack(self, sid, data):
        try:
            from opui.database.operations import connection_pool, rack_crud
            from db_proxy.models import Rack
            rack_id = int(data.get("rackId"))
            if rack_id is None or not rack_id:
                return {"success": False, "message": "缺少必要參數rackId"}

            clientId, machine_id, err = self._require_client_and_machine(sid)
            if err:
                return err

            with connection_pool.get_session() as session:
                rack = rack_crud.get_by_id(session, rack_id)
                if not rack:
                    return {"success": False, "message": "找不到Rack資訊"}

                rack.location_id = None
                rack_crud.update(session, rack_id, rack)
                await self.notify_parking_list(sid)

            return {"success": True, "message": "料架移除成功"}
        except Exception as e:
            return {"success": False, "message": f"料架移除失敗: {str(e)}"}

    async def notify_message(self, sid, message):
        payload = jsonable_encoder({"message": message})
        await self.sio.emit("notify_message", payload, room=sid)

    async def error_message(self, sid, message):
        payload = jsonable_encoder({"message": message})
        await self.sio.emit("error_message", payload, room=sid)

    async def notify_parking_list(self, sid):
        clientId, machine_id, err = self._require_client_and_machine(sid)
        if err:
            return err
        parking_list = self.get_parking_list_by_machineId(machine_id)
        await self.sio.emit("parking_list", parking_list, room=sid)
        return {"success": True}

    async def cancel_task(self, sid, data):
        """
        取消任務
        data: { side: "left"/"right", parkingSpace: <id> }
        """
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
            # return {"success": False, "message": "找不到對應的任務可取消"}
            # 任務可能已經由agvc刪除了
            print(
                f"[op_ui_socket.py] delete_task_by_parking({node_id}) failed")

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
        """
        確認rack架已送達並搬移至作業區
        data: { side: "left"/"right", parkingSpace: <id> }
        """
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

            # 檢查停車格狀態是否為已送達(2)
            from opui.database.operations import machine_crud, connection_pool
            with connection_pool.get_session() as session:
                machine = machine_crud.get_by_id(session, machine_id)
                if not machine:
                    return {"success": False, "message": "找不到機台資訊"}

                current_status = None
                if getattr(machine, 'parking_space_1', None) == node_id:
                    current_status = machine.parking_space_1_status
                elif getattr(machine, 'parking_space_2', None) == node_id:
                    current_status = machine.parking_space_2_status

                if current_status != 2:
                    return {"success": False, "message": f"停車位 [{node_id}] 狀態不正確，無法確認送達"}

            # 重設機台停車格狀態為未佔用(0)
            updated = self._update_machine_parking_status(machine_id, node_id, 0)
            if updated:
                await self.notify_machines(sid)
                await self.notify_parking_list(sid)
            await self.notify_message(sid, f"已確認停車位 [{node_id}] 的rack架已搬移至作業區")
            return {"success": True, "message": f"已確認停車位 [{node_id}] 的rack架已搬移至作業區"}
        except Exception as e:
            print(f"[confirm_delivery] 錯誤: {str(e)}")
            return {"success": False, "message": f"確認送達失敗: {str(e)}"}

    async def _handle_task_completion(self, task, task_info):
        """處理任務完成"""
        try:
            node_id = task.node_id
            machine_id = task_info['machine_id']
            task_type = task_info.get('task_type', 'call_empty')

            if task_type == 'call_empty':
                # 叫車任務完成：更新停車格狀態為已送達(2)
                updated = self._update_machine_parking_status(machine_id, node_id, 2)
                if updated:
                    await self._notify_clients_for_machine(machine_id)
                    # 叫車完成後發送最新的parking list，因為rack已送到停車格
                    await self._notify_parking_list_for_machine(machine_id)
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

    def _get_current_parking_status(self, machine_id, node_id):
        """獲取當前停車格狀態"""
        try:
            from opui.database.operations import machine_crud, connection_pool
            with connection_pool.get_session() as session:
                machine = machine_crud.get_by_id(session, machine_id)
                if machine:
                    if getattr(machine, 'parking_space_1', None) == node_id:
                        return machine.parking_space_1_status
                    elif getattr(machine, 'parking_space_2', None) == node_id:
                        return machine.parking_space_2_status
        except Exception as e:
            print(f"❌ 獲取停車格狀態失敗: {e}")
        return 0

    async def test_complete_task(self, sid, data):
        """
        測試用：手動完成任務
        data: { taskId: <id> }
        """
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
                        active_tasks[side] = {
                            'task_id': task_id,
                            'task_type': task_type,
                            'node_id': node_id,
                            'status': 'active'
                        }

            # 發送活躍任務狀態給前端
            if active_tasks:
                print(f"🔄 同步活躍任務狀態給客戶端 {sid}: {active_tasks}")
                await self.sio.emit("active_tasks", active_tasks, room=sid)
            else:
                print(f"🔄 客戶端 {sid} 沒有活躍任務")

        except Exception as e:
            print(f"❌ 同步活躍任務狀態失敗: {e}")

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
