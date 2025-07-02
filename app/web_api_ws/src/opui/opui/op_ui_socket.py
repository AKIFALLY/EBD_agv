import asyncio
from fastapi.encoders import jsonable_encoder
from opui.db import get_or_create_or_update_client, get_client, create_or_update_product, product_all, machine_all, room_all


class OpUiSocket:
    def __init__(self, sio):
        self.sio = sio
        self.user_sid_map = {}  # clientId -> sid
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
        # 如果還有其他事件，這裡可以繼續綁定

    async def connect(self, sid, environ):
        print("🔌 使用者連線:", sid)
        # await self.sio.emit("server_message", "✅ 已連線", room=sid)
        # await self.notify_client_data(sid)

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

        clientId = client.get("clientId") or sid
        client['clientId'] = clientId
        # print(client.clientId)
        # 查詢 client
        db_client = get_client(client)

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
        # 登入成功，回傳 client 資訊
        return {
            "success": True,
            "message": f"登入成功，clientId: {client_dict.get('clientId')}",
            "client": jsonable_encoder(db_client),
            "clientId": client_dict.get('clientId')
        }

    async def client_update(self, sid, data):
        clientId = data.get("clientId") or sid
        userAgent = data.get("userAgent") or ""
        op = data.get("op") or []
        machineId = data.get("machineId") or 1
        print(f"op: {op}")

        # parking_list = self.get_parking_list_by_machineId(machineId)

        db_client = get_or_create_or_update_client({
            "clientId": clientId,
            "userAgent": userAgent,
            "op": op,
            "machineId": machineId,
        })

        self.user_sid_map[clientId] = sid
        client_dict = dict(db_client)  # 確保是 dict

        # await self.notify_client_data(sid)
        # await self.sio.emit("parking_list", parking_list, room=sid)
        # 更新停車列表(因為如果有切換機器時，停車列表會變更)
        await self.notify_parking_list(sid)

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
        from opui.db import machine_crud, connection_pool
        with connection_pool.get_session() as session:
            machine = machine_crud.get_by_id(session, machine_id)
            if machine:
                if getattr(machine, 'parking_space_1', None) == node_id:
                    machine.parking_space_1_status = status
                if getattr(machine, 'parking_space_2', None) == node_id:
                    machine.parking_space_2_status = status
                machine_crud.update(session, machine.id, machine)

    def _check_parking_space_status(self, machine_id, node_id):
        from opui.db import machine_crud, connection_pool
        with connection_pool.get_session() as session:
            machine = machine_crud.get_by_id(session, machine_id)
            if not machine:
                return False, "找不到機台資訊"
            if getattr(machine, 'parking_space_1', None) == node_id:
                if machine.parking_space_1_status == 1:
                    return False, f"停車位 [{node_id}] 已被佔用"
            if getattr(machine, 'parking_space_2', None) == node_id:
                if machine.parking_space_2_status == 1:
                    return False, f"停車位 [{node_id}] 已被佔用"
        return True, None

    def _require_client_and_machine(self, sid):
        clientId, machine_id = self._get_client_and_machine_id(sid)
        if not clientId:
            return None, None, {"success": False, "message": "找不到客戶端資訊"}
        if not machine_id:
            return clientId, None, {"success": False, "message": "請先選擇機台（machine_id 無效）"}
        return clientId, machine_id, None

    async def call_empty(self, sid, data):
        try:
            from opui.db import create_task, get_call_empty_work_id, get_default_task_status_id
            node_id = int(data.get("parkingSpace"))
            clientId, machine_id, err = self._require_client_and_machine(sid)
            if err:
                return err
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
            print(
                f"[callEmpty] 任務已創建: ID={created_task['id']}, 名稱={created_task['name']}")
            self._update_machine_parking_status(machine_id, node_id, 1)
            await self.notify_machines(sid)
            return {"success": True, "message": f"叫車成功，任務 ID: {created_task['id']}"}
        except Exception as e:
            print(f"[callEmpty] 錯誤: {str(e)}")
            return {"success": False, "message": f"叫車失敗: {str(e)}"}

    async def dispatch_full(self, sid, data):
        try:
            from opui.db import create_task, get_dispatch_full_work_id, get_default_task_status_id
            node_id = int(data.get("parkingSpace"))
            product_name = data.get("name")
            count = data.get("count")
            rack_id = data.get("rackId")
            room = data.get("room")
            side = data.get("side")
            clientId, machine_id, err = self._require_client_and_machine(sid)
            if err:
                return err
            ok, msg = self._check_parking_space_status(machine_id, node_id)
            if not ok:
                return {"success": False, "message": msg}
            # 準備任務資料
            task_data = {
                "name": f"派滿車 - {product_name} x{count} 到停車位 [{node_id}]",
                "description": f"操作員從機台 {machine_id} 派滿車，產品: {product_name}，數量: {count}，目標停車位: [{node_id}]",
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
            print(
                f"[dispatchFull] 任務已創建: ID={created_task['id']}, 名稱={created_task['name']}")
            self._update_machine_parking_status(machine_id, node_id, 1)
            await self.notify_machines(sid)
            return {"success": True, "message": f"派車成功，任務 ID: {created_task['id']}"}
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
        from opui.db import connection_pool, rack_crud, machine_crud
        parking_list = {"left": [], "right": []}
        with connection_pool.get_session() as session:
            machine = machine_crud.get_by_id(session, int(machine_id))

            if not machine:
                return parking_list
            if getattr(machine, 'parking_space_1', None):
                left_racks = [r for r in rack_crud.get_all(
                    session) if r.location_id == machine.parking_space_1]
                parking_list["left"] = [
                    {"id": r.id, "name": r.name} for r in left_racks]
            if getattr(machine, 'parking_space_2', None):
                right_racks = [r for r in rack_crud.get_all(
                    session) if r.location_id == machine.parking_space_2]
                parking_list["right"] = [
                    {"id": r.id, "name": r.name} for r in right_racks]
        return parking_list

    async def add_rack(self, sid, data):
        try:
            from opui.db import connection_pool, rack_crud, location_crud, machine_crud
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
                    exist_rack.location_id = location_id
                    rack_crud.update(session, exist_rack.id, exist_rack)
                    rack_id = exist_rack.id
                    action = "更新"
                else:
                    new_rack = Rack(name=rack_name, location_id=location_id)
                    rack_crud.create(session, new_rack)
                    rack_id = new_rack.id
                    action = "新增"

            await self.notify_parking_list(sid)
            return {"success": True, "message": f"料架 {rack_name} [{rack_id}] {action} 成功"}
        except Exception as e:
            return {"success": False, "message": f"料架新增失敗: {str(e)}"}

    async def del_rack(self, sid, data):
        try:
            from opui.db import connection_pool, rack_crud
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
        data: { parkingSpace: <id> }
        """
        from opui.db import delete_task_by_parking, connection_pool, machine_crud
        node_id = int(data.get("parkingSpace"))
        clientId, machine_id, err = self._require_client_and_machine(sid)
        if err:
            return err
        # 刪除任務
        deleted = delete_task_by_parking(node_id)
        if not deleted:
            # return {"success": False, "message": "找不到對應的任務可取消"}
            # 任務可能已經由agvc刪除了
            print(
                f"[op_ui_socket.py] delete_task_by_parking({node_id}) failed")

        # 重設機台停車格狀態
        self._update_machine_parking_status(machine_id, node_id, 0)
        await self.notify_machines(sid)
        await self.notify_parking_list(sid)
        await self.notify_message(sid, f"已取消停車位 [{node_id}] 的任務")
        return {"success": True, "message": f"已取消停車位 [{node_id}] 的任務"}
