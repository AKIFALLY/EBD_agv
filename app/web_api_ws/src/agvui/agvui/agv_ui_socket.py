import asyncio
from fastapi.encoders import jsonable_encoder


class AgvUiSocket:
    def __init__(self, sio):
        self.sio = sio
        self.init_socketio()

    def init_socketio(self):
        self.sio.on('connect')(self.connect)
        self.sio.on('disconnect')(self.disconnect)
        self.sio.on('send_to_backend')(self.send_to_backend)

    async def connect(self, sid, environ):
        print("🔌 使用者連線:", sid)

    async def disconnect(self, sid):
        print("❌ 使用者離線:", sid)


    async def send_to_backend(self, sid, data):
        print("這是從前端送來的訊息", sid,data)
        # 可以做一些處理，例如存入資料庫或發送給其他服務器
        # 然後將處理結果回傳給前端(可用return) 也可另外送notify
        # 傳送給前端的訊息格式建議是json格式
        # 例如：
        # return {"success": True, "message": "成功 這是會送回給前端的訊息"}
        # return {"success": False, "message": "處理失敗 這是會送回給前端的訊息"}
        return {"success": True, "message": "成功 這是會送回給前端的訊息"}

    async def notify_agv_status(self, agv_status_data):
        # print(f"通知 AGV 狀態更新:", agv_status_data)
        payload = jsonable_encoder({"agv_status": agv_status_data})
        await self.sio.emit("agv_status_update", payload)

    async def notify_message(self, sid, message):
        payload = jsonable_encoder({"message": message})
        await self.sio.emit("notify_message", payload, room=sid)

    async def error_message(self, sid, message):
        payload = jsonable_encoder({"message": message})
        await self.sio.emit("error_message", payload, room=sid)