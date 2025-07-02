import os
import signal
import uvicorn
import logging  # ✅ 加入 logging
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
from plc_proxy.plc_client_node import PlcClientNode
from ecs.door_controller_config import DoorControllerConfig
from ecs.door_logic import DoorLogic
from db_proxy.connection_pool_manager import ConnectionPoolManager
from traffic_manager.traffic_controller import TrafficController
from web_api.routers.plc import create_plc_router
from web_api.routers.traffic import create_traffic_router
from web_api.routers.door import create_door_router
from web_api.routers.map_importer import create_map_importer_router
from web_api.routers.kuka import create_kuka_router

# ✅ 設定 logging（可以放最上面）
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)  # logger 物件


class ApiServer:
    def __init__(self):
        self.app = FastAPI()
        self.plc_client = PlcClientNode('plc_client', 'agvc')
        self.door_config = DoorControllerConfig()
        self.door_config.load_config_yaml("/app/config/door_config.yaml")
        self.door_controller = DoorLogic(self.plc_client, self.door_config)

        # 還沒有實作
        self.db_pool = ConnectionPoolManager(
            'postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
        self.traffic_controller = TrafficController(self.db_pool)

        # 註冊 API 端點
        self.setup_routes()
        # 註冊 PLC API 端點
        self.app.include_router(create_plc_router(self.plc_client))
        # 註冊交管區 API 端點
        self.app.include_router(create_traffic_router(self.traffic_controller))
        # 註冊門控制 API 端點
        self.app.include_router(create_door_router(self.door_controller))
        # Map Importer API 端點
        self.app.include_router(create_map_importer_router(self.db_pool))
        # 註冊 Kuka API 端點
        self.app.include_router(create_kuka_router(self.db_pool))

    def setup_routes(self):
        """定義 API 端點"""
        @self.app.get("/shutdown")
        async def shutdown():
            """關閉伺服器"""
            os.kill(os.getpid(), signal.SIGINT)
            return {"message": "Server is shutting down..."}

    def run(self):
        """啟動 API 伺服器"""
#        uvicorn.run(self.app, host="0.0.0.0", port=8000)
        uvicorn.run(self.app, host="0.0.0.0", port=8000,
                    log_level="debug")  # ✅ 加 log_level

    def shutdown(self):
        """Shutdown the TrafficControllerClient"""
        # self.traffic_controller_client.shutdown()


def main():
    server = ApiServer()
    server.run()


if __name__ == "__main__":
    main()
