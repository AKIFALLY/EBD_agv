import os
import asyncio
import uvicorn
import socketio
import json
from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from agvui.agv_ui_socket import AgvUiSocket
from agvui.agv_ui_ros import AgvUiRos


class AgvUiServer:
    def __init__(self, host="0.0.0.0", port=8003):
        self.host = host
        self.port = port
        self.loop = None
        self.local_agv_id = None
        self.container_type = None
        
        # 讀取本機身份
        self.load_local_identity()

        # 初始化 FastAPI 與 Socket.IO
        self.sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")
        self.app = FastAPI()
        self.sio_app = socketio.ASGIApp(self.sio, self.app)

        self.agv_ui_socket = AgvUiSocket(self.sio)

        # 設定 CORS
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        # 靜態檔與模板
        base_dir = os.path.dirname(__file__)
        static_dir = os.path.join(base_dir, "static")
        templates_dir = os.path.join(base_dir, "templates")

        self.app.mount("/static", StaticFiles(directory=static_dir), name="static")
        self.templates = Jinja2Templates(directory=templates_dir)

        # 註冊路由
        self._register_routes()

    def load_local_identity(self):
        """讀取本機 AGV 身份檔案"""
        try:
            # 先讀取 device_identity 確認容器類型
            device_identity_file = '/app/.device_identity'
            if os.path.exists(device_identity_file):
                with open(device_identity_file, 'r') as f:
                    for line in f:
                        if line.startswith('CONTAINER_TYPE='):
                            self.container_type = line.split('=')[1].strip()
                        elif line.startswith('DEVICE_ID='):
                            device_id = line.split('=')[1].strip()
            
            # 如果是 AGV 容器，讀取 .agv_identity
            if self.container_type == 'agv':
                agv_identity_file = '/app/.agv_identity'
                if os.path.exists(agv_identity_file):
                    with open(agv_identity_file, 'r') as f:
                        for line in f:
                            if line.startswith('AGV_ID='):
                                self.local_agv_id = line.split('=')[1].strip()
                                print(f"✅ 偵測到本機 AGV ID: {self.local_agv_id}")
                                break
            
            # 如果沒有 identity 檔案，嘗試從環境變數讀取
            if not self.local_agv_id:
                self.local_agv_id = os.environ.get('AGV_ID', None)
                if self.local_agv_id:
                    print(f"✅ 從環境變數讀取 AGV ID: {self.local_agv_id}")
            
            if not self.local_agv_id:
                print("⚠️ 無法偵測本機 AGV ID，將顯示所有 AGV 狀態")
                
        except Exception as e:
            print(f"❌ 讀取身份檔案錯誤: {e}")
            self.local_agv_id = None
    
    def _register_routes(self):
        @self.app.get("/", response_class=HTMLResponse)
        async def home(request: Request):
            # 支援 URL 參數覆寫 AGV ID (測試用)
            test_agv_id = request.query_params.get('agv_id', None)
            display_agv_id = test_agv_id if test_agv_id else self.local_agv_id
            
            # 如果有測試 ID，顯示測試模式
            is_test_mode = bool(test_agv_id)
            
            return self.templates.TemplateResponse("agv.html", {
                "request": request,
                "local_agv_id": display_agv_id,
                "container_type": self.container_type,
                "is_test_mode": is_test_mode
            })
        
        @self.app.get("/api/identity")
        async def get_identity(test_agv_id: str = None):
            """API endpoint to get local AGV identity"""
            return {
                "agv_id": test_agv_id if test_agv_id else self.local_agv_id,
                "container_type": self.container_type,
                "is_test_mode": bool(test_agv_id)
            }
        
        @self.app.get("/test", response_class=HTMLResponse)
        async def test_page(request: Request):
            """測試頁面 - 列出可測試的 AGV"""
            # 定義測試用 AGV 列表
            test_agvs = [
                {"id": "loader01", "type": "Loader AGV"},
                {"id": "loader02", "type": "Loader AGV"},
                {"id": "cargo01", "type": "Cargo Mover AGV"},
                {"id": "cargo02", "type": "Cargo Mover AGV"},
                {"id": "unloader01", "type": "Unloader AGV"},
                {"id": "unloader02", "type": "Unloader AGV"}
            ]
            
            # 使用模板系統而不是內嵌 HTML
            return self.templates.TemplateResponse("test.html", {
                "request": request,
                "test_agvs": test_agvs
            })

    async def read_status_file_task(self):
        """定時讀取 AGV 狀態檔案並透過 Socket.IO 廣播
        
        自動適應兩種部署模式：
        1. 單機模式（實際 AGV）：只讀取 /tmp/agv_status.json
        2. 多機模式（測試/中央監控）：讀取多個 /tmp/agv_status_*.json
        """
        # 測試環境的 AGV 列表
        agv_list = ["loader01", "loader02", "cargo01", "cargo02", "unloader01", "unloader02"]
        
        while True:
            try:
                files_found = False
                
                # 模式 1：檢查是否有多個 AGV 狀態檔案（測試/中央監控模式）
                for agv_id in agv_list:
                    status_file = f'/tmp/agv_status_{agv_id}.json'
                    if os.path.exists(status_file):
                        files_found = True
                        with open(status_file, 'r', encoding='utf-8') as f:
                            status_data = json.load(f)
                        
                        # 確保 AGV_ID 正確
                        status_data['AGV_ID'] = agv_id
                        status_data['agv_id'] = agv_id  # 相容舊格式
                        
                        # 透過 Socket.IO 廣播完整狀態
                        await self.agv_ui_socket.notify_agv_status(status_data)
                
                # 模式 2：如果沒有找到多機檔案，嘗試讀取單一檔案（實際 AGV 部署）
                if not files_found:
                    default_file = '/tmp/agv_status.json'
                    if os.path.exists(default_file):
                        with open(default_file, 'r', encoding='utf-8') as f:
                            status_data = json.load(f)
                        
                        # 如果有本機 AGV ID，確保資料中包含正確的 ID
                        if self.local_agv_id:
                            status_data['AGV_ID'] = self.local_agv_id
                            status_data['agv_id'] = self.local_agv_id
                        
                        await self.agv_ui_socket.notify_agv_status(status_data)
                        
                        # 記錄模式
                        if not hasattr(self, '_mode_logged'):
                            print(f"📍 單機模式：讀取 {default_file}")
                            if self.local_agv_id:
                                print(f"   本機 AGV ID: {self.local_agv_id}")
                            self._mode_logged = True
                else:
                    # 記錄模式
                    if not hasattr(self, '_mode_logged'):
                        print(f"📍 多機模式：讀取多個 AGV 狀態檔案")
                        self._mode_logged = True
                    
                # 每秒讀取一次
                await asyncio.sleep(1.0)
                
            except json.JSONDecodeError as e:
                print(f"⚠️ JSON 解析錯誤: {e}")
                await asyncio.sleep(1.0)
            except Exception as e:
                print(f"❌ 讀取狀態檔案錯誤: {e}")
                await asyncio.sleep(1.0)
    
    async def start(self):
        self.loop = asyncio.get_running_loop()
        # 啟動 ROS node（background thread）
        ros_node = AgvUiRos(self.loop, self.agv_ui_socket)
        ros_node.start()
        
        # 啟動狀態檔案讀取任務
        asyncio.create_task(self.read_status_file_task())
        print("✅ 已啟動 AGV 狀態檔案監控任務")

        # 啟動 uvicorn server
        config = uvicorn.Config(self.sio_app, host=self.host, port=self.port, loop="asyncio")
        server = uvicorn.Server(config)
        await server.serve()


# 👉 for ros2 run entry point 使用
def entry_point():
    asyncio.run(AgvUiServer().start())


# 👉 方便測試用：直接用 python 執行也可
if __name__ == "__main__":
    entry_point()
