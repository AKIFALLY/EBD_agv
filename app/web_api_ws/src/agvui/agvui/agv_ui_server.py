import os
import asyncio
import uvicorn
import socketio
from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from agvui.agv_ui_socket import AgvUiSocket
from agvui.agv_ui_ros import AgvUiRos


class AgvUiServer:
    def __init__(self, host="0.0.0.0", port=8001):
        self.host = host
        self.port = port
        self.loop = None

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

    def _register_routes(self):
        @self.app.get("/", response_class=HTMLResponse)
        async def home(request: Request):
            return self.templates.TemplateResponse("agv.html", {"request": request})

    async def start(self):
        self.loop = asyncio.get_running_loop()
        # 啟動 ROS node（background thread）
        ros_node = AgvUiRos(self.loop, self.agv_ui_socket)
        ros_node.start()

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
