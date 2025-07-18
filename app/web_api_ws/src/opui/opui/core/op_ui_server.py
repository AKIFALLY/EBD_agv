import os
import socketio
import uvicorn

from fastapi import FastAPI, Request, HTTPException, Query
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.templating import Jinja2Templates
from opui.core.op_ui_socket import OpUiSocket
from opui.core.device_auth import check_device_authorization
from contextlib import asynccontextmanager

from opui.api import process_settings
from opui.api import product
from opui.api import license


class OpUiServer:
    def __init__(self, host="0.0.0.0", port=8002):
        self.host = host
        self.port = port

        # 初始化 Socket.IO
        self.sio = socketio.AsyncServer(
            async_mode="asgi",
            cors_allowed_origins="*",
            engineio_logger=True,
            logger=True)

        # 定義 FastAPI 的 lifespan，負責初始化與關閉 OpUiSocket
        @asynccontextmanager
        async def lifespan(app: FastAPI):
            self.op_ui_socket = OpUiSocket(self.sio)
            yield
            # 如果需要清理資源，可以在這裡添加

        # 建立 FastAPI 應用
        self.app = FastAPI(lifespan=lifespan)

        # 將 Socket.IO 整合進 ASGI App
        self.sio_app = socketio.ASGIApp(self.sio, self.app)

        # 啟用 CORS
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"])

        # 靜態資源設定
        base_dir = os.path.dirname(os.path.abspath(__file__))
        static_dir = os.path.join(base_dir, "..", "frontend", "static")
        templates_dir = os.path.join(base_dir, "..", "frontend", "templates")

        self.templates = Jinja2Templates(directory=templates_dir)
        self.app.mount(
            "/static", StaticFiles(directory=static_dir), name="static")

        # 註冊路由
        self.register_routes()

    def get_cors_allowed_origins(self):
        """取得 CORS 允許的來源"""
        cors_config = os.environ.get("CORS_ALLOWED_ORIGINS")
        if cors_config:
            return [cors_config]
        else:
            return ["*"]

    def register_routes(self):
        """註冊 HTTP 路由和 API 端點"""
        
        @self.app.get("/home", response_class=HTMLResponse)
        async def home(request: Request):
            try:
                # 檢查設備授權
                device_id = request.query_params.get("deviceId")
                if not device_id:
                    raise HTTPException(status_code=400, detail="缺少 deviceId 參數")

                auth_result = await check_device_authorization(device_id)
                if not auth_result["success"]:
                    raise HTTPException(
                        status_code=403,
                        detail=auth_result["message"]
                    )

                return self.templates.TemplateResponse("home.html", {
                    "request": request,
                    "device_id": device_id
                })
            except HTTPException:
                raise
            except Exception as e:
                print(f"Error in home route: {e}")
                return HTMLResponse(content="Error in home route", status_code=500)

        @self.app.get("/setting", response_class=HTMLResponse)
        async def setting(request: Request):
            try:
                # 檢查設備授權
                device_id = request.query_params.get("deviceId")
                if not device_id:
                    raise HTTPException(status_code=400, detail="缺少 deviceId 參數")

                auth_result = await check_device_authorization(device_id)
                if not auth_result["success"]:
                    raise HTTPException(
                        status_code=403,
                        detail=auth_result["message"]
                    )

                return self.templates.TemplateResponse("setting.html", {
                    "request": request,
                    "device_id": device_id
                })
            except HTTPException:
                raise
            except Exception as e:
                print(f"Error in setting route: {e}")
                return HTMLResponse(content="Error in settings route", status_code=500)

        @self.app.get("/rack", response_class=HTMLResponse)
        async def rack(request: Request):
            try:
                # 檢查設備授權
                device_id = request.query_params.get("deviceId")
                if not device_id:
                    raise HTTPException(status_code=400, detail="缺少 deviceId 參數")

                auth_result = await check_device_authorization(device_id)
                if not auth_result["success"]:
                    raise HTTPException(
                        status_code=403,
                        detail=auth_result["message"]
                    )

                return self.templates.TemplateResponse("rack.html", {
                    "request": request,
                    "device_id": device_id
                })
            except HTTPException:
                raise
            except Exception as e:
                print(f"Error in rack route: {e}")
                return HTMLResponse(content="Error in rack route", status_code=500)

        # 整合 API 路由器
        self.app.include_router(process_settings.router)
        self.app.include_router(product.router)
        self.app.include_router(license.router)

        # 新增 AGV 和任務相關 API
        from opui.api import agv
        self.app.include_router(agv.router, prefix="/api", tags=["agv", "tasks"])

    def run(self):
        """啟動伺服器"""
        try:
            uvicorn.run(self.sio_app, host=self.host, port=self.port)
        except Exception as e:
            print(f"Error running server: {e}")


def main():
    server = OpUiServer()
    server.run()


if __name__ == "__main__":
    main()
