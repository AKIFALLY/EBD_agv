import os
import socketio
import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.templating import Jinja2Templates
from opui.op_ui_socket import OpUiSocket
from opui.routers import process_settings
from opui.routers import product


class OpUiServer:
    def __init__(self, host="0.0.0.0", port=8002):
        # Socket.IO AsyncServer
        self.sio = socketio.AsyncServer(
            async_mode="asgi",
            cors_allowed_origins="*")

        # FastAPI app
        self.app = FastAPI()

        # Socket.IO ASGI App 包裝 FastAPI
        self.sio_app = socketio.ASGIApp(self.sio, self.app)
        self.op_ui_socket = OpUiSocket(self.sio)

        # CORS 設定
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"])

        # 靜態資源設定
        base_dir = os.path.dirname(os.path.abspath(__file__))
        static_dir = os.path.join(base_dir, "static")
        templates_dir = os.path.join(base_dir, "templates")

        self.templates = Jinja2Templates(directory=templates_dir)
        self.app.mount(
            "/static", StaticFiles(directory=static_dir), name="static")

        # 註冊路由
        self.register_routes()

        # 監聽主機跟埠號
        self.host = host
        self.port = port

    def get_cors_allowed_origins(self):
        cors_config = os.environ.get("CORS_ALLOWED_ORIGINS")
        if cors_config:
            return [cors_config]
        else:
            return ["*"]

    def register_routes(self):
        @self.app.get("/", response_class=HTMLResponse)
        async def home(request: Request):
            try:
                return self.templates.TemplateResponse("home.html", {"request": request})
            except Exception as e:
                print(f"Error in home route: {e}")
                return HTMLResponse(content="Error in home route", status_code=500)

        @self.app.get("/setting", response_class=HTMLResponse)
        async def setting(request: Request):
            try:
                return self.templates.TemplateResponse("setting.html", {"request": request})
            except Exception as e:
                print(f"Error in setting route: {e}")
                return HTMLResponse(content="Error in setting route", status_code=500)

        self.app.include_router(process_settings.router)
        self.app.include_router(product.router)

    def run(self):
        try:
            uvicorn.run(self.sio_app, host=self.host, port=self.port)
        except Exception as e:
            print(f"Error running server: {e}")


def main():
    server = OpUiServer()
    server.run()


if __name__ == "__main__":
    main()
