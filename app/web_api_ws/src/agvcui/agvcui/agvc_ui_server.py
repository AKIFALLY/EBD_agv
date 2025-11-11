import os
import socketio
import uvicorn
import logging
from datetime import datetime
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.templating import Jinja2Templates
from agvcui.agvc_ui_socket import AgvcUiSocket
from agvcui.routers import map, tasks, works, devices, signals
from agvcui.routers import rosout_logs, runtime_logs, audit_logs
from agvcui.routers import clients, racks, products, rooms, agvs, auth, users
from agvcui.routers import nodes, path_nodes, tafl_editor, tafl_editor_direct
from agvcui.middleware import AuthMiddleware
from contextlib import asynccontextmanager

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)


class AgvcUiServer:
    def __init__(self, host="0.0.0.0", port=8001):
        self.host = host
        self.port = port

        # 初始化 Socket.IO
        self.sio = socketio.AsyncServer(
            async_mode="asgi", cors_allowed_origins="*")

        # 定義 FastAPI 的 lifespan，負責初始化與關閉 AgvcUiSocket
        @asynccontextmanager
        async def lifespan(app: FastAPI):
            self.op_ui_socket = AgvcUiSocket(self.sio)
            yield
            await self.op_ui_socket.close()

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
            allow_headers=["*"],
        )

        # 添加認證中間件
        self.app.add_middleware(AuthMiddleware)

        # 設定靜態與模板目錄
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.templates = Jinja2Templates(
            directory=os.path.join(base_dir, "templates"),
            auto_reload=True  # Enable template auto-reload in development
        )

        # 添加自定義過濾器
        import json

        def tojson_chinese(obj, indent=None):
            """自定義 JSON 過濾器，支援中文字符顯示"""
            return json.dumps(obj, indent=indent, ensure_ascii=False)

        self.templates.env.filters['tojson_chinese'] = tojson_chinese

        self.app.mount(
            "/static", StaticFiles(directory=os.path.join(base_dir, "static")), name="static")

        # 註冊 API 路由
        self.register_routes()

    def register_routes(self):
        from agvcui.middleware import get_current_user_from_request

        @self.app.get("/health")
        async def health_check():
            """健康檢查端點"""
            overall_status = "healthy"
            http_status_code = 200
            health_details = {
                "service": "agvcui",
                "port": 8001,
                "timestamp": datetime.now().isoformat()
            }

            try:
                # 檢查資料庫連接（如果有的話）
                db_status = "healthy"
                try:
                    from agvcui.database import connection_pool
                    from sqlalchemy import text
                    with connection_pool.get_session() as session:
                        session.execute(text("SELECT 1"))
                    db_status = "healthy"
                except Exception as e:
                    db_status = f"unhealthy: {str(e)}"
                    overall_status = "degraded"  # 資料庫不健康時，服務狀態降級

                health_details["database"] = db_status

                # 檢查 Socket.IO 連接（如果需要）
                # 可以在這裡添加其他組件的健康檢查

                health_details["status"] = overall_status

                return JSONResponse(
                    status_code=http_status_code,
                    content=health_details
                )

            except Exception as e:
                # 發生未預期的錯誤時，回傳 503 Service Unavailable
                health_details["status"] = "unhealthy"
                health_details["error"] = str(e)

                return JSONResponse(
                    status_code=503,
                    content=health_details
                )

        @self.app.get("/", response_class=HTMLResponse)
        async def home(request: Request):
            current_user = get_current_user_from_request(request)

            # 查詢啟用的房間和製程資訊
            from agvcui.database import connection_pool
            from db_proxy.models import Room, ProcessSettings
            from sqlmodel import select

            enabled_rooms = []
            try:
                with connection_pool.get_session() as session:
                    # 查詢啟用的房間並關聯製程設定
                    stmt = select(Room, ProcessSettings).join(
                        ProcessSettings, Room.process_settings_id == ProcessSettings.id
                    ).where(Room.enable == 1).order_by(Room.id)

                    results = session.exec(stmt).all()

                    for room, process_settings in results:
                        enabled_rooms.append({
                            'id': room.id,
                            'name': room.name,
                            'description': room.description,
                            'soaking_times': process_settings.soaking_times,
                            'process_description': process_settings.description
                        })

            except Exception as e:
                print(f"❌ 查詢房間資料失敗: {e}")
                # 如果查詢失敗，提供預設的房間2資料
                enabled_rooms = [{
                    'id': 2,
                    'name': '房間2',
                    'description': '預設房間',
                    'soaking_times': 1,
                    'process_description': '標準製程'
                }]

            return self.templates.TemplateResponse("home.html", {
                "request": request,
                "active_tab": "home",
                "current_user": current_user,
                "enabled_rooms": enabled_rooms
            })

        @self.app.get("/setting", response_class=HTMLResponse)
        async def setting(request: Request):
            current_user = get_current_user_from_request(request)
            return self.templates.TemplateResponse("setting.html", {
                "request": request,
                "active_tab": "setting",
                "current_user": current_user
            })

        self.app.include_router(map.get_router(self.templates))
        self.app.include_router(tasks.get_router(self.templates))
        self.app.include_router(works.get_router(self.templates))
        self.app.include_router(rosout_logs.get_router(self.templates))
        self.app.include_router(runtime_logs.get_router(self.templates))
        self.app.include_router(audit_logs.get_router(self.templates))
        self.app.include_router(clients.get_router(self.templates))
        self.app.include_router(racks.get_router(self.templates))
        self.app.include_router(products.get_router(self.templates))
        self.app.include_router(rooms.get_router(self.templates))
        self.app.include_router(agvs.get_router(self.templates))
        self.app.include_router(devices.get_router(self.templates))
        self.app.include_router(signals.get_router(self.templates))
        self.app.include_router(auth.get_router(self.templates))
        self.app.include_router(users.get_router(self.templates))
        self.app.include_router(nodes.get_router(self.templates))

        # Add Path Nodes router (路徑節點管理)
        # 傳遞 socket 實例以支援地圖更新廣播
        self.app.include_router(path_nodes.create_path_nodes_router(self.sio))

        # Add TAFL Editor routers
        # tafl_editor provides the page route (/tafl/editor) and basic API
        self.app.include_router(tafl_editor.get_router(self.templates))
        # tafl_editor_direct provides enhanced API routes (Phase 3)
        # API routes from tafl_editor_direct will override the basic ones
        self.app.include_router(tafl_editor_direct.router)

    def run(self):
        uvicorn.run(self.sio_app, host=self.host, port=self.port)


def main():
    import signal
    import sys
    
    server = AgvcUiServer()
    
    def signal_handler(sig, frame):
        """處理 Ctrl+C 信號，優雅地關閉伺服器"""
        logger.info("\n📛 收到中斷信號 (Ctrl+C)，正在優雅地關閉伺服器...")
        try:
            # 如果有需要清理的資源，可以在這裡處理
            logger.info("✅ AGVCUI 伺服器已安全關閉")
        except Exception as e:
            logger.error(f"❌ 關閉時發生錯誤: {e}")
        finally:
            sys.exit(0)
    
    # 註冊信號處理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        logger.info(f"🚀 啟動 AGVCUI 伺服器在 {server.host}:{server.port}")
        server.run()
    except KeyboardInterrupt:
        # 這個應該不會被觸發，因為 signal handler 會先處理
        logger.info("\n⚠️ 接收到鍵盤中斷，正在關閉...")
    except Exception as e:
        logger.error(f"❌ 伺服器錯誤: {e}")
        import traceback
        traceback.print_exc()
        # 改為返回錯誤碼而非直接退出，避免容器終止
        # 這樣容器仍會保持運行，可以透過 SSH 查看錯誤日誌
        return 1


if __name__ == "__main__":
    main()
