import os
import socketio
import uvicorn
from datetime import datetime

from fastapi import FastAPI, Request, HTTPException, Query
from fastapi.responses import HTMLResponse, RedirectResponse, JSONResponse
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

        @self.app.get("/health")
        async def health_check():
            """健康檢查端點"""
            overall_status = "healthy"
            http_status_code = 200
            health_details = {
                "service": "opui",
                "port": 8002,
                "timestamp": datetime.now().isoformat()
            }

            try:
                # 檢查資料庫連接（如果有的話）
                db_status = "healthy"
                try:
                    from db_proxy.connection_pool_manager import ConnectionPoolManager
                    from sqlalchemy import text
                    pool = ConnectionPoolManager('postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
                    with pool.get_session() as session:
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
        async def root_dispatcher(request: Request):
            """主路由分發器 - 根據 device_type 導向不同介面"""
            try:
                # 1. 取得 deviceId
                device_id = request.query_params.get("deviceId")
                if not device_id:
                    return self.templates.TemplateResponse("error.html", {
                        "request": request,
                        "message": "缺少 deviceId 參數"
                    })
                
                # 2. 驗證並取得設備資訊
                auth_result = await check_device_authorization(device_id)
                if not auth_result["success"]:
                    return self.templates.TemplateResponse("unauthorized.html", {
                        "request": request,
                        "message": auth_result["message"]
                    })
                
                license_data = auth_result["license_data"]
                device_type = license_data.device_type
                
                # 3. 根據 device_type 導向不同介面
                if device_type == "op_station":
                    # 導向原本的 OPUI 介面
                    return RedirectResponse(url=f"/home?deviceId={device_id}")
                    
                elif device_type == "hmi_terminal":
                    # 導向 HMI 介面
                    return RedirectResponse(url=f"/hmi?deviceId={device_id}")
                    
                else:
                    # 未知類型
                    return self.templates.TemplateResponse("error.html", {
                        "request": request,
                        "message": f"不支援的設備類型: {device_type}"
                    })
                    
            except Exception as e:
                print(f"Error in root dispatcher: {e}")
                return self.templates.TemplateResponse("error.html", {
                    "request": request,
                    "message": "System error occurred"
                })
        
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
                
                # 檢查設備類型是否正確
                license_data = auth_result["license_data"]
                if license_data.device_type != "op_station":
                    # 重定向到根路由，讓根路由處理正確的導向
                    return RedirectResponse(url=f"/?deviceId={device_id}")

                return self.templates.TemplateResponse("home.html", {
                    "request": request,
                    "device_id": device_id
                })
            except HTTPException:
                raise
            except Exception as e:
                print(f"Error in home route: {e}")
                return self.templates.TemplateResponse("error.html", {
                    "request": request,
                    "message": "Error loading home page"
                })

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
                
                # 檢查設備類型是否正確
                license_data = auth_result["license_data"]
                if license_data.device_type != "op_station":
                    # 重定向到根路由，讓根路由處理正確的導向
                    return RedirectResponse(url=f"/?deviceId={device_id}")

                return self.templates.TemplateResponse("setting.html", {
                    "request": request,
                    "device_id": device_id
                })
            except HTTPException:
                raise
            except Exception as e:
                print(f"Error in setting route: {e}")
                return self.templates.TemplateResponse("error.html", {
                    "request": request,
                    "message": "Error loading settings page"
                })

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
                
                # 檢查設備類型是否正確
                license_data = auth_result["license_data"]
                if license_data.device_type != "op_station":
                    # 重定向到根路由，讓根路由處理正確的導向
                    return RedirectResponse(url=f"/?deviceId={device_id}")

                return self.templates.TemplateResponse("rack.html", {
                    "request": request,
                    "device_id": device_id
                })
            except HTTPException:
                raise
            except Exception as e:
                print(f"Error in rack route: {e}")
                return self.templates.TemplateResponse("error.html", {
                    "request": request,
                    "message": "Error loading rack page"
                })
        
        @self.app.get("/hmi", response_class=HTMLResponse)
        async def hmi(request: Request):
            """HMI 介面 - 根據 deviceId 顯示不同的 Location"""
            try:
                # 1. 驗證設備
                device_id = request.query_params.get("deviceId")
                if not device_id:
                    raise HTTPException(status_code=400, detail="缺少 deviceId 參數")
                
                auth_result = await check_device_authorization(device_id)
                if not auth_result["success"]:
                    raise HTTPException(
                        status_code=403,
                        detail=auth_result["message"]
                    )
                
                license_data = auth_result["license_data"]
                
                # 檢查設備類型是否正確
                if license_data.device_type != "hmi_terminal":
                    # 重定向到根路由，讓根路由處理正確的導向
                    return RedirectResponse(url=f"/?deviceId={device_id}")
                
                # 2. 從 permissions 欄位取得配置
                permissions = license_data.permissions or {}
                locations_to_monitor = permissions.get("locations", [])
                button_layout = permissions.get("layout", "1x2")
                
                # 3. 查詢每個 Location 的資料
                from opui.database.operations import connection_pool
                from db_proxy.crud.location_crud import location_crud
                from db_proxy.crud.rack_crud import rack_crud
                from sqlmodel import select
                from db_proxy.models import Location, Rack, Product
                
                locations_data = []
                with connection_pool.get_session() as session:
                    for loc_name in locations_to_monitor:
                        # 查詢 Location
                        location = session.exec(
                            select(Location).where(Location.name == loc_name)
                        ).first()
                        
                        if location:
                            rack = None
                            product = None

                            # 查詢該位置是否有 Rack
                            rack = session.exec(
                                select(Rack).where(Rack.location_id == location.id)
                            ).first()

                            # 查詢 Rack 關聯的產品資訊
                            if rack and rack.product_id:
                                product = session.exec(
                                    select(Product).where(Product.id == rack.product_id)
                                ).first()

                            locations_data.append({
                                "location": location,
                                "rack": rack,
                                "product": product
                            })
                        else:
                            # Location 不存在，加入空資料
                            locations_data.append({
                                "location": {"name": loc_name, "id": None},
                                "rack": None,
                                "product": None
                            })
                
                # 4. 渲染 HMI 模板
                return self.templates.TemplateResponse("hmi.html", {
                    "request": request,
                    "device_id": device_id,
                    "device_description": license_data.description,
                    "locations": locations_data,
                    "layout": button_layout
                })
                
            except HTTPException:
                raise
            except Exception as e:
                print(f"Error in HMI route: {e}")
                import traceback
                traceback.print_exc()
                return self.templates.TemplateResponse("error.html", {
                    "request": request,
                    "message": "Error loading HMI interface"
                })

        # 整合 API 路由器
        self.app.include_router(process_settings.router)
        self.app.include_router(product.router)
        self.app.include_router(license.router)

        # 新增 AGV 和任務相關 API
        from opui.api import agv
        self.app.include_router(agv.router, prefix="/api", tags=["agv", "tasks"])
        
        # 新增 HMI 相關 API
        from opui.api import hmi
        self.app.include_router(hmi.router)
        
        # 新增 Rack 相關 API
        from opui.api import rack
        self.app.include_router(rack.router)

    def run(self):
        """啟動伺服器"""
        try:
            uvicorn.run(self.sio_app, host=self.host, port=self.port)
        except Exception as e:
            print(f"Error running server: {e}")


def main():
    import signal
    import sys
    import logging
    
    # 設定 logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    logger = logging.getLogger(__name__)
    
    server = OpUiServer()
    
    def signal_handler(sig, frame):
        """處理 Ctrl+C 信號，優雅地關閉伺服器"""
        logger.info("\n📛 收到中斷信號 (Ctrl+C)，正在優雅地關閉伺服器...")
        try:
            # 如果有需要清理的資源，可以在這裡處理
            logger.info("✅ OPUI 伺服器已安全關閉")
        except Exception as e:
            logger.error(f"❌ 關閉時發生錯誤: {e}")
        finally:
            sys.exit(0)
    
    # 註冊信號處理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        logger.info(f"🚀 啟動 OPUI 伺服器在 {server.host}:{server.port}")
        server.run()
    except KeyboardInterrupt:
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
