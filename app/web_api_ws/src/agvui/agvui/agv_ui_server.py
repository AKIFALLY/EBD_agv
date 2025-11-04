import os
import asyncio
import uvicorn
import socketio
import json
from datetime import datetime
from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.templating import Jinja2Templates
try:
    from agvui.agv_ui_socket import AgvUiSocket
    from agvui.agv_ui_ros import AgvUiRos
except ImportError:
    # Fallback for direct execution
    from agv_ui_socket import AgvUiSocket
    from agv_ui_ros import AgvUiRos


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

    def merge_status_data(self, new_data, legacy_data, agv_id):
        """合併新格式和舊格式資料，並標記來源

        Args:
            new_data: 新格式資料 (/tmp/agv_status_{agv_id}.json)
            legacy_data: 舊格式資料 (/tmp/agv_status.json)
            agv_id: AGV ID

        Returns:
            dict: 合併後的資料，包含 source_map
        """
        merged = {
            'metadata': {},
            'agv_status': {},
            'contexts': {},
            'type_specific': {},
            'door_status': {},
            'io_data': {},
            'alarms': {},
            'source_map': {}
        }

        # 輔助函數：清理字符串中的 NULL 填充字符
        def clean_string(s):
            """移除字符串末尾的 NULL 字符和其他填充字符"""
            if isinstance(s, str):
                # 移除 NULL 字符 (\u0000) 和其他控制字符
                return s.rstrip('\x00').strip()
            return s

        # 輔助函數：遞迴清理字典中的所有字符串
        def clean_dict_strings(d):
            """遞迴清理字典中的所有字符串值"""
            if isinstance(d, dict):
                return {k: clean_dict_strings(v) for k, v in d.items()}
            elif isinstance(d, list):
                return [clean_dict_strings(item) for item in d]
            elif isinstance(d, str):
                return clean_string(d)
            else:
                return d

        # 輔助函數：扁平化字典以建立 source_map
        def flatten_dict(d, parent_key='', sep='.'):
            items = []
            for k, v in d.items():
                new_key = f"{parent_key}{sep}{k}" if parent_key else k
                if isinstance(v, dict):
                    items.extend(flatten_dict(v, new_key, sep=sep).items())
                else:
                    items.append((new_key, v))
            return dict(items)

        # 輔助函數：深度合併字典
        def deep_merge(target, source, source_label, prefix=''):
            for key, value in source.items():
                field_path = f"{prefix}.{key}" if prefix else key

                if key not in target:
                    # 目標中沒有此欄位，直接加入
                    target[key] = value
                    merged['source_map'][field_path] = source_label
                elif isinstance(value, dict) and isinstance(target[key], dict):
                    # 兩者都是字典，遞迴合併
                    deep_merge(target[key], value, source_label, field_path)
                # 如果目標已有此欄位，保持原有值（新格式優先）

        # 1. 優先處理新格式資料（標記為 N）
        if new_data:
            for category in ['metadata', 'agv_status', 'contexts', 'type_specific', 'door_status', 'io_data', 'alarms']:
                if category in new_data:
                    merged[category] = new_data[category].copy() if isinstance(new_data[category], dict) else new_data[category]
                    # 標記所有新格式欄位
                    flattened = flatten_dict({category: merged[category]})
                    for field_path in flattened.keys():
                        merged['source_map'][field_path] = 'N'

        # 2. 用舊格式資料補足缺失欄位（標記為 L）
        if legacy_data:
            for category in ['metadata', 'agv_status', 'contexts', 'type_specific', 'door_status', 'io_data', 'alarms']:
                if category in legacy_data:
                    if category not in merged or not merged[category]:
                        # 類別完全缺失，使用舊格式
                        merged[category] = legacy_data[category].copy() if isinstance(legacy_data[category], dict) else legacy_data[category]
                        flattened = flatten_dict({category: merged[category]})
                        for field_path in flattened.keys():
                            merged['source_map'][field_path] = 'L'
                    elif isinstance(merged[category], dict) and isinstance(legacy_data[category], dict):
                        # 深度合併，補足缺失欄位
                        deep_merge(merged[category], legacy_data[category], 'L', category)

        # 3. 確保 AGV_ID 正確
        if 'metadata' not in merged:
            merged['metadata'] = {}
        merged['metadata']['AGV_ID'] = agv_id
        merged['AGV_ID'] = agv_id
        merged['agv_id'] = agv_id

        # 4. 統計資訊
        new_count = sum(1 for v in merged['source_map'].values() if v == 'N')
        legacy_count = sum(1 for v in merged['source_map'].values() if v == 'L')
        merged['source_stats'] = {
            'new_format_fields': new_count,
            'legacy_format_fields': legacy_count,
            'total_fields': new_count + legacy_count
        }

        # 5. 清理所有字符串中的 NULL 填充字符（來自 PLC/C++ 固定長度字符串）
        merged = clean_dict_strings(merged)

        return merged

    def _register_routes(self):
        @self.app.get("/health")
        async def health_check():
            """健康檢查端點"""
            overall_status = "healthy"
            http_status_code = 200
            health_details = {
                "service": "agvui",
                "port": 8003,
                "timestamp": datetime.now().isoformat(),
                "local_agv_id": self.local_agv_id,
                "container_type": self.container_type
            }

            try:
                # AGV UI 通常不需要資料庫連接
                # 但可以檢查 ROS 節點狀態或其他組件

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
        
        @self.app.get("/multi", response_class=HTMLResponse)
        async def multi_monitor(request: Request):
            """多車監控頁面 - 同時監控所有 AGV"""
            return self.templates.TemplateResponse("multi.html", {
                "request": request,
                "container_type": self.container_type
            })
        
        @self.app.get("/demo", response_class=HTMLResponse)
        async def demo_page(request: Request):
            """深色主題展示頁面 - 展示新的 UI 風格"""
            return self.templates.TemplateResponse("demo.html", {
                "request": request
            })
        
        @self.app.get("/api/agv-status/{agv_id}")
        async def get_agv_status(agv_id: str):
            """取得特定 AGV 的狀態資料"""
            try:
                # 從 /tmp/ 讀取對應的狀態檔案
                status_file = f'/tmp/agv_status_{agv_id}.json'
                if os.path.exists(status_file):
                    with open(status_file, 'r', encoding='utf-8') as f:
                        status_data = json.load(f)
                    
                    # 確保 AGV ID 正確
                    status_data['agv_id'] = agv_id
                    return status_data
                else:
                    # 檔案不存在，回傳空資料
                    return {"error": f"Status file not found for {agv_id}", "agv_id": agv_id}
                    
            except json.JSONDecodeError as e:
                return {"error": f"JSON parse error: {str(e)}", "agv_id": agv_id}
            except Exception as e:
                return {"error": str(e), "agv_id": agv_id}
        
        @self.app.get("/api/agv-plc/{agv_id}")
        async def get_agv_plc_status(agv_id: str):
            """取得特定 AGV 的完整 PLC 資料 (330+ 屬性)"""
            try:
                # 嘗試讀取 PLC 測試資料
                plc_file = f'/tmp/agv_plc_{agv_id}.json'
                if os.path.exists(plc_file):
                    with open(plc_file, 'r', encoding='utf-8') as f:
                        plc_data = json.load(f)
                    
                    # 確保 AGV ID 正確
                    plc_data['agv_id'] = agv_id
                    plc_data['AGV_ID'] = agv_id
                    
                    # 同時透過 Socket.IO 發送
                    await self.agv_ui_socket.notify_agv_status(plc_data)
                    
                    return plc_data
                else:
                    # 如果沒有 PLC 檔案，嘗試讀取普通狀態檔案
                    return await get_agv_status(agv_id)
                    
            except json.JSONDecodeError as e:
                return {"error": f"JSON parse error: {str(e)}", "agv_id": agv_id}
            except Exception as e:
                return {"error": str(e), "agv_id": agv_id}
        
        @self.app.get("/api/all-agv-status")
        async def get_all_agv_status():
            """取得所有 AGV 的狀態資料"""
            agv_list = ["loader01", "loader02", "cargo01", "cargo02", "unloader01", "unloader02"]
            all_status = {}
            
            for agv_id in agv_list:
                try:
                    status_file = f'/tmp/agv_status_{agv_id}.json'
                    if os.path.exists(status_file):
                        with open(status_file, 'r', encoding='utf-8') as f:
                            status_data = json.load(f)
                        status_data['agv_id'] = agv_id
                        all_status[agv_id] = status_data
                    else:
                        all_status[agv_id] = None
                except Exception as e:
                    print(f"Error reading status for {agv_id}: {e}")
                    all_status[agv_id] = None
            
            return all_status

    async def read_status_file_task(self):
        """定時讀取 AGV 狀態檔案並透過 Socket.IO 廣播

        整合新舊格式：
        1. 新格式（優先）：/tmp/agv_status_{agv_id}.json (Recorder Class, v2.0)
        2. 舊格式（補足）：/tmp/agv_status.json (Base Class, v1.0)

        合併策略：新格式優先，舊格式補足缺失欄位，並標記每個欄位的來源
        """
        # 測試環境的 AGV 列表
        agv_list = ["loader01", "loader02", "cargo01", "cargo02", "unloader01", "unloader02"]

        while True:
            try:
                files_found = False

                # 嘗試讀取舊格式檔案（用於補足缺失資料）
                legacy_data = None
                legacy_file = '/tmp/agv_status.json'
                if os.path.exists(legacy_file):
                    try:
                        with open(legacy_file, 'r', encoding='utf-8') as f:
                            legacy_data = json.load(f)
                    except json.JSONDecodeError as e:
                        print(f"⚠️ 舊格式 JSON 解析錯誤: {e}")
                        legacy_data = None

                # 模式 1：檢查是否有多個 AGV 狀態檔案（測試/中央監控模式）
                for agv_id in agv_list:
                    new_file = f'/tmp/agv_status_{agv_id}.json'
                    if os.path.exists(new_file):
                        # ✅ 過濾非本機 AGV：只處理本機 AGV 的數據
                        if self.local_agv_id and agv_id != self.local_agv_id:
                            continue  # 跳過非本機 AGV

                        files_found = True

                        # 讀取新格式資料
                        new_data = None
                        try:
                            with open(new_file, 'r', encoding='utf-8') as f:
                                new_data = json.load(f)
                        except json.JSONDecodeError as e:
                            print(f"⚠️ 新格式 JSON 解析錯誤 ({agv_id}): {e}")
                            new_data = None

                        # 合併新舊格式
                        merged_data = self.merge_status_data(new_data, legacy_data, agv_id)

                        # 透過 Socket.IO 廣播合併後的狀態
                        await self.agv_ui_socket.notify_agv_status(merged_data)

                        # 首次記錄模式
                        if not hasattr(self, '_mode_logged'):
                            print(f"📍 多機整合模式：讀取並合併新舊格式")
                            print(f"   - 新格式: {new_file}")
                            print(f"   - 舊格式: {legacy_file} (補足)")
                            self._mode_logged = True

                # 模式 2：如果沒有找到新格式檔案，使用舊格式（實際 AGV 部署）
                if not files_found and legacy_data:
                    # 確定使用哪個 AGV ID
                    target_agv_id = self.local_agv_id if self.local_agv_id else 'unknown'

                    # 即使只有舊格式，也透過 merge_status_data 處理以保持資料結構一致
                    merged_data = self.merge_status_data(None, legacy_data, target_agv_id)

                    await self.agv_ui_socket.notify_agv_status(merged_data)

                    # 記錄模式
                    if not hasattr(self, '_mode_logged'):
                        print(f"📍 單機模式：僅讀取舊格式")
                        print(f"   - 舊格式: {legacy_file}")
                        if self.local_agv_id:
                            print(f"   - 本機 AGV ID: {self.local_agv_id}")
                        self._mode_logged = True

                # 每秒讀取一次
                await asyncio.sleep(1.0)

            except json.JSONDecodeError as e:
                print(f"⚠️ JSON 解析錯誤: {e}")
                await asyncio.sleep(1.0)
            except Exception as e:
                print(f"❌ 讀取狀態檔案錯誤: {e}")
                import traceback
                traceback.print_exc()
                await asyncio.sleep(1.0)
    
    async def read_plc_file_task(self):
        """定時讀取 PLC 狀態檔案並透過 Socket.IO 廣播
        
        這是專門處理 PLC 完整資料 (330+ 屬性) 的任務
        """
        agv_list = ["loader01", "loader02", "cargo01", "cargo02", "unloader01", "unloader02"]
        
        while True:
            try:
                # 讀取 PLC 檔案
                for agv_id in agv_list:
                    plc_file = f'/tmp/agv_plc_{agv_id}.json'
                    if os.path.exists(plc_file):
                        with open(plc_file, 'r', encoding='utf-8') as f:
                            plc_data = json.load(f)
                        
                        # 確保 AGV ID 正確
                        plc_data['agv_id'] = agv_id
                        plc_data['AGV_ID'] = agv_id
                        
                        # 透過 Socket.IO 廣播 PLC 資料 (使用不同的事件名稱)
                        await self.agv_ui_socket.notify_plc_status(plc_data)
                
                # 每 2 秒讀取一次 (PLC 資料更新頻率較低)
                await asyncio.sleep(2.0)
                
            except json.JSONDecodeError as e:
                print(f"⚠️ PLC JSON 解析錯誤: {e}")
                await asyncio.sleep(2.0)
            except Exception as e:
                print(f"❌ 讀取 PLC 檔案錯誤: {e}")
                await asyncio.sleep(2.0)

    async def start(self):
        self.loop = asyncio.get_running_loop()
        # ✅ 已禁用：使用文件读取获取整合数据（带 N/L 标记）
        # 啟動 ROS node（background thread），傳遞 local_agv_id 作為命名空間
        # ros_node = AgvUiRos(self.loop, self.agv_ui_socket, self.local_agv_id)
        # ros_node.start()

        # 啟動狀態檔案讀取任務 (JSON 狀態) - 提供整合数据 + N/L 标记
        asyncio.create_task(self.read_status_file_task())
        print("✅ 已啟動 AGV 狀態檔案監控任務")
        
        # 啟動 PLC 檔案讀取任務 (PLC 完整資料)
        asyncio.create_task(self.read_plc_file_task())
        print("✅ 已啟動 PLC 狀態檔案監控任務")

        # 啟動 uvicorn server
        config = uvicorn.Config(self.sio_app, host=self.host, port=self.port, loop="asyncio")
        server = uvicorn.Server(config)
        await server.serve()


# 👉 for ros2 run entry point 使用
def entry_point():
    try:
        asyncio.run(AgvUiServer().start())
    except KeyboardInterrupt:
        print("\n⚠️ 接收到鍵盤中斷，正在關閉 AGVUI...")
    except Exception as e:
        print(f"❌ AGVUI 伺服器錯誤: {e}")
        import traceback
        traceback.print_exc()
        # 返回錯誤碼而非直接退出，避免容器終止
        return 1
    return 0


# 👉 方便測試用：直接用 python 執行也可
if __name__ == "__main__":
    entry_point()
