#!/usr/bin/env python3
"""
Cargo AGV 即時監控系統 - FastAPI WebSocket 後端

功能:
- 讀取 AGV 狀態 JSON 文件 
- 透過 WebSocket 即時推送資料到前端
- 提供 Web 監控界面

使用:
python3 app.py
然後開啟瀏覽器訪問 http://localhost:8080
"""

import os
import json
import asyncio
import time
from pathlib import Path
from datetime import datetime
from typing import Optional, Dict, Any, List

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, FileResponse
import uvicorn

# FastAPI 應用程式設定
app = FastAPI(
    title="Cargo AGV Monitor",
    description="Cargo AGV 即時監控系統",
    version="1.0.0"
)

# 靜態文件和模板配置
app.mount("/static", StaticFiles(directory="static"), name="static")
templates = Jinja2Templates(directory="templates")

# Favicon 路由
@app.get("/favicon.ico", include_in_schema=False)
async def favicon():
    return FileResponse("static/favicon.ico")

# 配置設定
AGV_STATUS_DIR = "/home/ct/EBD_agv/app/agv_status_json"  # AGV 狀態文件目錄
AGV_STATUS_FILE = "current_status.json"  # 通用狀態文件名
UPDATE_INTERVAL = 1.0  # 更新間隔 (秒)
MAX_HISTORY = 100  # 最大歷史記錄數量

class AGVMonitor:
    """AGV 監控管理器"""
    
    def __init__(self):
        self.current_data: Optional[Dict[str, Any]] = None
        self.last_modified: Optional[float] = None
        self.connected_clients: List[WebSocket] = []
        self.status_history: List[Dict[str, Any]] = []
        
    async def load_agv_status(self) -> Optional[Dict[str, Any]]:
        """載入 AGV 狀態資料"""
        try:
            # 使用固定的狀態文件路徑
            status_file = os.path.join(AGV_STATUS_DIR, AGV_STATUS_FILE)
            
            if not os.path.exists(status_file):
                return {"error": f"找不到 AGV 狀態文件: {status_file}", "timestamp": datetime.now().isoformat()}
                
            # 檢查文件修改時間
            current_mtime = os.path.getmtime(status_file)
            
            # 檢查文件是否有更新
            if self.last_modified is None or current_mtime > self.last_modified:
                self.last_modified = current_mtime
                
                with open(status_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    
                # 添加文件資訊
                data["_file_info"] = {
                    "filepath": status_file,
                    "filename": os.path.basename(status_file),
                    "last_modified": datetime.fromtimestamp(current_mtime).isoformat(),
                    "file_size": os.path.getsize(status_file)
                }
                
                self.current_data = data
                
                # 更新歷史記錄
                self._update_history(data)
                
            return self.current_data
            
        except Exception as e:
            return {
                "error": f"載入 AGV 狀態失敗: {str(e)}",
                "timestamp": datetime.now().isoformat()
            }
    
    def _update_history(self, data: Dict[str, Any]):
        """更新狀態歷史記錄"""
        # 提取關鍵狀態資訊 (使用新的資料結構)
        history_entry = {
            "timestamp": data.get("metadata", {}).get("timestamp", datetime.now().isoformat()),
            "states": {
                "base": data.get("contexts", {}).get("base_context", {}).get("current_state"),
                "cargo": data.get("contexts", {}).get("cargo_context", {}).get("current_state"), 
                "robot": data.get("contexts", {}).get("robot_context", {}).get("current_state")
            },
            "agv_status": {
                "agv_id": data.get("metadata", {}).get("agv_id") or data.get("agv_status_complete", {}).get("AGV_ID"),
                "power": data.get("agv_status_complete", {}).get("POWER"),
                "auto": data.get("agv_status_complete", {}).get("AGV_Auto"),
                "moving": data.get("agv_status_complete", {}).get("AGV_MOVING"),
                "alarm": data.get("agv_status_complete", {}).get("AGV_ALARM")
            }
        }
        
        # 添加到歷史記錄
        self.status_history.append(history_entry)
        
        # 保持歷史記錄數量限制
        if len(self.status_history) > MAX_HISTORY:
            self.status_history = self.status_history[-MAX_HISTORY:]
    
    def get_status_summary(self) -> Dict[str, Any]:
        """取得狀態摘要"""
        if not self.current_data:
            return {"status": "no_data", "message": "無 AGV 狀態資料"}
            
        try:
            summary = {
                "timestamp": self.current_data.get("metadata", {}).get("timestamp"),
                "agv_type": self.current_data.get("metadata", {}).get("agv_type"),
                "agv_id": self.current_data.get("metadata", {}).get("agv_id"),
                "file_info": self.current_data.get("_file_info", {}),
                "contexts_count": len(self.current_data.get("contexts", {})),
                "agv_variables_count": len(self.current_data.get("agv_status_complete", {})),
                "agv_base_variables_count": len(self.current_data.get("agv_base_variables", {})),
                "history_count": len(self.status_history),
                "status": "connected",
                "display_mode": "with_base_variables"  # 標記為包含基礎變數模式
            }
            
            # 添加當前狀態 (使用新的 contexts 結構)
            contexts = self.current_data.get("contexts", {})
            summary["current_states"] = {
                "base": contexts.get("base_context", {}).get("current_state"),
                "cargo": contexts.get("cargo_context", {}).get("current_state"),
                "robot": contexts.get("robot_context", {}).get("current_state")
            }
            
            # 添加 AGV 狀態 (使用完整狀態)
            agv_status_complete = self.current_data.get("agv_status_complete", {})
            summary["agv_status"] = {
                "agv_id": self.current_data.get("metadata", {}).get("agv_id") or agv_status_complete.get("AGV_ID"),
                "power": agv_status_complete.get("POWER"),
                "auto": agv_status_complete.get("AGV_Auto"),
                "moving": agv_status_complete.get("AGV_MOVING"),
                "alarm": agv_status_complete.get("AGV_ALARM")
            }
            
            return summary
            
        except Exception as e:
            return {"status": "error", "message": f"處理狀態摘要失敗: {str(e)}"}
    
    async def add_client(self, websocket: WebSocket):
        """添加 WebSocket 客戶端"""
        await websocket.accept()
        self.connected_clients.append(websocket)
        print(f"🔗 新客戶端連接，目前連接數: {len(self.connected_clients)}")
        
        # 立即發送當前資料
        if self.current_data:
            try:
                await websocket.send_json({
                    "type": "full_data",
                    "data": self.current_data
                })
            except Exception as e:
                print(f"❌ 發送初始資料失敗: {e}")
    
    async def remove_client(self, websocket: WebSocket):
        """移除 WebSocket 客戶端"""
        if websocket in self.connected_clients:
            self.connected_clients.remove(websocket)
            print(f"🔌 客戶端斷開連接，目前連接數: {len(self.connected_clients)}")
    
    async def broadcast_update(self, message: Dict[str, Any]):
        """向所有客戶端廣播更新"""
        if not self.connected_clients:
            return
            
        disconnected_clients = []
        
        for client in self.connected_clients:
            try:
                await client.send_json(message)
            except Exception as e:
                print(f"❌ 廣播失敗: {e}")
                disconnected_clients.append(client)
        
        # 清理斷開的連接
        for client in disconnected_clients:
            await self.remove_client(client)

# 全域監控器實例
monitor = AGVMonitor()

@app.get("/", response_class=HTMLResponse)
async def root(request: Request):
    """主監控頁面"""
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/api/status")
async def get_status():
    """取得當前 AGV 狀態"""
    await monitor.load_agv_status()
    return monitor.current_data or {"error": "無法載入 AGV 狀態"}

@app.get("/api/summary")  
async def get_summary():
    """取得狀態摘要"""
    await monitor.load_agv_status()
    return monitor.get_status_summary()

@app.get("/api/history")
async def get_history():
    """取得狀態歷史"""
    return {
        "history": monitor.status_history[-20:],  # 返回最近20筆記錄
        "total_count": len(monitor.status_history)
    }

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket 連接處理"""
    await monitor.add_client(websocket)
    
    try:
        while True:
            # 保持連接並監聽客戶端訊息
            try:
                data = await asyncio.wait_for(websocket.receive_text(), timeout=1.0)
                # 處理客戶端訊息 (如果需要)
                print(f"📨 收到客戶端訊息: {data}")
            except asyncio.TimeoutError:
                # 超時是正常的，繼續執行
                pass
                
    except WebSocketDisconnect:
        await monitor.remove_client(websocket)
    except Exception as e:
        print(f"❌ WebSocket 錯誤: {e}")
        await monitor.remove_client(websocket)

async def background_updater():
    """背景資料更新任務"""
    print("🔄 背景更新任務已啟動")
    
    while True:
        try:
            # 載入最新資料
            old_data = monitor.current_data
            new_data = await monitor.load_agv_status()
            
            # 檢查是否有變更
            if new_data and new_data != old_data:
                # 廣播完整資料更新
                await monitor.broadcast_update({
                    "type": "full_data",
                    "data": new_data,
                    "summary": monitor.get_status_summary()
                })
                
            # 定期發送摘要更新
            summary = monitor.get_status_summary()
            await monitor.broadcast_update({
                "type": "summary",
                "data": summary
            })
            
        except Exception as e:
            print(f"❌ 背景更新錯誤: {e}")
            
        await asyncio.sleep(UPDATE_INTERVAL)

@app.on_event("startup")
async def startup_event():
    """應用程式啟動事件"""
    print("🚀 Cargo AGV 監控系統啟動中...")
    
    # 檢查狀態文件目錄
    if not os.path.exists(AGV_STATUS_DIR):
        print(f"⚠️ 狀態文件目錄不存在: {AGV_STATUS_DIR}")
        print("請確保 Cargo AGV 系統正在運行並產生狀態文件")
    else:
        print(f"✅ 狀態文件目錄: {AGV_STATUS_DIR}")
    
    # 啟動背景更新任務
    asyncio.create_task(background_updater())
    print("✅ 監控系統已啟動")

if __name__ == "__main__":
    print("🖥️ 啟動 Cargo AGV 監控系統...")
    print("📍 本地監控界面: http://localhost:8090")
    print("📍 網路監控界面: http://192.168.10.11:8090")
    print(f"📁 狀態文件目錄: {AGV_STATUS_DIR}")
    print("🔄 按 Ctrl+C 停止服務")
    
    uvicorn.run(
        "app:app",
        host="0.0.0.0",
        port=8090,
        reload=False,
        log_level="info"
    )