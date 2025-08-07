"""
WCS Flow Designer 路由模組
提供流程設計器的 API 端點，包括流程文件的 CRUD 操作
支援標準流程格式 (FLOW_FORMAT_STANDARD.yaml)
"""

import json
import os
import yaml
from pathlib import Path
from typing import List, Dict, Any, Optional
from datetime import datetime

from fastapi import APIRouter, Request, HTTPException, UploadFile, File, Form
from fastapi.responses import HTMLResponse, JSONResponse, FileResponse, RedirectResponse
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel

from agvcui.middleware import get_current_user_from_request
from agvcui.utils.permissions import can_create, can_edit, can_delete
from agvcui.database.flow_ops import FlowFileManager

# 創建全局 router 和 flow_manager 實例
router = APIRouter()
flow_manager = FlowFileManager()

# 全局 templates 變數（將在應用啟動時設置）
_templates = None

def set_templates(tmpl: Jinja2Templates):
    """設置 templates 實例"""
    global _templates
    _templates = tmpl

def get_router(templates: Jinja2Templates = None) -> APIRouter:
    """獲取 router 實例（向後兼容）"""
    if templates:
        set_templates(templates)
    return router

# Flow 文件操作工具類
class FlowFileOperations:
    """統一的 Flow 文件操作接口"""
    
    @staticmethod
    def get_flows_dir():
        """獲取 flows 目錄路徑"""
        return Path("/app/config/wcs/flows")
    
    @staticmethod
    def get_flow_file_path(flow_name: str):
        """獲取流程文件的完整路徑"""
        flows_dir = FlowFileOperations.get_flows_dir()
        return flows_dir / f"{flow_name}.yaml"
    
    @staticmethod
    def load_flow_data(flow_name: str):
        """載入流程文件數據"""
        flow_file = FlowFileOperations.get_flow_file_path(flow_name)
        
        if flow_file.exists():
            try:
                with open(flow_file, 'r', encoding='utf-8') as f:
                    return yaml.safe_load(f)
            except Exception as e:
                print(f"載入流程文件失敗: {e}")
                return None
        return None
    
    @staticmethod
    def save_flow_data(flow_name: str, flow_data: dict):
        """保存流程文件數據"""
        flows_dir = FlowFileOperations.get_flows_dir()
        
        # 確保目錄存在
        flows_dir.mkdir(parents=True, exist_ok=True)
        
        flow_file = flows_dir / f"{flow_name}.yaml"
        
        try:
            with open(flow_file, 'w', encoding='utf-8') as f:
                yaml.dump(flow_data, f, default_flow_style=False, 
                         allow_unicode=True, sort_keys=False)
            return True
        except Exception as e:
            print(f"保存流程文件失敗: {e}")
            return False
    
    @staticmethod
    def delete_flow_file(flow_name: str):
        """刪除流程文件"""
        flow_file = FlowFileOperations.get_flow_file_path(flow_name)
        
        if flow_file.exists():
            try:
                flow_file.unlink()
                return True
            except Exception:
                return False
        return False

@router.get("/flow-designer", response_class=HTMLResponse)
async def flow_designer_page(request: Request, flow: str = None):
    """
    Flow Designer 主頁面
    支援 URL 參數載入指定流程: /flow-designer?flow=flow_name
    """
    current_user = get_current_user_from_request(request)
    
    # 開發模式：如果沒有用戶，使用測試用戶
    if not current_user:
        # 創建一個測試用戶物件（開發環境）
        class TestUser:
            def __init__(self):
                self.username = "admin"
                self.role = "admin"  # admin 角色有完整權限
        current_user = TestUser()
    
    # 使用全局 templates
    global _templates
    if not _templates:
        # 如果 templates 未初始化，嘗試創建一個默認的
        from pathlib import Path
        templates_dir = Path(__file__).parent.parent / "templates"
        _templates = Jinja2Templates(directory=str(templates_dir))
    
    # 檢查是否有指定要載入的流程
    flow_data = None
    if flow:
        try:
            # 使用 FlowFileOperations 來載入 YAML 格式的流程文件
            yaml_data = FlowFileOperations.load_flow_data(flow)
            if not yaml_data:
                # 流程不存在，記錄但不報錯，讓用戶在界面上處理
                print(f"⚠️ 指定的流程 '{flow}' 不存在")
            else:
                print(f"✅ 成功載入流程 '{flow}' 從 YAML 文件")
                # 傳遞完整的 YAML 資料，讓前端可以取得完整的類型定義
                flow_data = yaml_data
                print(f"   傳遞完整的 YAML 資料（包含主要 nodes 和 flow_designer_data）")
        except Exception as e:
            print(f"載入流程時發生錯誤: {e}")
            # 不拋出異常，讓用戶在界面上處理
    
    return _templates.TemplateResponse(
        "flow_designer.html", 
        {
            "request": request,
            "user": current_user,
            "flow_name": flow,
            "flow_data": json.dumps(flow_data) if flow_data else None
        }
    )

@router.get("/flows", response_class=HTMLResponse)
async def flows_page(request: Request):
    """Flows 管理頁面"""
    current_user = get_current_user_from_request(request)
    
    # 開發模式：總是使用測試用戶以啟用所有功能
    class TestUser:
        def __init__(self):
            self.username = "admin"
            self.role = "admin"  # admin 角色有完整權限
    
    # 開發環境中總是使用測試用戶
    current_user = TestUser()
    print(f"[Flow Designer] 使用測試用戶: {current_user.username} (role: {current_user.role})")
    
    # 使用全局 templates
    global _templates
    if not _templates:
        from pathlib import Path
        templates_dir = Path(__file__).parent.parent / "templates"
        _templates = Jinja2Templates(directory=str(templates_dir))
    
    return _templates.TemplateResponse(
        "flows.html",
        {
            "request": request,
            "user": current_user,
            "current_user": current_user  # 同時傳遞 current_user 變數
        }
    )

@router.get("/api/flows/list")
async def list_flows():
    """獲取所有流程列表"""
    try:
        flows = []
        flows_dir = Path("/app/config/wcs/flows")
        
        if flows_dir.exists():
            for flow_file in flows_dir.glob("*.yaml"):
                try:
                    with open(flow_file, 'r', encoding='utf-8') as f:
                        data = yaml.safe_load(f)
                        if data:
                            # 計算觸發條件數量 (nodes 的數量)
                            trigger_count = len(data.get('nodes', [])) if data.get('nodes') else 0
                            
                            flows.append({
                                'filename': flow_file.name,
                                'name': data.get('name', flow_file.stem),
                                'description': data.get('description', ''),
                                'enabled': data.get('enabled', False),
                                'work_id': data.get('work_id', ''),
                                'priority': data.get('priority', 0),
                                'trigger_conditions_count': trigger_count,
                                'applicable_locations': data.get('applicable_locations', [])
                            })
                except Exception as e:
                    print(f"Error loading flow {flow_file}: {e}")
                    pass
        
        return {"success": True, "flows": flows}
    except Exception as e:
        return {"success": False, "error": str(e)}

@router.get("/api/flows/{flow_name}")
async def get_flow(flow_name: str):
    """獲取特定流程"""
    try:
        flow_data = flow_manager.load_flow(flow_name)
        if flow_data:
            return {"success": True, "flow": flow_data}
        else:
            raise HTTPException(status_code=404, detail="流程不存在")
    except HTTPException:
        raise
    except Exception as e:
        return {"success": False, "error": str(e)}

@router.post("/api/flows/save")
async def save_flow(request: Request):
    """保存流程"""
    try:
        data = await request.json()
        flow_name = data.get('name')
        flow_data = data.get('data')
        
        if not flow_name or not flow_data:
            raise HTTPException(status_code=400, detail="缺少必要參數")
        
        flow_manager.save_flow(flow_name, flow_data)
        return {"success": True, "message": "流程保存成功"}
    except HTTPException:
        raise
    except Exception as e:
        return {"success": False, "error": str(e)}

@router.delete("/api/flows/{flow_name}")
async def delete_flow(flow_name: str):
    """刪除流程"""
    try:
        if flow_manager.delete_flow(flow_name):
            return {"success": True, "message": "流程刪除成功"}
        else:
            raise HTTPException(status_code=404, detail="流程不存在")
    except HTTPException:
        raise
    except Exception as e:
        return {"success": False, "error": str(e)}

@router.get("/api/flows/statistics")
async def get_statistics():
    """獲取流程統計信息"""
    try:
        stats = flow_manager.get_statistics()
        return {"success": True, "statistics": stats}
    except Exception as e:
        return {"success": False, "error": str(e)}

@router.get("/api/nodes/definitions")
async def get_node_definitions():
    """獲取所有可用的節點定義（從 WCS Functions 和 YAML 文件中提取）"""
    try:
        all_nodes = {}
        flows_dir = Path("/app/config/wcs/flows")
        
        # 1. 從 WCS Functions 產生節點定義
        try:
            from .generate_nodes_from_wcs import update_flow_designer_api
            wcs_nodes = update_flow_designer_api()
            all_nodes.update(wcs_nodes)
            print(f"載入了 {len(wcs_nodes)} 個 WCS 函數節點")
        except Exception as e:
            print(f"無法載入 WCS 函數節點: {e}")
        
        # 2. 從 YAML 文件中提取額外的節點定義
        if flows_dir.exists():
            for flow_file in flows_dir.glob("*.yaml"):
                try:
                    with open(flow_file, 'r', encoding='utf-8') as f:
                        data = yaml.safe_load(f)
                        if data and 'nodes' in data:
                            # 從節點中提取定義
                            for node in data['nodes']:
                                node_key = node.get('function') or node.get('id')
                                # 如果這個節點還沒有被 WCS 函數定義，才加入
                                if node_key and node_key not in all_nodes:
                                    all_nodes[node_key] = {
                                        'type': node.get('type', 'action'),
                                        'name': node.get('name', node_key),
                                        'description': node.get('description', ''),
                                        'function': node.get('function', ''),
                                        'inputs': node.get('inputs', {}),
                                        'outputs': node.get('outputs', {}),
                                        'parameters': node.get('parameters', {})
                                    }
                except Exception as e:
                    print(f"Error loading flow {flow_file}: {e}")
        
        # 返回所有節點定義
        return {"nodes": all_nodes, "count": len(all_nodes)}
    except Exception as e:
        return {"success": False, "error": str(e)}

@router.get("/api/nodes/list")
async def list_nodes():
    """獲取所有節點類型列表（預設節點）"""
    try:
        # 定義可用的節點類型
        nodes_data = {
            "action_nodes": [
                {
                    "name": "生成任務",
                    "description": "生成 AGV 執行任務",
                    "category": "action",
                    "icon": "📋",
                    "color": "#4CAF50",
                    "inputs": ["task_data"],
                    "outputs": ["task_id"],
                    "parameters": ["task_type", "priority", "agv_type"]
                },
                {
                    "name": "更新 Rack 狀態",
                    "description": "更新 Rack 的狀態信息",
                    "category": "action",
                    "icon": "🔄",
                    "color": "#2196F3",
                    "inputs": ["rack_id"],
                    "outputs": ["success"],
                    "parameters": ["status", "side"]
                },
                {
                    "name": "發送通知",
                    "description": "發送系統通知或警報",
                    "category": "action",
                    "icon": "🔔",
                    "color": "#FF9800",
                    "inputs": ["message"],
                    "outputs": [],
                    "parameters": ["notification_type", "recipients"]
                }
            ],
            "condition_nodes": [
                {
                    "name": "檢查 Rack 狀態",
                    "description": "檢查 Rack 的 A/B 面狀態",
                    "category": "condition",
                    "icon": "❓",
                    "color": "#9C27B0",
                    "inputs": ["rack_id"],
                    "outputs": ["is_complete", "has_carrier"],
                    "parameters": ["side", "check_type"]
                },
                {
                    "name": "取得位置列表",
                    "description": "根據類型取得位置列表",
                    "category": "condition",
                    "icon": "📍",
                    "color": "#00BCD4",
                    "inputs": [],
                    "outputs": ["locations", "no_locations"],
                    "parameters": ["location_type", "filter_has_rack"]
                },
                {
                    "name": "檢查進行中任務",
                    "description": "檢查是否有進行中的任務",
                    "category": "condition",
                    "icon": "⏳",
                    "color": "#FFC107",
                    "inputs": ["location"],
                    "outputs": ["has_pending", "no_pending"],
                    "parameters": ["task_type", "location_type"]
                }
            ],
            "logic_nodes": [
                {
                    "name": "循環處理",
                    "description": "對列表中的每個項目執行操作",
                    "category": "logic",
                    "icon": "🔁",
                    "color": "#795548",
                    "inputs": ["items"],
                    "outputs": ["current_item", "completed"],
                    "parameters": []
                },
                {
                    "name": "條件分支",
                    "description": "根據條件選擇不同的執行路徑",
                    "category": "logic",
                    "icon": "🔀",
                    "color": "#607D8B",
                    "inputs": ["condition"],
                    "outputs": ["true_path", "false_path"],
                    "parameters": []
                },
                {
                    "name": "並行執行",
                    "description": "同時執行多個操作",
                    "category": "logic",
                    "icon": "⚡",
                    "color": "#E91E63",
                    "inputs": ["trigger"],
                    "outputs": ["branch_1", "branch_2", "branch_3"],
                    "parameters": ["max_branches"]
                }
            ]
        }
        
        return {"success": True, "nodes": nodes_data}
    except Exception as e:
        return {"success": False, "error": str(e)}

@router.get("/flows/create", response_class=HTMLResponse)
async def create_flow_page(request: Request):
    """流程創建頁面"""
    current_user = get_current_user_from_request(request)
    
    # 開發模式：如果沒有用戶，使用測試用戶
    if not current_user:
        class TestUser:
            def __init__(self):
                self.username = "admin"
                self.role = "admin"
        current_user = TestUser()
    
    # 重導向到 Flow Designer 的新建模式
    return RedirectResponse(url="/flow-designer?mode=create", status_code=302)

@router.post("/flows/{filename}/delete")
async def delete_flow(request: Request, filename: str):
    """刪除流程 API (用於 flows 頁面的刪除按鈕)"""
    current_user = get_current_user_from_request(request)
    
    # 開發模式：如果沒有用戶，使用測試用戶
    if not current_user:
        class TestUser:
            def __init__(self):
                self.username = "admin"
                self.role = "admin"
        current_user = TestUser()
    
    # 權限檢查
    if current_user.role != 'admin':
        raise HTTPException(status_code=403, detail="無刪除權限")
    
    try:
        # 檢查流程檔案是否存在
        flows_dir = Path("/app/config/wcs/flows")
        flow_file = flows_dir / filename
        
        if not flow_file.exists():
            raise HTTPException(status_code=404, detail="流程檔案不存在")
        
        # 刪除流程檔案
        try:
            flow_file.unlink()
            # 重導向回 flows 頁面
            return RedirectResponse(url="/flows", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"刪除流程檔案失敗: {str(e)}")
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"刪除失敗: {str(e)}")

@router.post("/flow-designer/delete", response_class=HTMLResponse)
async def delete_flow_and_redirect(request: Request):
    """刪除流程並重導向"""
    current_user = get_current_user_from_request(request)
    
    # 權限檢查
    if not can_delete(current_user, 'flow'):
        raise HTTPException(status_code=403, detail="無刪除權限")
    
    try:
        form_data = await request.form()
        filename = form_data.get('filename')
        
        if not filename:
            raise HTTPException(status_code=400, detail="缺少檔案名")
        
        # 檢查流程檔案是否存在
        flows_dir = Path("/app/config/wcs/flows")
        flow_file = flows_dir / filename
        
        if not flow_file.exists():
            raise HTTPException(status_code=404, detail="流程檔案不存在")

        # 刪除流程檔案
        try:
            flow_file.unlink()
            return RedirectResponse(url="/flows", status_code=303)
        except Exception:
            raise HTTPException(status_code=500, detail="刪除流程檔案失敗")

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"刪除失敗: {str(e)}")