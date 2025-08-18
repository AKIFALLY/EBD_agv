"""
WCS Flow Designer 路由模組
提供流程設計器的 API 端點，包括流程文件的 CRUD 操作
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

from agvcui.database.flow_ops import FlowFileManager
from agvcui.middleware import get_current_user_from_request
from agvcui.utils.permissions import can_create, can_edit, can_delete


# Pydantic 模型定義
class FlowDefinition(BaseModel):
    id: str
    name: str
    description: Optional[str] = ""
    nodes: List[Dict[str, Any]] = []
    connections: List[Dict[str, Any]] = []
    metadata: Optional[Dict[str, Any]] = {}
    created_at: Optional[str] = None
    updated_at: Optional[str] = None


class FlowSummary(BaseModel):
    name: str
    description: Optional[str] = ""
    node_count: int
    connection_count: int
    created_at: Optional[str] = None
    updated_at: Optional[str] = None


# 文件操作輔助函數
class FlowFileOperations:
    @staticmethod
    def get_flows_dir():
        return Path("/app/config/wcs/flows")
    
    @staticmethod
    def get_flow_file_path(flow_name: str):
        flows_dir = FlowFileOperations.get_flows_dir()
        # 確保文件名安全
        safe_name = flow_name.replace("/", "_").replace("\\", "_")
        return flows_dir / f"{safe_name}.yaml"
    
    @staticmethod
    def load_flow_data(flow_name: str):
        flow_file = FlowFileOperations.get_flow_file_path(flow_name)
        if not flow_file.exists():
            return None
        
        try:
            with open(flow_file, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        except Exception:
            return None
    
    @staticmethod
    def save_flow_data(flow_name: str, flow_data: dict):
        flows_dir = FlowFileOperations.get_flows_dir()
        flows_dir.mkdir(parents=True, exist_ok=True)
        
        flow_file = FlowFileOperations.get_flow_file_path(flow_name)
        
        try:
            with open(flow_file, 'w', encoding='utf-8') as f:
                yaml.dump(flow_data, f, default_flow_style=False, 
                         allow_unicode=True, sort_keys=False)
            return True
        except Exception:
            return False
    
    @staticmethod
    def delete_flow_file(flow_name: str):
        flow_file = FlowFileOperations.get_flow_file_path(flow_name)
        if flow_file.exists():
            try:
                flow_file.unlink()
                return True
            except Exception:
                return False
        return False


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()
    flow_manager = FlowFileManager()

    @router.get("/flow-designer", response_class=HTMLResponse)
    async def flow_designer_page(request: Request):
        """
        Flow Designer 主頁面
        """
        current_user = get_current_user_from_request(request)
        
        return templates.TemplateResponse("flow_designer.html", {
            "request": request,
            "current_user": current_user,
            "page_title": "WCS Flow Designer"
        })

    @router.get("/flows", response_class=HTMLResponse)
    async def flows_page(request: Request):
        """
        Flows 管理頁面
        """
        current_user = get_current_user_from_request(request)
        
        return templates.TemplateResponse("flows.html", {
            "request": request,
            "current_user": current_user,
            "page_title": "流程管理"
        })

    @router.get("/api/flow-designer/flows", response_model=List[FlowSummary])
    async def list_flows(request: Request):
        """
        獲取所有流程文件列表
        """
        try:
            flows = flow_manager.list_flows()
            return [
                FlowSummary(
                    name=flow["name"],
                    description=flow.get("description", ""),
                    node_count=len(flow.get("nodes", [])),
                    connection_count=len(flow.get("connections", [])),
                    created_at=flow.get("metadata", {}).get("created_at"),
                    updated_at=flow.get("metadata", {}).get("updated_at")
                )
                for flow in flows
            ]
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"獲取流程列表失敗: {str(e)}")

    @router.get("/api/flow-designer/flows/{flow_name}", response_model=FlowDefinition)
    async def get_flow_by_name(request: Request, flow_name: str):
        """
        根據名稱獲取特定流程
        """
        try:
            flow_data = flow_manager.load_flow(flow_name)
            if not flow_data:
                raise HTTPException(status_code=404, detail=f"流程 '{flow_name}' 不存在")
            
            return FlowDefinition(**flow_data)
        except FileNotFoundError:
            raise HTTPException(status_code=404, detail=f"流程 '{flow_name}' 不存在")
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"載入流程失敗: {str(e)}")

    @router.post("/api/flow-designer/flows/{flow_name}")
    async def save_flow(request: Request, flow_name: str, flow_data: FlowDefinition):
        """
        保存流程定義
        """
        current_user = get_current_user_from_request(request)
        
        # 檢查權限
        if not can_create(request):
            raise HTTPException(status_code=403, detail="沒有創建權限")

        try:
            # 添加元數據
            flow_dict = flow_data.dict()
            flow_dict["name"] = flow_name
            
            # 設置時間戳
            now = datetime.now().isoformat()
            if "metadata" not in flow_dict:
                flow_dict["metadata"] = {}
            
            # 如果是新流程，設置創建時間
            existing_flow = flow_manager.load_flow(flow_name)
            if not existing_flow:
                flow_dict["metadata"]["created_at"] = now
                flow_dict["metadata"]["created_by"] = current_user.get("username", "unknown")
            
            flow_dict["metadata"]["updated_at"] = now
            flow_dict["metadata"]["updated_by"] = current_user.get("username", "unknown")

            # 保存流程
            success = flow_manager.save_flow(flow_name, flow_dict)
            
            if success:
                return JSONResponse(
                    content={"message": f"流程 '{flow_name}' 保存成功", "name": flow_name},
                    status_code=200
                )
            else:
                raise HTTPException(status_code=500, detail="保存失敗")

        except Exception as e:
            raise HTTPException(status_code=500, detail=f"保存流程失敗: {str(e)}")

    @router.delete("/api/flow-designer/flows/{flow_name}")
    async def delete_flow(request: Request, flow_name: str):
        """
        刪除流程
        """
        current_user = get_current_user_from_request(request)
        
        # 檢查權限
        if not can_delete(request):
            raise HTTPException(status_code=403, detail="沒有刪除權限")

        try:
            success = flow_manager.delete_flow(flow_name)
            
            if success:
                return JSONResponse(
                    content={"message": f"流程 '{flow_name}' 刪除成功"},
                    status_code=200
                )
            else:
                raise HTTPException(status_code=404, detail=f"流程 '{flow_name}' 不存在")

        except Exception as e:
            raise HTTPException(status_code=500, detail=f"刪除流程失敗: {str(e)}")

    @router.post("/api/flow-designer/flows/{flow_name}/duplicate")
    async def duplicate_flow(request: Request, flow_name: str, new_name: str):
        """
        複製流程
        """
        current_user = get_current_user_from_request(request)
        
        # 檢查權限
        if not can_create(request):
            raise HTTPException(status_code=403, detail="沒有創建權限")

        try:
            success = flow_manager.duplicate_flow(flow_name, new_name)
            
            if success:
                return JSONResponse(
                    content={"message": f"流程已複製為 '{new_name}'", "name": new_name},
                    status_code=200
                )
            else:
                raise HTTPException(status_code=404, detail=f"來源流程 '{flow_name}' 不存在")

        except Exception as e:
            raise HTTPException(status_code=500, detail=f"複製流程失敗: {str(e)}")

    @router.post("/api/flow-designer/import")
    async def import_flow(request: Request, file: UploadFile = File(...)):
        """
        匯入流程文件
        """
        current_user = get_current_user_from_request(request)
        
        # 檢查權限
        if not can_create(request):
            raise HTTPException(status_code=403, detail="沒有匯入權限")

        try:
            # 檢查文件類型
            if not file.filename.endswith('.json'):
                raise HTTPException(status_code=400, detail="只支援 JSON 格式文件")

            # 讀取文件內容
            content = await file.read()
            flow_data = json.loads(content.decode('utf-8'))

            # 驗證流程數據結構
            if not isinstance(flow_data, dict) or "name" not in flow_data:
                raise HTTPException(status_code=400, detail="無效的流程文件格式")

            flow_name = flow_data["name"]
            
            # 添加匯入元數據
            now = datetime.now().isoformat()
            if "metadata" not in flow_data:
                flow_data["metadata"] = {}
            
            flow_data["metadata"]["imported_at"] = now
            flow_data["metadata"]["imported_by"] = current_user.get("username", "unknown")
            flow_data["metadata"]["original_filename"] = file.filename

            # 保存流程
            success = flow_manager.save_flow(flow_name, flow_data)
            
            if success:
                return JSONResponse(
                    content={"message": f"流程 '{flow_name}' 匯入成功", "name": flow_name},
                    status_code=200
                )
            else:
                raise HTTPException(status_code=500, detail="匯入失敗")

        except json.JSONDecodeError:
            raise HTTPException(status_code=400, detail="JSON 格式錯誤")
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"匯入流程失敗: {str(e)}")

    @router.get("/api/flow-designer/flows/{flow_name}/export")
    async def export_flow(request: Request, flow_name: str):
        """
        匯出流程文件
        """
        try:
            flow_data = flow_manager.load_flow(flow_name)
            if not flow_data:
                raise HTTPException(status_code=404, detail=f"流程 '{flow_name}' 不存在")

            # 創建臨時文件
            temp_file = flow_manager.export_flow_to_file(flow_name)
            
            if temp_file and os.path.exists(temp_file):
                return FileResponse(
                    path=temp_file,
                    filename=f"{flow_name}.json",
                    media_type="application/json"
                )
            else:
                raise HTTPException(status_code=500, detail="匯出失敗")

        except Exception as e:
            raise HTTPException(status_code=500, detail=f"匯出流程失敗: {str(e)}")

    @router.get("/api/flow-designer/functions-config")
    async def get_functions_config(request: Request):
        """
        獲取函數配置
        """
        try:
            config = flow_manager.get_functions_config()
            return JSONResponse(content=config)
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"獲取函數配置失敗: {str(e)}")

    @router.post("/api/flow-designer/validate")
    async def validate_flow(request: Request, flow_data: FlowDefinition):
        """
        驗證流程定義
        """
        try:
            validation_result = flow_manager.validate_flow(flow_data.dict())
            return JSONResponse(content=validation_result)
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"驗證流程失敗: {str(e)}")

    @router.get("/api/flow-designer/templates")
    async def get_flow_templates(request: Request):
        """
        獲取流程模板
        """
        try:
            templates_list = flow_manager.get_templates()
            return JSONResponse(content=templates_list)
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"獲取模板失敗: {str(e)}")

    @router.get("/api/flow-designer/stats")
    async def get_flow_stats(request: Request):
        """
        獲取流程統計信息
        """
        try:
            stats = flow_manager.get_statistics()
            return JSONResponse(content=stats)
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"獲取統計信息失敗: {str(e)}")

    @router.get("/api/flows/list")
    async def list_flow_files(request: Request):
        """
        獲取所有流程配置文件
        """
        try:
            flows_dir = Path("/app/config/wcs/flows")
            flows = []
            
            if flows_dir.exists():
                for yaml_file in flows_dir.glob("*.yaml"):
                    try:
                        with open(yaml_file, 'r', encoding='utf-8') as f:
                            flow_data = yaml.safe_load(f)
                            
                        if flow_data:
                            flows.append({
                                "filename": yaml_file.name,
                                "name": flow_data.get("name", yaml_file.stem),
                                "description": flow_data.get("description", ""),
                                "priority": flow_data.get("priority", 0),
                                "work_id": flow_data.get("work_id", ""),
                                "enabled": flow_data.get("enabled", True),
                                "trigger_conditions_count": len(flow_data.get("trigger_conditions", [])),
                                "applicable_locations": flow_data.get("applicable_locations", [])
                            })
                    except Exception as e:
                        # 如果單個文件解析失敗，記錄但繼續處理其他文件
                        flows.append({
                            "filename": yaml_file.name,
                            "name": yaml_file.stem,
                            "description": f"解析錯誤: {str(e)}",
                            "priority": 0,
                            "work_id": "",
                            "enabled": False,
                            "trigger_conditions_count": 0,
                            "applicable_locations": [],
                            "error": True
                        })
            
            return JSONResponse(content={"flows": flows})
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"讀取流程文件失敗: {str(e)}")

    @router.get("/api/nodes/list")
    async def list_node_files(request: Request):
        """
        獲取所有節點配置文件
        """
        try:
            nodes_dir = Path("/app/config/wcs/nodes")
            all_nodes = {
                "action_nodes": [],
                "condition_nodes": [],
                "logic_nodes": []
            }
            
            if nodes_dir.exists():
                for node_type in ["action_nodes", "condition_nodes", "logic_nodes"]:
                    yaml_file = nodes_dir / f"{node_type}.yaml"
                    if yaml_file.exists():
                        try:
                            with open(yaml_file, 'r', encoding='utf-8') as f:
                                nodes_data = yaml.safe_load(f)
                            
                            if nodes_data and node_type in nodes_data:
                                nodes_list = []
                                for node_key, node_info in nodes_data[node_type].items():
                                    nodes_list.append({
                                        "key": node_key,
                                        "name": node_info.get("name", node_key),
                                        "description": node_info.get("description", ""),
                                        "category": node_info.get("category", ""),
                                        "icon": node_info.get("icon", ""),
                                        "color": node_info.get("color", ""),
                                        "inputs": node_info.get("inputs", []),
                                        "outputs": node_info.get("outputs", []),
                                        "parameters": node_info.get("parameters", []),
                                        "actions": node_info.get("actions", [])
                                    })
                                all_nodes[node_type] = nodes_list
                        except Exception as e:
                            # 如果文件解析失敗，返回錯誤信息
                            all_nodes[node_type] = [{
                                "key": "error",
                                "name": f"解析 {node_type} 失敗",
                                "description": str(e),
                                "category": "error",
                                "icon": "❌",
                                "color": "#ff0000",
                                "inputs": [],
                                "outputs": [],
                                "parameters": [],
                                "actions": [],
                                "error": True
                            }]
            
            return JSONResponse(content={"nodes": all_nodes})
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"讀取節點文件失敗: {str(e)}")

    @router.get("/flows/create", response_class=HTMLResponse)
    async def flow_create_form(request: Request):
        """
        新增流程表單頁面
        """
        try:
            print("DEBUG: Starting flow_create_form")
            current_user = get_current_user_from_request(request)
            print(f"DEBUG: current_user = {current_user}")
            
            # 檢查權限 - 但不阻止頁面顯示，而是在模板中處理
            print("DEBUG: About to check permissions")
            has_create_permission = can_create(request)
            print(f"DEBUG: can_create result = {has_create_permission}")
            
            # 如果沒有權限，重定向到登入頁面或顯示錯誤訊息
            if not has_create_permission:
                if not current_user:
                    # 沒有登入，重定向到登入頁面  
                    print("DEBUG: No user, redirecting to login")
                    return RedirectResponse(url="/login?redirect=/flows/create", status_code=302)
                else:
                    # 已登入但權限不足
                    print("DEBUG: User logged in but insufficient permissions")
                    raise HTTPException(status_code=403, detail="權限不足：需要操作員(operator)或管理員(admin)權限才能創建流程")
            
            print("DEBUG: About to render template")
            template_context = {
                "request": request,
                "current_user": current_user,
                "flow": None,  # 新增模式
                "form_title": "新增流程",
                "form_action": "/flows/create"
            }
            print(f"DEBUG: template_context = {template_context}")
            
            response = templates.TemplateResponse("flow_form.html", template_context)
            print("DEBUG: Template rendered successfully")
            return response
            
        except Exception as e:
            print(f"DEBUG: Exception in flow_create_form: {e}")
            import traceback
            print(f"DEBUG: Traceback: {traceback.format_exc()}")
            raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

    @router.post("/flows/create")
    async def flow_create(
        request: Request,
        name: str = Form(...),
        description: str = Form(""),
        priority: int = Form(50),
        work_id: str = Form(""),
        enabled: bool = Form(False),
        applicable_locations: str = Form("")
    ):
        """
        處理新增流程
        """
        current_user = get_current_user_from_request(request)
        
        # 檢查權限 - 處理未登入用戶
        has_create_permission = can_create(request)
        if not has_create_permission:
            if not current_user:
                # 沒有登入，重定向到登入頁面  
                return RedirectResponse(url="/login?redirect=/flows/create", status_code=302)
            else:
                # 已登入但權限不足
                raise HTTPException(status_code=403, detail="權限不足：需要操作員(operator)或管理員(admin)權限才能創建流程")

        try:
            # 檢查流程是否已存在
            if FlowFileOperations.load_flow_data(name):
                raise HTTPException(status_code=400, detail="流程名稱已存在")

            # 處理適用位置
            locations = []
            if applicable_locations.strip():
                try:
                    locations = [int(x.strip()) for x in applicable_locations.split(',') if x.strip().isdigit()]
                except ValueError:
                    raise HTTPException(status_code=400, detail="適用位置格式錯誤，請輸入數字並用逗號分隔")

            # 創建流程數據
            flow_data = {
                "name": name,
                "description": description,
                "priority": priority,
                "work_id": work_id if work_id else "",
                "enabled": enabled,
                "trigger_conditions": [],
                "action": {
                    "type": "create_task",
                    "task_type": "custom"
                },
                "applicable_locations": locations,
                "debug": {
                    "enabled": False,
                    "log_conditions": True,
                    "dry_run": False
                }
            }

            # 保存流程文件
            if FlowFileOperations.save_flow_data(name, flow_data):
                return RedirectResponse(url="/flows", status_code=303)
            else:
                raise HTTPException(status_code=500, detail="保存流程文件失敗")

        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"創建失敗: {str(e)}")

    @router.get("/flows/{filename}/edit")
    async def flow_edit_redirect(request: Request, filename: str):
        """
        編輯流程 - 重定向到 flow-designer (使用檔案名稱)
        """
        current_user = get_current_user_from_request(request)
        
        # 檢查權限 - 處理未登入用戶
        has_edit_permission = can_edit(request)
        if not has_edit_permission:
            if not current_user:
                # 沒有登入，重定向到登入頁面  
                return RedirectResponse(url=f"/login?redirect=/flows/{filename}/edit", status_code=302)
            else:
                # 已登入但權限不足
                raise HTTPException(status_code=403, detail="權限不足：需要操作員(operator)或管理員(admin)權限才能編輯流程")

        # 檢查流程檔案是否存在 
        flows_dir = Path("/app/config/wcs/flows")
        flow_file = flows_dir / filename
        
        if not flow_file.exists():
            raise HTTPException(status_code=404, detail="流程檔案不存在")
            
        # 使用檔案名稱而不是 YAML 中的 name 欄位來避免 URL 編碼問題
        flow_name = filename.replace(".yaml", "")
        
        # 重定向到 flow-designer 並傳入檔案名稱 (不需要 URL 編碼)
        return RedirectResponse(url=f"/flow-designer?flow={flow_name}", status_code=302)

    @router.post("/flows/{filename}/delete")
    async def flow_delete(request: Request, filename: str):
        """
        刪除流程 (使用檔案名稱)
        """
        current_user = get_current_user_from_request(request)
        
        # 檢查權限 - 處理未登入用戶
        has_delete_permission = can_delete(request)
        if not has_delete_permission:
            if not current_user:
                # 沒有登入，重定向到登入頁面  
                return RedirectResponse(url=f"/login?redirect=/flows", status_code=302)
            else:
                # 已登入但權限不足
                raise HTTPException(status_code=403, detail="權限不足：需要管理員(admin)權限才能刪除流程")

        try:
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

    return router