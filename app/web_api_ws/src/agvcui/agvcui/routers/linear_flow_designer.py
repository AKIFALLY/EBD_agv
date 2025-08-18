"""
Linear Flow Designer Router
線性流程設計器路由模組 - 用於編輯和管理 Flow WCS v2 格式的流程
"""

import json
import yaml
from pathlib import Path
from typing import List, Dict, Any, Optional
from datetime import datetime

# Try to import httpx, but don't fail if it's not available
try:
    import httpx
    HTTPX_AVAILABLE = True
except ImportError:
    HTTPX_AVAILABLE = False
    print("Warning: httpx not available, will use fallback for API calls")

from fastapi import APIRouter, Request, HTTPException, Form
from fastapi.responses import HTMLResponse, JSONResponse, FileResponse, RedirectResponse
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel

# Create router
router = APIRouter(prefix="/linear-flow", tags=["linear_flow"])

# Templates instance (will be set from main app)
_templates = None

# Cache management
CACHE_FILE = Path("/app/config/wcs/flow_functions_cache.yaml")
_cache_updated_this_session = False  # Track if cache was updated in this session

def set_templates(tmpl: Jinja2Templates):
    """Set templates instance"""
    global _templates
    _templates = tmpl

def get_router(templates: Jinja2Templates = None) -> APIRouter:
    """Get router instance"""
    if templates:
        set_templates(templates)
    return router


def update_functions_cache(functions_data: Dict[str, Any]) -> None:
    """Update the functions cache file with defaults"""
    cache_data = {
        "meta": {
            "updated_at": datetime.now().isoformat(),
            "version": "2.0.0",
            "source": "flow_wcs_api"
        },
        "functions": functions_data
    }
    
    # Ensure directory exists
    CACHE_FILE.parent.mkdir(parents=True, exist_ok=True)
    
    # Write YAML cache
    with open(CACHE_FILE, 'w', encoding='utf-8') as f:
        yaml.dump(cache_data, f, default_flow_style=False, 
                  allow_unicode=True, sort_keys=False)
    
    print(f"✅ Functions cache updated at {CACHE_FILE}")


def load_functions_cache() -> Optional[Dict[str, Any]]:
    """Load functions from cache file"""
    if CACHE_FILE.exists():
        try:
            with open(CACHE_FILE, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                print(f"📦 Loaded functions from cache (updated: {data.get('meta', {}).get('updated_at', 'unknown')})")
                return data
        except Exception as e:
            print(f"❌ Failed to load cache: {e}")
    return None


class LinearFlowModel(BaseModel):
    """Linear Flow data model"""
    id: str
    name: str
    work_id: Optional[str] = None
    enabled: bool = True
    priority: int = 100
    workflow: List[Dict[str, Any]]
    config: Optional[Dict[str, Any]] = None
    variables: Optional[Dict[str, Any]] = None


class LinearFlowManager:
    """Manager for Linear Flow v2 format files"""
    
    def __init__(self):
        self.flows_dir = Path("/app/config/wcs/flows")
        self.ensure_directory()
    
    def ensure_directory(self):
        """Ensure flows directory exists"""
        self.flows_dir.mkdir(parents=True, exist_ok=True)
    
    def list_flows(self) -> List[Dict[str, Any]]:
        """List all linear flows"""
        flows = []
        
        if self.flows_dir.exists():
            for flow_file in self.flows_dir.glob("*.yaml"):
                try:
                    with open(flow_file, 'r', encoding='utf-8') as f:
                        data = yaml.safe_load(f)
                        
                        # 確保 data 是字典且包含正確的系統類型
                        if isinstance(data, dict) and data.get('meta', {}).get('system') == 'linear_flow_v2':
                            flow_info = data.get('flow', {})
                            flows.append({
                                'filename': flow_file.name,
                                'id': flow_info.get('id'),
                                'name': flow_info.get('name'),
                                'work_id': flow_info.get('work_id'),
                                'enabled': flow_info.get('enabled', True),
                                'priority': flow_info.get('priority', 100),
                                'description': data.get('meta', {}).get('description', ''),
                                'version': data.get('meta', {}).get('version', ''),
                                'sections': len(data.get('workflow', [])),
                                'modified': datetime.fromtimestamp(flow_file.stat().st_mtime).isoformat()
                            })
                except (yaml.YAMLError, TypeError) as e:
                    print(f"Error loading flow {flow_file}: {e}")
                except Exception as e:
                    print(f"Unexpected error loading flow {flow_file}: {e}")
        
        return flows
    
    def load_flow(self, flow_id: str) -> Optional[Dict[str, Any]]:
        """Load a specific flow"""
        for flow_file in self.flows_dir.glob("*.yaml"):
            try:
                with open(flow_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                    
                    # 確保 data 是字典且包含所需結構
                    if isinstance(data, dict) and data.get('flow', {}).get('id') == flow_id:
                        return data
            except (yaml.YAMLError, TypeError) as e:
                print(f"Error parsing flow file {flow_file}: {e}")
                continue
            except Exception as e:
                print(f"Unexpected error loading flow file {flow_file}: {e}")
                continue
        
        return None
    
    def save_flow(self, flow_id: str, flow_data: Dict[str, Any]) -> bool:
        """Save a flow"""
        try:
            # Ensure it's a v2 format
            if 'meta' not in flow_data:
                flow_data['meta'] = {}
            
            flow_data['meta']['system'] = 'linear_flow_v2'
            flow_data['meta']['version'] = flow_data['meta'].get('version', '2.0.0')
            flow_data['meta']['modified'] = datetime.now().isoformat()
            
            # Save to file
            file_path = self.flows_dir / f"{flow_id}.yaml"
            
            with open(file_path, 'w', encoding='utf-8') as f:
                yaml.dump(flow_data, f, default_flow_style=False, 
                         allow_unicode=True, sort_keys=False)
            
            return True
            
        except Exception as e:
            print(f"Error saving flow {flow_id}: {e}")
            return False
    
    def delete_flow(self, flow_id: str) -> bool:
        """Delete a flow"""
        try:
            for flow_file in self.flows_dir.glob("*.yaml"):
                try:
                    with open(flow_file, 'r', encoding='utf-8') as f:
                        data = yaml.safe_load(f)
                        
                        # 確保 data 是字典且包含所需結構
                        if isinstance(data, dict) and data.get('flow', {}).get('id') == flow_id:
                            flow_file.unlink()
                            return True
                except (yaml.YAMLError, TypeError) as e:
                    # 跳過無法解析的檔案
                    print(f"Skipping invalid file {flow_file}: {e}")
                    continue
            
            return False
            
        except Exception as e:
            print(f"Error deleting flow {flow_id}: {e}")
            return False
    
    def validate_flow(self, flow_data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate flow structure"""
        errors = []
        warnings = []
        
        # Check required fields
        if 'meta' not in flow_data:
            errors.append("Missing 'meta' section")
        elif flow_data['meta'].get('system') != 'linear_flow_v2':
            errors.append("Invalid system type (must be 'linear_flow_v2')")
        
        if 'flow' not in flow_data:
            errors.append("Missing 'flow' section")
        elif 'id' not in flow_data['flow']:
            errors.append("Missing flow ID")
        
        if 'workflow' not in flow_data:
            errors.append("Missing 'workflow' section")
        elif not isinstance(flow_data['workflow'], list):
            errors.append("Workflow must be a list of sections")
        else:
            # Validate workflow sections
            for i, section in enumerate(flow_data['workflow']):
                if 'section' not in section:
                    warnings.append(f"Section {i+1} missing name")
                
                if 'steps' not in section or not section['steps']:
                    warnings.append(f"Section {i+1} has no steps")
                else:
                    # Validate steps
                    for j, step in enumerate(section['steps']):
                        if 'id' not in step:
                            warnings.append(f"Step {j+1} in section {i+1} missing ID")
                        if 'exec' not in step:
                            warnings.append(f"Step {j+1} in section {i+1} missing exec command")
        
        return {
            'valid': len(errors) == 0,
            'errors': errors,
            'warnings': warnings
        }


# Create global manager instance
flow_manager = LinearFlowManager()


@router.get("/", response_class=RedirectResponse)
async def linear_flow_root():
    """Redirect root to list page"""
    return RedirectResponse(url="/linear-flow/list", status_code=302)


@router.get("/designer", response_class=HTMLResponse)
async def linear_flow_designer_page(request: Request, flow: str = None):
    """Linear Flow Designer main page"""
    global _templates
    
    if not _templates:
        from pathlib import Path
        templates_dir = Path(__file__).parent.parent / "templates"
        _templates = Jinja2Templates(directory=str(templates_dir))
    
    # Load flow if specified
    flow_data = None
    if flow:
        flow_data = flow_manager.load_flow(flow)
    
    return _templates.TemplateResponse(
        "linear_flow_designer.html",
        {
            "request": request,
            "flow_id": flow,
            "flow_data": json.dumps(flow_data) if flow_data else None
        }
    )


@router.get("/list", response_class=HTMLResponse)
async def linear_flows_list_page(request: Request):
    """Linear Flows list page"""
    global _templates
    
    if not _templates:
        from pathlib import Path
        templates_dir = Path(__file__).parent.parent / "templates"
        _templates = Jinja2Templates(directory=str(templates_dir))
    
    return _templates.TemplateResponse(
        "linear_flows.html",
        {"request": request}
    )


@router.get("/api/flows")
async def list_linear_flows():
    """API: List all linear flows"""
    try:
        flows = flow_manager.list_flows()
        return {"success": True, "flows": flows}
    except Exception as e:
        return {"success": False, "error": str(e)}


@router.get("/api/flows/{flow_id}")
async def get_linear_flow(flow_id: str):
    """API: Get a specific flow"""
    try:
        flow_data = flow_manager.load_flow(flow_id)
        
        if flow_data:
            return {"success": True, "flow": flow_data}
        else:
            raise HTTPException(status_code=404, detail="Flow not found")
            
    except HTTPException:
        raise
    except Exception as e:
        return {"success": False, "error": str(e)}


@router.post("/api/flows/save")
async def save_linear_flow(request: Request):
    """API: Save a flow"""
    try:
        data = await request.json()
        flow_id = data.get('flow_id')
        flow_data = data.get('flow_data')
        
        if not flow_id or not flow_data:
            raise HTTPException(status_code=400, detail="Missing required parameters")
        
        # Validate flow
        validation = flow_manager.validate_flow(flow_data)
        if not validation['valid']:
            return {
                "success": False,
                "errors": validation['errors'],
                "warnings": validation['warnings']
            }
        
        # Save flow
        success = flow_manager.save_flow(flow_id, flow_data)
        
        if success:
            return {
                "success": True,
                "message": "Flow saved successfully",
                "warnings": validation['warnings']
            }
        else:
            return {"success": False, "error": "Failed to save flow"}
            
    except HTTPException:
        raise
    except Exception as e:
        return {"success": False, "error": str(e)}


@router.delete("/api/flows/{flow_id}")
async def delete_linear_flow(flow_id: str):
    """API: Delete a flow"""
    try:
        success = flow_manager.delete_flow(flow_id)
        
        if success:
            return {"success": True, "message": "Flow deleted successfully"}
        else:
            raise HTTPException(status_code=404, detail="Flow not found")
            
    except HTTPException:
        raise
    except Exception as e:
        return {"success": False, "error": str(e)}


@router.post("/api/flows/validate")
async def validate_linear_flow(request: Request):
    """API: Validate a flow"""
    try:
        data = await request.json()
        flow_data = data.get('flow_data')
        
        if not flow_data:
            raise HTTPException(status_code=400, detail="Missing flow data")
        
        validation = flow_manager.validate_flow(flow_data)
        
        return {
            "success": True,
            "valid": validation['valid'],
            "errors": validation['errors'],
            "warnings": validation['warnings']
        }
        
    except HTTPException:
        raise
    except Exception as e:
        return {"success": False, "error": str(e)}


@router.get("/api/functions")
async def get_available_functions(request: Request, source: str = "flow_wcs"):
    """API: Get available functions for flows with 3-layer fallback
    
    Priority:
    1. Live API from flow_wcs (with defaults)
    2. Cache YAML (when API fails)
    3. Static fallback (last resort)
    
    Args:
        source: Function source:
                - 'flow_wcs' (default): Try API first, then cache, then static
                - 'cache': Load from cache only
                - 'static': Use static functions only
                - 'config': Alias for 'static' (for compatibility)
    
    Note: Always returns categorized format. Frontend creates index for O(1) lookup.
    """
    global _cache_updated_this_session
    
    # Handle 'config' as an alias for 'static' (compatibility)
    if source == "config":
        source = "static"
    
    # Try flow_wcs API first (Layer 1: Live API)
    if source == "flow_wcs" and HTTPX_AVAILABLE:
        try:
            async with httpx.AsyncClient(timeout=5.0) as client:
                # Construct the correct API URL based on the request host
                # When accessed via agvc.ui (8001), we need to call agvc.webapi (8000) 
                # When accessed via localhost:8001, we call localhost:8000
                host = request.headers.get("host", "localhost:8001")
                
                # Map the UI host to the API host
                if "agvc.ui" in host:
                    api_url = "http://agvc.webapi/api/flow/functions"
                elif "localhost:8001" in host or ":8001" in host:
                    api_url = "http://localhost:8000/api/flow/functions"
                else:
                    # Default fallback - use the same host but port 8000
                    base_host = host.split(':')[0]
                    api_url = f"http://{base_host}:8000/api/flow/functions"
                
                response = await client.get(api_url)
                if response.status_code == 200:
                    data = response.json()
                    if data.get("success"):
                        functions_data = data["functions"]
                        
                        # Update cache only once per session
                        if not _cache_updated_this_session:
                            update_functions_cache(functions_data)
                            _cache_updated_this_session = True
                            print("✅ Cache updated for this session")
                        
                        # Always return categorized format
                        return {
                            "success": True,
                            "functions": functions_data,
                            "source": "flow_wcs_live",
                            "format": "categorized",
                            "cached_at": datetime.now().isoformat()
                        }
        except Exception as e:
            print(f"⚠️ API call failed: {e}, falling back to cache")
    
    # Try cache (Layer 2: Cache YAML)
    if source in ["flow_wcs", "cache"]:
        cached_data = load_functions_cache()
        if cached_data:
            functions_data = cached_data.get("functions", {})
            
            # Always return categorized format
            return {
                "success": True,
                "functions": functions_data,
                "source": "cache",
                "format": "categorized",
                "cached_at": cached_data.get("meta", {}).get("updated_at", "unknown")
            }
        print("⚠️ No cache available, falling back to static functions")
    
    # Static fallback with defaults (Layer 3)
    # Auto-generated at: 2025-08-18T09:48:17.198190
    functions = {
        "query": [
            {
                "name": "query.locations",
                "description": "查詢位置資料",
                "params": ["type", "rooms", "has_rack"],
                "returns": "array",
                "defaults": {"type": "enter_or_exit", "rooms": [1, 2, 3, 4, 5], "has_rack": True}
            },
            {
                "name": "query.racks",
                "description": "查詢架台資料",
                "params": ["location_id", "status"],
                "returns": "array",
                "defaults": {"location_id": 1, "status": "available"}
            },
            {
                "name": "query.tasks",
                "description": "查詢任務資料",
                "params": ["status", "type", "limit"],
                "returns": "array",
                "defaults": {"status": "pending", "type": "RACK_ROTATION", "limit": 10}
            },
            {
                "name": "query.agvs",
                "description": "查詢 AGV 資料",
                "params": ["status", "type"],
                "returns": "array",
                "defaults": {"status": "idle", "type": "cargo"}
            },
        ],
        "check": [
            {
                "name": "check.empty",
                "description": "檢查資料是否為空",
                "params": ["data"],
                "returns": "boolean",
                "defaults": {"data": "${query_result}"}
            },
            {
                "name": "check.rack_status",
                "description": "檢查架台狀態",
                "params": ["rack_id", "side", "check_type"],
                "returns": "boolean",
                "defaults": {"rack_id": "rack001", "side": "a", "check_type": "occupied"}
            },
            {
                "name": "check.task_exists",
                "description": "檢查任務是否存在",
                "params": ["type", "location_id", "rack_id", "status"],
                "returns": "boolean",
                "defaults": {"type": "RACK_ROTATION", "location_id": 1, "rack_id": None, "status": None}
            },
            {
                "name": "check.location_available",
                "description": "檢查位置是否可用",
                "params": ["location_id"],
                "returns": "boolean",
                "defaults": {"location_id": 1}
            },
            {
                "name": "check.system_ready",
                "description": "檢查系統就緒狀態",
                "params": ["components"],
                "returns": "boolean",
                "defaults": {"components": ["database", "agv_fleet", "plc"]}
            },
        ],
        "task": [
            {
                "name": "task.create_task",
                "description": "建立新任務",
                "params": ["type", "work_id", "location_id", "rack_id", "room_id", "priority", "metadata"],
                "returns": "string",
                "defaults": {"type": "RACK_ROTATION", "work_id": 1, "location_id": "loc001", "rack_id": None, "room_id": None, "priority": "normal", "metadata": {}}
            },
            {
                "name": "task.update_task",
                "description": "更新任務狀態",
                "params": ["task_id", "status"],
                "returns": "boolean",
                "defaults": {"task_id": "${task_id}", "status": "executing"}
            },
            {
                "name": "task.assign_task",
                "description": "分配任務給 AGV",
                "params": ["task_id", "agv_id"],
                "returns": "object",
                "defaults": {"task_id": "${task_id}", "agv_id": "agv01"}
            },
            {
                "name": "task.cancel_task",
                "description": "取消任務",
                "params": ["task_id", "reason"],
                "returns": "boolean",
                "defaults": {"task_id": "${task_id}", "reason": "User cancelled"}
            },
        ],
        "action": [
            {
                "name": "action.rotate_rack",
                "description": "旋轉架台",
                "params": ["rack_id", "angle"],
                "returns": "boolean",
                "defaults": {"rack_id": "rack001", "angle": 180}
            },
            {
                "name": "action.send_notification",
                "description": "發送通知",
                "params": ["message", "level", "priority"],
                "returns": "boolean",
                "defaults": {"message": "Task completed", "level": "info", "priority": "normal"}
            },
            {
                "name": "action.log_message",
                "description": "記錄日誌",
                "params": ["message", "level", "tags"],
                "returns": "boolean",
                "defaults": {"message": "Flow step executed", "level": "info", "tags": []}
            },
            {
                "name": "action.optimize_batch",
                "description": "最佳化任務批次",
                "params": ["task_type", "optimization_strategy"],
                "returns": "object",
                "defaults": {"task_type": "RACK_ROTATION", "optimization_strategy": "distance"}
            },
            {
                "name": "action.analyze_priorities",
                "description": "分析任務優先級",
                "params": ["tasks", "strategy"],
                "returns": "array",
                "defaults": {"tasks": [], "strategy": "deadline_first"}
            },
            {
                "name": "action.find_optimal_agv",
                "description": "尋找最佳 AGV",
                "params": ["task", "agvs", "criteria"],
                "returns": "object",
                "defaults": {"task": {}, "agvs": [], "criteria": ["distance"]}
            },
            {
                "name": "action.recover_from_error",
                "description": "錯誤恢復",
                "params": ["error_type", "step_id", "retry_count"],
                "returns": "object",
                "defaults": {"error_type": "unknown", "step_id": "", "retry_count": 3}
            },
            {
                "name": "action.calculate_metrics",
                "description": "計算效能指標",
                "params": ["metrics"],
                "returns": "object",
                "defaults": {"metrics": ["task_completion_rate"]}
            },
            {
                "name": "action.optimize_performance",
                "description": "優化系統效能",
                "params": ["target", "threshold"],
                "returns": "object",
                "defaults": {"target": "efficiency", "threshold": 80}
            },
            {
                "name": "action.send_alert",
                "description": "發送警報",
                "params": ["message", "severity"],
                "returns": "boolean",
                "defaults": {"message": "System alert", "severity": "warning"}
            },
            {
                "name": "action.cleanup_resources",
                "description": "清理資源",
                "params": ["targets"],
                "returns": "boolean",
                "defaults": {"targets": ["temp_variables"]}
            },
            {
                "name": "action.generate_report",
                "description": "生成報告",
                "params": ["flow_id", "include"],
                "returns": "object",
                "defaults": {"flow_id": "", "include": ["execution_time", "tasks_processed"]}
            },
            {
                "name": "action.save_data",
                "description": "保存資料到檔案",
                "params": ["data", "path"],
                "returns": "boolean",
                "defaults": {"data": {}, "path": "/tmp/flow_data.json"}
            },
        ],
        "control": [
            {
                "name": "control.wait_time",
                "description": "等待指定時間",
                "params": ["seconds"],
                "returns": "boolean",
                "defaults": {"seconds": 1.0}
            },
            {
                "name": "control.stop_flow",
                "description": "停止流程執行",
                "params": ["reason"],
                "returns": "boolean",
                "defaults": {"reason": "Condition met"}
            },
            {
                "name": "control.count_items",
                "description": "計算項目數量",
                "params": ["variable"],
                "returns": "number",
                "defaults": {"variable": "${query_result}"}
            },
            {
                "name": "control.switch_case",
                "description": "Switch case 控制",
                "params": ["value", "cases", "default"],
                "returns": "any",
                "defaults": {"value": "${status}", "cases": {"pending": "wait", "ready": "execute"}, "default": "skip"}
            },
            {
                "name": "control.foreach",
                "description": "迴圈遍歷",
                "params": ["items", "var", "steps", "max_iterations"],
                "returns": "array",
                "defaults": {"items": [], "var": "item", "steps": [], "max_iterations": 1000}
            },
            {
                "name": "control.update_variable",
                "description": "更新變數值",
                "params": ["variable", "operation", "value"],
                "returns": "any",
                "defaults": {"variable": "counter", "operation": "set", "value": 0}
            },
            {
                "name": "control.switch_advanced",
                "description": "進階條件分支控制",
                "params": ["value", "cases"],
                "returns": "any",
                "defaults": {"value": 0, "cases": []}
            },
        ],
        "special": [
            {
                "name": "foreach",
                "description": "迴圈遍歷",
                "params": ["items", "var", "steps", "max_iterations"],
                "returns": "array",
                "defaults": {"items": [], "var": "item", "steps": [], "max_iterations": 1000}
            },
        ],
    }

    # Always return categorized format
    return {
        "success": True, 
        "functions": functions,
        "source": "static",
        "format": "categorized",
        "cached_at": "static_fallback"
    }


@router.get("/api/templates")
async def get_flow_templates():
    """API: Get flow templates"""
    templates = [
        {
            "id": "basic_query",
            "name": "Basic Query Flow",
            "description": "Simple flow with database query",
            "template": {
                "meta": {
                    "system": "linear_flow_v2",
                    "version": "2.0.0"
                },
                "flow": {
                    "id": "new_flow",
                    "name": "New Flow",
                    "enabled": True
                },
                "workflow": [
                    {
                        "section": "Query Data",
                        "steps": [
                            {
                                "id": "query_step",
                                "exec": "query.locations",
                                "params": {
                                    "type": "room_inlet"
                                },
                                "store": "locations"
                            }
                        ]
                    }
                ]
            }
        },
        {
            "id": "foreach_template",
            "name": "ForEach Loop Template",
            "description": "Flow with foreach iteration",
            "template": {
                "meta": {
                    "system": "linear_flow_v2",
                    "version": "2.0.0"
                },
                "flow": {
                    "id": "foreach_flow",
                    "name": "ForEach Flow"
                },
                "workflow": [
                    {
                        "section": "Process Items",
                        "steps": [
                            {
                                "id": "foreach_items",
                                "exec": "foreach",
                                "items": "${items}",
                                "var": "item",
                                "steps": [
                                    {
                                        "id": "process_item",
                                        "exec": "action.log",
                                        "params": {
                                            "message": "Processing ${item}"
                                        }
                                    }
                                ]
                            }
                        ]
                    }
                ]
            }
        },
        {
            "id": "conditional_template",
            "name": "Conditional Flow",
            "description": "Flow with conditional execution",
            "template": {
                "meta": {
                    "system": "linear_flow_v2",
                    "version": "2.0.0"
                },
                "flow": {
                    "id": "conditional_flow",
                    "name": "Conditional Flow"
                },
                "workflow": [
                    {
                        "section": "Check and Execute",
                        "steps": [
                            {
                                "id": "check_condition",
                                "exec": "check.system_ready",
                                "params": {
                                    "components": ["database"]
                                },
                                "store": "is_ready"
                            },
                            {
                                "id": "execute_if_ready",
                                "skip_if_not": "${is_ready}",
                                "exec": "task.create",
                                "params": {
                                    "type": "TEST_TASK"
                                }
                            }
                        ]
                    }
                ]
            }
        }
    ]
    
    return {"success": True, "templates": templates}


@router.post("/api/flows/export/{flow_id}")
async def export_flow(flow_id: str):
    """API: Export flow as YAML file"""
    try:
        flow_data = flow_manager.load_flow(flow_id)
        
        if not flow_data:
            raise HTTPException(status_code=404, detail="Flow not found")
        
        # Create YAML content
        yaml_content = yaml.dump(flow_data, default_flow_style=False, 
                                allow_unicode=True, sort_keys=False)
        
        # Return as file download
        from io import BytesIO
        file_content = BytesIO(yaml_content.encode('utf-8'))
        
        return FileResponse(
            file_content,
            media_type="application/x-yaml",
            filename=f"{flow_id}.yaml"
        )
        
    except HTTPException:
        raise
    except Exception as e:
        return {"success": False, "error": str(e)}


@router.post("/api/flows/import")
async def import_flow(request: Request):
    """API: Import flow from YAML"""
    try:
        # Get YAML content from request
        data = await request.json()
        yaml_content = data.get('yaml_content')
        
        if not yaml_content:
            raise HTTPException(status_code=400, detail="Missing YAML content")
        
        # Parse YAML
        flow_data = yaml.safe_load(yaml_content)
        
        # Validate
        validation = flow_manager.validate_flow(flow_data)
        if not validation['valid']:
            return {
                "success": False,
                "errors": validation['errors'],
                "warnings": validation['warnings']
            }
        
        # Extract flow ID
        flow_id = flow_data.get('flow', {}).get('id')
        if not flow_id:
            raise HTTPException(status_code=400, detail="Flow ID not found in YAML")
        
        # Save flow
        success = flow_manager.save_flow(flow_id, flow_data)
        
        if success:
            return {
                "success": True,
                "flow_id": flow_id,
                "message": "Flow imported successfully"
            }
        else:
            return {"success": False, "error": "Failed to import flow"}
            
    except HTTPException:
        raise
    except yaml.YAMLError as e:
        return {"success": False, "error": f"Invalid YAML: {str(e)}"}
    except Exception as e:
        return {"success": False, "error": str(e)}