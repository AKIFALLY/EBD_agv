from datetime import datetime
from zoneinfo import ZoneInfo
from fastapi import APIRouter, Request, Form, HTTPException
from fastapi.responses import HTMLResponse, RedirectResponse
from typing import Optional
from fastapi.templating import Jinja2Templates
from agvcui.db import (
    get_eqps, count_eqps, get_complete_device, create_complete_device,
    update_complete_device, delete_complete_device,
    get_ports_not_assigned_to_any_eqp, get_all_locations
)
from agvcui.utils.permissions import can_create, can_edit, can_delete


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/devices", response_class=HTMLResponse)
    async def devices_list(request: Request, page: int = 1):
        from agvcui.middleware import get_current_user_from_request

        limit = 20
        offset = (page - 1) * limit
        devices = get_eqps(offset=offset, limit=limit)

        # eqp ports
        for eqp in devices:
            print(f"{eqp.name} has {len(eqp.ports)} ports.")
            for port in eqp.ports:
                print(f"  - Port: {port.name}")

        total = count_eqps()
        total_pages = (total + limit - 1) // limit
        taipei_time = datetime.now(ZoneInfo("Asia/Taipei"))
        latest_query = taipei_time.strftime("%Y/%m/%d %H:%M:%S")
        current_user = get_current_user_from_request(request)

        return templates.TemplateResponse("devices.html", {
            "request": request,
            "devices": devices,
            "active_tab": "devices",
            "latest_query": latest_query,
            "current_page": page,
            "total_pages": total_pages,
            "total_count": total,
            "current_user": current_user
        })

    @router.get("/devices/create", response_class=HTMLResponse)
    async def device_create_form(request: Request):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_create(request):
            raise HTTPException(status_code=403, detail="權限不足")

        # 獲取所有可用的位置選項
        locations = get_all_locations()

        return templates.TemplateResponse("device_form.html", {
            "request": request,
            "active_tab": "devices",
            "current_user": current_user,
            "device": None,  # 新增模式
            "locations": locations,  # 位置選項
            "is_edit_mode": False,  # 標記為創建模式
            "form_title": "新增設備",
            "form_action": "/devices/create"
        })

    @router.post("/devices/create")
    async def device_create(
        request: Request,
        device_json: str = Form(...)  # 接收完整的設備JSON數據
    ):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_create(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            import json
            device_data = json.loads(device_json)

            # 創建完整的設備
            new_device = create_complete_device(device_data)
            if not new_device:
                raise Exception("創建設備失敗")

            return RedirectResponse(url="/devices", status_code=303)
        except json.JSONDecodeError:
            raise HTTPException(status_code=400, detail="設備數據格式錯誤")
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"創建設備失敗: {str(e)}")

    @router.get("/devices/{device_id}", response_class=HTMLResponse)
    async def device_detail(request: Request, device_id: int):
        # 不檢查登入權限，任何人都可看
        device = get_complete_device(device_id)
        if not device:
            raise HTTPException(status_code=404, detail="設備不存在")
        locations = get_all_locations()
        return templates.TemplateResponse("device_detail.html", {
            "request": request,
            "active_tab": "devices",
            "device": device,
            "locations": locations,
            "is_detail_mode": True,
            "form_title": "設備詳情"
        })

    @router.get("/devices/{device_id}/edit", response_class=HTMLResponse)
    async def device_edit_form(request: Request, device_id: int):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_edit(request):
            raise HTTPException(status_code=403, detail="權限不足")

        # 獲取完整的設備信息
        complete_device = get_complete_device(device_id)
        if not complete_device:
            raise HTTPException(status_code=404, detail="設備不存在")

        # 獲取所有可用的位置選項
        locations = get_all_locations()

        return templates.TemplateResponse("device_form.html", {
            "request": request,
            "active_tab": "devices",
            "current_user": current_user,
            "device": complete_device,  # 完整的設備信息
            "locations": locations,  # 位置選項
            "is_edit_mode": True,  # 標記為編輯模式
            "form_title": "編輯設備",
            "form_action": f"/devices/{device_id}/edit"
        })

    @router.post("/devices/{device_id}/edit")
    async def device_edit(
        request: Request,
        device_id: int,
        device_json: str = Form(...)  # 接收完整的設備JSON數據
    ):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_edit(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            import json
            device_data = json.loads(device_json)

            # 更新完整的設備
            updated_device = update_complete_device(device_id, device_data)
            if not updated_device:
                raise HTTPException(status_code=404, detail="設備不存在")

            return RedirectResponse(url="/devices", status_code=303)
        except json.JSONDecodeError:
            raise HTTPException(status_code=400, detail="設備數據格式錯誤")
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"更新設備失敗: {str(e)}")

    @router.post("/devices/{device_id}/delete")
    async def device_delete(request: Request, device_id: int):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_delete(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            # 刪除完整的設備（包含所有端口和信號）
            success = delete_complete_device(device_id)
            if not success:
                raise HTTPException(status_code=404, detail="設備不存在")

            return RedirectResponse(url="/devices", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"刪除設備失敗: {str(e)}")

    @router.get("/api/eqps/{eqp_id}/ports")
    async def get_eqp_ports(eqp_id: int):
        from agvcui.database.equipment_ops import get_eqp_ports_by_eqp_id, get_signals_by_eqp_id
        ports = get_eqp_ports_by_eqp_id(eqp_id)
        if ports is None:
            raise HTTPException(status_code=404, detail="設備不存在或無端口")

        # 查詢該設備下所有 signal
        signals = get_signals_by_eqp_id(
            eqp_id, offset=0, limit=1000)  # 假設不會超過1000
        # 依 eqp_port_id 分組
        signals_by_port = {}
        for s in signals:
            if s.eqp_port_id not in signals_by_port:
                signals_by_port[s.eqp_port_id] = []
            signals_by_port[s.eqp_port_id].append(s)

        def convert_value(value, type_of_value):
            if value is None or type_of_value is None:
                return None
            t = type_of_value.lower()
            try:
                if t == "bool":
                    return str(value).lower() in ("true", "1", "yes", "on")
                elif t == "int":
                    return int(value)
                elif t == "float":
                    return float(value)
                elif t == "string":
                    return str(value)
                else:
                    return value
            except Exception:
                return value

        def find_signal_value(port_signals, keyword):
            for sig in port_signals:
                if keyword.lower() in (sig.name or '').lower():
                    return convert_value(sig.value, getattr(sig, "type_of_value", None))
            return None

        result = []
        for port in ports:
            port_signals = signals_by_port.get(port.id, [])
            presence = find_signal_value(port_signals, "Presence")
            allow_load = find_signal_value(port_signals, "Allow_Load")
            allow_unload = find_signal_value(port_signals, "Allow_Unload")
            result.append({
                "id": port.id,
                "name": port.name,
                "presence": presence,
                "allow_load": allow_load,
                "allow_unload": allow_unload,
            })
        return result

    return router
