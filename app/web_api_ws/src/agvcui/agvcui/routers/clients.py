from datetime import datetime
from zoneinfo import ZoneInfo
from fastapi import APIRouter, Request, Form, HTTPException
from fastapi.responses import HTMLResponse, RedirectResponse, JSONResponse
from fastapi.templating import Jinja2Templates
from agvcui.db import (
    get_clients, count_clients, get_client_by_id,
    update_client, reset_client_op_settings, delete_client,
    get_all_machines
)
from agvcui.utils.permissions import can_create, can_edit, can_delete


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/clients", response_class=HTMLResponse)
    async def clients_list(request: Request, page: int = 1):
        from agvcui.middleware import get_current_user_from_request

        limit = 20
        offset = (page - 1) * limit
        clients = get_clients(offset=offset, limit=limit)
        total = count_clients()
        total_pages = (total + limit - 1) // limit
        taipei_time = datetime.now(ZoneInfo("Asia/Taipei"))
        latest_query = taipei_time.strftime("%Y/%m/%d %H:%M:%S")
        current_user = get_current_user_from_request(request)

        return templates.TemplateResponse("clients.html", {
            "request": request,
            "clients": clients,
            "active_tab": "clients",
            "latest_query": latest_query,
            "current_page": page,
            "total_pages": total_pages,
            "total_count": total,
            "current_user": current_user,
            "now": taipei_time  # 用於計算時間差
        })

    @router.get("/clients/{client_id}/details")
    async def client_details(request: Request, client_id: str):
        """獲取客戶端詳細資訊 (JSON API)"""
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        client = get_client_by_id(client_id)
        if not client:
            raise HTTPException(status_code=404, detail="客戶端不存在")

        return JSONResponse({
            "id": client.id,
            "machine_id": client.machine_id,
            "user_agent": client.user_agent,
            "op": client.op or {},
            "created_at": client.created_at.isoformat() if client.created_at else None,
            "updated_at": client.updated_at.isoformat() if client.updated_at else None
        })

    @router.get("/clients/{client_id}/edit", response_class=HTMLResponse)
    async def client_edit_form(request: Request, client_id: str):
        """顯示客戶端編輯表單"""
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_edit(request):
            raise HTTPException(status_code=403, detail="權限不足")

        client = get_client_by_id(client_id)
        if not client:
            raise HTTPException(status_code=404, detail="客戶端不存在")

        # 獲取所有機器列表供選擇
        machines = get_all_machines()

        return templates.TemplateResponse("client_form.html", {
            "request": request,
            "active_tab": "clients",
            "current_user": current_user,
            "client": client,
            "machines": machines,
            "form_title": "編輯客戶端",
            "form_action": f"/clients/{client_id}/edit"
        })

    @router.post("/clients/{client_id}/edit")
    async def client_edit(
        request: Request,
        client_id: str,
        machine_id: str = Form(""),
        user_agent: str = Form("")
    ):
        """處理客戶端編輯請求"""
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_edit(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            client_data = {}

            # 處理機器 ID
            if machine_id and machine_id.isdigit():
                client_data["machine_id"] = int(machine_id)
            elif machine_id == "":
                client_data["machine_id"] = None

            # 處理用戶代理
            if user_agent:
                client_data["user_agent"] = user_agent

            updated_client = update_client(client_id, client_data)
            if not updated_client:
                raise HTTPException(status_code=404, detail="客戶端不存在")

            return RedirectResponse(url="/clients", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"更新客戶端失敗: {str(e)}")

    @router.post("/clients/{client_id}/reset")
    async def client_reset(request: Request, client_id: str):
        """重置客戶端 OP 設定"""
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_edit(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            updated_client = reset_client_op_settings(client_id)
            if not updated_client:
                raise HTTPException(status_code=404, detail="客戶端不存在")

            return JSONResponse({"success": True, "message": "客戶端設定已重置"})
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"重置設定失敗: {str(e)}")

    @router.post("/clients/{client_id}/delete")
    async def client_delete(request: Request, client_id: str):
        """刪除客戶端"""
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_delete(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            success = delete_client(client_id)
            if not success:
                raise HTTPException(status_code=404, detail="客戶端不存在")

            return RedirectResponse(url="/clients", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"刪除客戶端失敗: {str(e)}")

    return router
