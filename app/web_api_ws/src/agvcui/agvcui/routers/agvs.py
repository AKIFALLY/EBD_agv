from datetime import datetime
from zoneinfo import ZoneInfo
from fastapi import APIRouter, Request, Form, HTTPException
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.templating import Jinja2Templates
from agvcui.db import (
    get_all_agvs, get_agv_by_id, create_agv, update_agv, delete_agv
)
from agvcui.middleware import get_current_user_from_request
from agvcui.utils.permissions import can_create, can_edit, can_delete


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/agvs", response_class=HTMLResponse)
    async def agvs_list(request: Request, page: int = 1):
        from agvcui.middleware import get_current_user_from_request

        agvs = get_all_agvs()
        total = len(agvs)
        taipei_time = datetime.now(ZoneInfo("Asia/Taipei"))
        latest_query = taipei_time.strftime("%Y/%m/%d %H:%M:%S")
        current_user = get_current_user_from_request(request)

        return templates.TemplateResponse("agvs.html", {
            "request": request,
            "agvs": agvs,
            "active_tab": "agvs",
            "latest_query": latest_query,
            "current_page": page,
            "total_pages": 1,  # 暫時不分頁
            "total_count": total,
            "current_user": current_user
        })

    @router.get("/agvs/create", response_class=HTMLResponse)
    async def agv_create_form(request: Request):
        current_user = get_current_user_from_request(request)

        # 檢查管理員權限
        if not current_user or current_user.role != "admin":
            return RedirectResponse(url="/login?redirect=/agvs/create", status_code=302)

        return templates.TemplateResponse("agv_form.html", {
            "request": request,
            "active_tab": "agvs",
            "current_user": current_user,
            "agv": None,
            "form_title": "新增 AGV",
            "form_action": "/agvs/create"
        })

    @router.post("/agvs/create")
    async def agv_create(
        request: Request,
        name: str = Form(...),
        description: str = Form(""),
        model: str = Form(...),
        x: float = Form(...),
        y: float = Form(...),
        heading: float = Form(...),
        battery: str = Form(""),
        status_id: str = Form(""),
        last_node_id: str = Form(""),
        enable: str = Form("")
    ):
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_create(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            agv_data = {
                "name": name,
                "description": description if description else None,
                "model": model,
                "x": x,
                "y": y,
                "heading": heading,
                "battery": float(battery) if battery and battery.replace('.', '').isdigit() else None,
                "status_id": int(status_id) if status_id and status_id.isdigit() else None,
                "last_node_id": int(last_node_id) if last_node_id and last_node_id.isdigit() else None,
                "enable": 1 if enable == "1" else 0
            }

            create_agv(agv_data)
            return RedirectResponse(url="/agvs", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"創建 AGV 失敗: {str(e)}")

    @router.get("/agvs/{agv_id}/edit", response_class=HTMLResponse)
    async def agv_edit_form(request: Request, agv_id: int):
        current_user = get_current_user_from_request(request)

        # 檢查管理員權限
        if not current_user or current_user.role != "admin":
            return RedirectResponse(url="/login?redirect=/agvs", status_code=302)

        # 獲取 AGV 資料
        agvs = get_all_agvs()
        agv = next((a for a in agvs if a.id == agv_id), None)

        if not agv:
            raise HTTPException(status_code=404, detail="AGV not found")

        return templates.TemplateResponse("agv_form.html", {
            "request": request,
            "active_tab": "agvs",
            "current_user": current_user,
            "agv": agv,
            "form_title": f"編輯 AGV - {agv.name}",
            "form_action": f"/agvs/{agv_id}/edit"
        })

    @router.post("/agvs/{agv_id}/edit")
    async def agv_edit(
        request: Request,
        agv_id: int,
        name: str = Form(...),
        description: str = Form(""),
        model: str = Form(...),
        x: float = Form(...),
        y: float = Form(...),
        heading: float = Form(...),
        battery: str = Form(""),
        status_id: str = Form(""),
        last_node_id: str = Form(""),
        enable: str = Form("")
    ):
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_edit(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            agv_data = {
                "name": name,
                "description": description if description else None,
                "model": model,
                "x": x,
                "y": y,
                "heading": heading,
                "battery": float(battery) if battery and battery.replace('.', '').isdigit() else None,
                "status_id": int(status_id) if status_id and status_id.isdigit() else None,
                "last_node_id": int(last_node_id) if last_node_id and last_node_id.isdigit() else None,
                "enable": 1 if enable == "1" else 0
            }

            updated_agv = update_agv(agv_id, agv_data)
            if not updated_agv:
                raise HTTPException(status_code=404, detail="AGV 不存在")

            return RedirectResponse(url="/agvs", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"更新 AGV 失敗: {str(e)}")

    @router.post("/agvs/{agv_id}/delete")
    async def agv_delete(request: Request, agv_id: int):
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_delete(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            success = delete_agv(agv_id)
            if not success:
                raise HTTPException(status_code=404, detail="AGV 不存在")

            return RedirectResponse(url="/agvs", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"刪除 AGV 失敗: {str(e)}")

    return router
