from datetime import datetime
from zoneinfo import ZoneInfo
from fastapi import APIRouter, Request, Form, HTTPException
from fastapi.responses import HTMLResponse, RedirectResponse
from typing import Optional
from fastapi.templating import Jinja2Templates
from ..database.task_ops import (
    get_works, count_works, get_work_by_id, create_work, update_work, delete_work
)
from ..utils.permissions import can_create, can_edit, can_delete
import json


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/works", response_class=HTMLResponse)
    async def work_list(request: Request, page: int = 1, search: str = None,
                        sort_by: str = "id", sort_order: str = "desc"):
        from agvcui.middleware import get_current_user_from_request

        limit = 20
        offset = (page - 1) * limit

        # 獲取工作類型列表
        works = get_works(offset=offset, limit=limit, search=search,
                          sort_by=sort_by, sort_order=sort_order)
        total = count_works(search=search)
        total_pages = (total + limit - 1) // limit

        taipei_time = datetime.now(ZoneInfo("Asia/Taipei"))
        latest_query = taipei_time.strftime("%Y/%m/%d %H:%M:%S")
        current_user = get_current_user_from_request(request)

        return templates.TemplateResponse("works.html", {
            "request": request,
            "works": works,
            "active_tab": "works",
            "latest_query": latest_query,
            "current_page": page,
            "total_pages": total_pages,
            "total_count": total,
            "current_user": current_user,
            "search": search or "",
            "sort_by": sort_by,
            "sort_order": sort_order
        })

    @router.get("/works/create", response_class=HTMLResponse)
    async def work_create_form(request: Request):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_create(request):
            raise HTTPException(status_code=403, detail="權限不足")

        return templates.TemplateResponse("work_form.html", {
            "request": request,
            "active_tab": "works",
            "current_user": current_user,
            "work": None,  # 新增模式
            "form_title": "新增工作類型",
            "form_action": "/works/create"
        })

    @router.post("/works/create")
    async def work_create(
        request: Request,
        name: str = Form(...),
        description: str = Form(""),
        parameters: str = Form("")
    ):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_create(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            # 處理參數 JSON
            parameters_dict = None
            if parameters.strip():
                try:
                    parameters_dict = json.loads(parameters)
                except json.JSONDecodeError:
                    raise HTTPException(status_code=400, detail="參數格式錯誤，請輸入有效的 JSON")

            # 創建工作類型
            work_data = {
                "name": name,
                "description": description if description else None,
                "parameters": parameters_dict
            }

            create_work(work_data)
            return RedirectResponse(url="/works", status_code=303)

        except Exception as e:
            raise HTTPException(status_code=500, detail=f"創建失敗: {str(e)}")

    @router.get("/works/{work_id}/edit", response_class=HTMLResponse)
    async def work_edit_form(request: Request, work_id: int):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_edit(request):
            raise HTTPException(status_code=403, detail="權限不足")

        work = get_work_by_id(work_id, include_tasks=True)
        if not work:
            raise HTTPException(status_code=404, detail="工作類型不存在")

        return templates.TemplateResponse("work_form.html", {
            "request": request,
            "active_tab": "works",
            "current_user": current_user,
            "work": work,  # 編輯模式
            "form_title": "編輯工作類型",
            "form_action": f"/works/{work_id}/edit"
        })

    @router.post("/works/{work_id}/edit")
    async def work_edit(
        request: Request,
        work_id: int,
        name: str = Form(...),
        description: str = Form(""),
        parameters: str = Form("")
    ):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_edit(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            # 檢查工作類型是否存在
            existing_work = get_work_by_id(work_id)
            if not existing_work:
                raise HTTPException(status_code=404, detail="工作類型不存在")

            # 處理參數 JSON
            parameters_dict = None
            if parameters.strip():
                try:
                    parameters_dict = json.loads(parameters)
                except json.JSONDecodeError:
                    raise HTTPException(status_code=400, detail="參數格式錯誤，請輸入有效的 JSON")

            # 更新工作類型
            work_data = {
                "name": name,
                "description": description if description else None,
                "parameters": parameters_dict
            }

            update_work(work_id, work_data)
            return RedirectResponse(url="/works", status_code=303)

        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"更新失敗: {str(e)}")

    @router.post("/works/{work_id}/delete")
    async def work_delete(request: Request, work_id: int):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_delete(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            # 檢查工作類型是否存在
            existing_work = get_work_by_id(work_id)
            if not existing_work:
                raise HTTPException(status_code=404, detail="工作類型不存在")

            # 刪除工作類型
            success = delete_work(work_id)
            if not success:
                raise HTTPException(status_code=500, detail="刪除失敗")

            return RedirectResponse(url="/works", status_code=303)

        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"刪除失敗: {str(e)}")

    return router
