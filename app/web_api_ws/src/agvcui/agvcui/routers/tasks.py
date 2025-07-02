from datetime import datetime
from zoneinfo import ZoneInfo
from fastapi import APIRouter, Request, Form, HTTPException
from fastapi.responses import HTMLResponse, RedirectResponse
from typing import Optional
from fastapi.templating import Jinja2Templates
from agvcui.db import (
    get_tasks, count_tasks, get_task_by_id, create_task, update_task, delete_task,
    work_all, task_status_all, room_all, agv_all, node_all
)
from agvcui.database.task_ops import get_tasks_with_hierarchy
from agvcui.utils.permissions import can_create, can_edit, can_delete


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/tasks", response_class=HTMLResponse)
    async def task_list(request: Request, page: int = 1, agv_id: Optional[int] = None, view: str = "list"):
        from agvcui.middleware import get_current_user_from_request

        limit = 20
        offset = (page - 1) * limit

        # 根據 AGV ID 過濾任務
        if agv_id:
            # 獲取所有任務進行過濾
            all_tasks = get_tasks(offset=0, limit=1000)  # 獲取更多任務進行過濾
            # 修復：使用正確的屬性訪問方式，處理 None 值
            tasks = [task for task in all_tasks if getattr(
                task, 'agv_id', None) == agv_id]
            total = len(tasks)
            # 手動分頁
            tasks = tasks[offset:offset + limit]
        else:
            tasks = get_tasks(offset=offset, limit=limit)
            total = count_tasks()

        total_pages = (total + limit - 1) // limit

        taipei_time = datetime.now(ZoneInfo("Asia/Taipei"))
        latest_query = taipei_time.strftime("%Y/%m/%d %H:%M:%S")
        current_user = get_current_user_from_request(request)

        # 如果是階層視圖，獲取階層結構
        hierarchy_tasks = None
        if view == "hierarchy":
            hierarchy_tasks = get_tasks_with_hierarchy()

        return templates.TemplateResponse("tasks.html", {
            "request": request,
            "tasks": tasks,
            "hierarchy_tasks": hierarchy_tasks,
            "active_tab": "tasks",
            "latest_query": latest_query,
            "current_page": page,
            "total_pages": total_pages,
            "total_count": total,
            "current_user": current_user,
            "agv_id": agv_id,  # 傳遞 AGV ID 給模板
            "view": view  # 傳遞視圖類型
        })

    @router.get("/tasks/create", response_class=HTMLResponse)
    async def task_create_form(request: Request, agv_id: Optional[int] = None, node_id: Optional[int] = None):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_create(request):
            raise HTTPException(status_code=403, detail="權限不足")

        # 獲取選項數據
        works = work_all()
        task_statuses = task_status_all()
        rooms = room_all()
        agvs = agv_all()
        nodes = node_all()

        # 獲取可用的父任務（排除當前任務本身）
        available_parent_tasks = get_tasks(offset=0, limit=1000)  # 獲取所有任務作為父任務選項

        # 創建一個假的 task 物件來預填表單
        prefilled_task = None
        if agv_id or node_id:
            prefilled_task = {
                'agv_id': agv_id,
                'node_id': node_id,
                'start_location_id': node_id,  # 如果有 node_id，也設置為起始位置
            }

        return templates.TemplateResponse("task_form.html", {
            "request": request,
            "active_tab": "tasks",
            "current_user": current_user,
            "works": works,
            "task_statuses": task_statuses,
            "rooms": rooms,
            "agvs": agvs,
            "nodes": nodes,
            "available_parent_tasks": available_parent_tasks,
            "task": prefilled_task,  # 使用預填資料
            "form_title": "新增任務",
            "form_action": "/tasks/create"
        })

    @router.post("/tasks/create")
    async def task_create(
        request: Request,
        name: str = Form(...),
        description: str = Form(""),
        mission_code: str = Form(""),
        work_id: str = Form(""),
        status_id: str = Form(""),
        room_id: str = Form(""),
        node_id: str = Form(""),
        agv_id: str = Form(""),
        priority: int = Form(0),
        parameters: str = Form(""),
        parent_task_id: str = Form("")
    ):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_create(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:

            # 處理 JSON 參數
            import json
            parsed_parameters = None
            if parameters and parameters.strip():
                try:
                    parsed_parameters = json.loads(parameters)
                except json.JSONDecodeError:
                    raise HTTPException(
                        status_code=400, detail="參數格式錯誤，請輸入有效的 JSON")

            task_data = {
                "name": name,
                "description": description if description else None,
                "mission_code": mission_code if mission_code else None,
                "work_id": int(work_id) if work_id and work_id.isdigit() else None,
                "status_id": int(status_id) if status_id and status_id.isdigit() else None,
                "room_id": int(room_id) if room_id and room_id.isdigit() else None,
                "node_id": int(node_id) if node_id and node_id.isdigit() else None,
                "agv_id": int(agv_id) if agv_id and agv_id.isdigit() else None,
                "parent_task_id": int(parent_task_id) if parent_task_id and parent_task_id.isdigit() else None,
                "priority": priority,
                "parameters": parsed_parameters
            }

            create_task(task_data)
            return RedirectResponse(url="/tasks", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"創建任務失敗: {str(e)}")

    @router.get("/tasks/{task_id}/edit", response_class=HTMLResponse)
    async def task_edit_form(request: Request, task_id: int):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_edit(request):
            raise HTTPException(status_code=403, detail="權限不足")

        task = get_task_by_id(task_id)
        if not task:
            raise HTTPException(status_code=404, detail="任務不存在")

        # 獲取選項數據
        works = work_all()
        task_statuses = task_status_all()
        rooms = room_all()
        agvs = agv_all()
        nodes = node_all()

        # 獲取可用的父任務（排除當前任務本身及其子任務）
        available_parent_tasks = [t for t in get_tasks(offset=0, limit=1000) if t.id != task_id]

        return templates.TemplateResponse("task_form.html", {
            "request": request,
            "active_tab": "tasks",
            "current_user": current_user,
            "works": works,
            "task_statuses": task_statuses,
            "rooms": rooms,
            "agvs": agvs,
            "nodes": nodes,
            "available_parent_tasks": available_parent_tasks,
            "task": task,  # 編輯模式
            "form_title": "編輯任務",
            "form_action": f"/tasks/{task_id}/edit"
        })

    @router.post("/tasks/{task_id}/edit")
    async def task_edit(
        request: Request,
        task_id: int,
        name: str = Form(...),
        description: str = Form(""),
        mission_code: str = Form(""),
        work_id: str = Form(""),
        status_id: str = Form(""),
        room_id: str = Form(""),
        node_id: str = Form(""),
        agv_id: str = Form(""),
        priority: int = Form(0),
        parameters: str = Form(""),
        parent_task_id: str = Form("")
    ):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_edit(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:

            # 處理 JSON 參數
            import json
            parsed_parameters = None
            if parameters and parameters.strip():
                try:
                    parsed_parameters = json.loads(parameters)
                except json.JSONDecodeError:
                    raise HTTPException(
                        status_code=400, detail="參數格式錯誤，請輸入有效的 JSON")

            task_data = {
                "name": name,
                "description": description if description else None,
                "mission_code": mission_code if mission_code else None,
                "work_id": int(work_id) if work_id and work_id.isdigit() else None,
                "status_id": int(status_id) if status_id.isdigit() else None,
                "room_id": int(room_id) if room_id and room_id.isdigit() else None,
                "node_id": int(node_id) if node_id and node_id.isdigit() else None,
                "agv_id": int(agv_id) if agv_id and agv_id.isdigit() else None,
                "parent_task_id": int(parent_task_id) if parent_task_id and parent_task_id.isdigit() else None,
                "priority": priority,
                "parameters": parsed_parameters
            }

            updated_task = update_task(task_id, task_data)
            if not updated_task:
                raise HTTPException(status_code=404, detail="任務不存在")

            return RedirectResponse(url="/tasks", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"更新任務失敗: {str(e)}")

    @router.post("/tasks/{task_id}/delete")
    async def task_delete(request: Request, task_id: int):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_delete(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            success = delete_task(task_id)
            if not success:
                raise HTTPException(status_code=404, detail="任務不存在")

            return RedirectResponse(url="/tasks", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"刪除任務失敗: {str(e)}")

    return router
