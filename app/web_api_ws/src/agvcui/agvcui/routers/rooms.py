from datetime import datetime
from zoneinfo import ZoneInfo
from fastapi import APIRouter, Request, Form, HTTPException
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.templating import Jinja2Templates
from agvcui.database.room_ops import (
    get_rooms, count_rooms, get_room_by_id,
    update_room, get_all_process_settings, get_room_locations
)


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/rooms", response_class=HTMLResponse)
    async def rooms_list(request: Request, page: int = 1):
        from agvcui.middleware import get_current_user_from_request

        limit = 20
        offset = (page - 1) * limit

        rooms = get_rooms(offset=offset, limit=limit)
        total = count_rooms()
        total_pages = (total + limit - 1) // limit
        taipei_time = datetime.now(ZoneInfo("Asia/Taipei"))
        latest_query = taipei_time.strftime("%Y/%m/%d %H:%M:%S")
        current_user = get_current_user_from_request(request)

        # 獲取每個房間的入口/出口位置
        rooms_with_locations = []
        for room in rooms:
            locations = get_room_locations(room.id)
            room_dict = {
                "id": room.id,
                "name": room.name,
                "description": room.description,
                "process_settings_id": room.process_settings_id,
                "enable": room.enable,
                "enter_location_id": locations["enter_location_id"] if locations else None,
                "exit_location_id": locations["exit_location_id"] if locations else None
            }
            rooms_with_locations.append(room_dict)

        return templates.TemplateResponse("rooms.html", {
            "request": request,
            "rooms": rooms_with_locations,
            "active_tab": "rooms",
            "latest_query": latest_query,
            "current_page": page,
            "total_pages": total_pages,
            "total_count": total,
            "current_user": current_user
        })

    @router.get("/rooms/{room_id}/edit", response_class=HTMLResponse)
    async def room_edit_form(request: Request, room_id: int):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 獲取房間資料
        room = get_room_by_id(room_id)
        if not room:
            raise HTTPException(status_code=404, detail="房間未找到")

        # 獲取位置資訊
        locations = get_room_locations(room_id)

        # 獲取所有製程設置
        process_settings = get_all_process_settings()

        return templates.TemplateResponse("room_form.html", {
            "request": request,
            "active_tab": "rooms",
            "current_user": current_user,
            "process_settings": process_settings,
            "room": room,
            "locations": locations,
            "form_title": "編輯房間製程設定",
            "form_action": f"/rooms/{room_id}/edit"
        })

    @router.post("/rooms/{room_id}/edit")
    async def room_edit(
        request: Request,
        room_id: int,
        process_settings_id: int = Form(...),
        enable: bool = Form(False)
    ):
        try:
            room_data = {
                "process_settings_id": process_settings_id,
                "enable": 1 if enable else 0
            }
            update_room(room_id, room_data)
            return RedirectResponse(url="/rooms", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=str(e))

    return router
