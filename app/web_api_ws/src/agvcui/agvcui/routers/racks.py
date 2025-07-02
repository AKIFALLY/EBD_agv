from datetime import datetime
from zoneinfo import ZoneInfo
from fastapi import APIRouter, Request, Form, HTTPException
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.templating import Jinja2Templates
from agvcui.db import (
    get_racks, count_racks, get_rack_by_id,
    create_rack, update_rack, delete_rack,
    get_all_products, get_all_agvs, get_all_locations, get_all_rack_statuses
)


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/racks", response_class=HTMLResponse)
    async def racks_list(request: Request, page: int = 1):
        from agvcui.middleware import get_current_user_from_request

        limit = 20
        offset = (page - 1) * limit
        racks = get_racks(offset=offset, limit=limit)
        total = count_racks()
        total_pages = (total + limit - 1) // limit
        taipei_time = datetime.now(ZoneInfo("Asia/Taipei"))
        latest_query = taipei_time.strftime("%Y/%m/%d %H:%M:%S")
        current_user = get_current_user_from_request(request)

        return templates.TemplateResponse("racks.html", {
            "request": request,
            "racks": racks,
            "active_tab": "racks",
            "latest_query": latest_query,
            "current_page": page,
            "total_pages": total_pages,
            "total_count": total,
            "current_user": current_user
        })

    @router.get("/racks/create", response_class=HTMLResponse)
    async def rack_create_form(request: Request):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 獲取相關資料
        products = get_all_products()
        agvs = get_all_agvs()
        locations = get_all_locations()
        rack_statuses = get_all_rack_statuses()

        return templates.TemplateResponse("rack_form.html", {
            "request": request,
            "active_tab": "racks",
            "current_user": current_user,
            "products": products,
            "agvs": agvs,
            "locations": locations,
            "rack_statuses": rack_statuses,
            "rack": None,  # 新增時為 None
            "form_title": "新增貨架",
            "form_action": "/racks/create"
        })

    @router.post("/racks/create")
    async def rack_create(
        request: Request,
        name: str = Form(...),
        agv_id: str = Form(""),
        location_id: str = Form(""),
        product_id: str = Form(""),
        is_carry: int = Form(0),
        is_in_map: int = Form(0),
        is_docked: int = Form(0),
        status_id: int = Form(1),
        direction: int = Form(0)
    ):
        try:
            # 處理可選的整數字段
            def parse_optional_int(value: str) -> int | None:
                if value and value.strip():
                    try:
                        return int(value)
                    except ValueError:
                        return None
                return None

            rack_data = {
                "name": name,
                "agv_id": parse_optional_int(agv_id),
                "location_id": parse_optional_int(location_id),
                "product_id": parse_optional_int(product_id),
                "is_carry": is_carry,
                "is_in_map": is_in_map,
                "is_docked": is_docked,
                "status_id": status_id,
                "direction": direction
            }
            create_rack(rack_data)
            return RedirectResponse(url="/racks", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=str(e))

    @router.get("/racks/{rack_id}/edit", response_class=HTMLResponse)
    async def rack_edit_form(request: Request, rack_id: int):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 獲取貨架資料
        rack = get_rack_by_id(rack_id)
        if not rack:
            raise HTTPException(status_code=404, detail="貨架未找到")

        # 獲取相關資料
        products = get_all_products()
        agvs = get_all_agvs()
        locations = get_all_locations()
        rack_statuses = get_all_rack_statuses()

        return templates.TemplateResponse("rack_form.html", {
            "request": request,
            "active_tab": "racks",
            "current_user": current_user,
            "products": products,
            "agvs": agvs,
            "locations": locations,
            "rack_statuses": rack_statuses,
            "rack": rack,
            "form_title": "編輯貨架",
            "form_action": f"/racks/{rack_id}/edit"
        })

    @router.post("/racks/{rack_id}/edit")
    async def rack_edit(
        request: Request,
        rack_id: int,
        name: str = Form(...),
        agv_id: str = Form(""),
        location_id: str = Form(""),
        product_id: str = Form(""),
        is_carry: int = Form(0),
        is_in_map: int = Form(0),
        is_docked: int = Form(0),
        status_id: int = Form(1),
        direction: int = Form(0)
    ):
        try:
            # 處理可選的整數字段
            def parse_optional_int(value: str) -> int | None:
                if value and value.strip():
                    try:
                        return int(value)
                    except ValueError:
                        return None
                return None

            rack_data = {
                "name": name,
                "agv_id": parse_optional_int(agv_id),
                "location_id": parse_optional_int(location_id),
                "product_id": parse_optional_int(product_id),
                "is_carry": is_carry,
                "is_in_map": is_in_map,
                "is_docked": is_docked,
                "status_id": status_id,
                "direction": direction
            }
            update_rack(rack_id, rack_data)
            return RedirectResponse(url="/racks", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=str(e))

    @router.post("/racks/{rack_id}/delete")
    async def rack_delete(rack_id: int):
        try:
            delete_rack(rack_id)
            return RedirectResponse(url="/racks", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=str(e))

    return router
