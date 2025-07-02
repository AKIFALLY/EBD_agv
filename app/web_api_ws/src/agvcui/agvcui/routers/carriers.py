from datetime import datetime
from zoneinfo import ZoneInfo
from fastapi import APIRouter, Request, Form, HTTPException
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.templating import Jinja2Templates
from agvcui.db import (
    get_carriers, count_carriers, get_carriers_grouped,
    get_carrier_status_list, get_carrier_by_id, update_carrier,
    delete_carrier, create_carrier, get_all_rooms, get_all_racks, get_all_eqp_ports
)
from agvcui.utils.permissions import can_create, can_edit, can_delete


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/carriers", response_class=HTMLResponse)
    async def carriers_list(request: Request, page: int = 1, rack_id: int = None):
        from agvcui.middleware import get_current_user_from_request

        # 獲取分組資料
        grouped_carriers = get_carriers_grouped()

        # 獲取所有載具用於列表檢視
        all_carriers = get_carriers(offset=0, limit=1000)  # 獲取所有載具

        # 如果有 rack_id 參數，篩選特定貨架的載具
        filtered_rack_id = None
        if rack_id:
            filtered_rack_id = rack_id
            # 篩選分組資料中的特定貨架
            if grouped_carriers['rack_groups']:
                grouped_carriers['rack_groups'] = [
                    group for group in grouped_carriers['rack_groups']
                    if group['rack_id'] == rack_id
                ]
            # 清空其他分組，只顯示貨架載具
            grouped_carriers['room_groups'] = []
            grouped_carriers['unassigned_carriers'] = []

            # 篩選列表資料中的特定貨架載具
            all_carriers = [
                carrier for carrier in all_carriers
                if carrier.rack_id == rack_id
            ]

        total = count_carriers()
        taipei_time = datetime.now(ZoneInfo("Asia/Taipei"))
        latest_query = taipei_time.strftime("%Y/%m/%d %H:%M:%S")
        current_user = get_current_user_from_request(request)

        # 狀態顏色和名稱映射函數
        def get_status_color(status_id):
            status_colors = {
                1: 'is-success',    # 空閒
                2: 'is-warning',    # 使用中
                3: 'is-danger',     # 故障
                4: 'is-info',       # 待處理
                5: 'is-primary',    # 處理中
                6: 'is-dark',       # NG
            }
            return status_colors.get(status_id, 'is-light')

        def get_status_name(status_id):
            status_names = {
                1: '空閒',
                2: '使用中',
                3: '故障',
                4: '待處理',
                5: '處理中',
                6: 'NG',
            }
            return status_names.get(status_id, '未知')

        return templates.TemplateResponse("carriers.html", {
            "request": request,
            "grouped_carriers": grouped_carriers,
            "all_carriers": all_carriers,
            "active_tab": "carriers",
            "latest_query": latest_query,
            "current_page": page,
            "total_pages": 1,  # 分組檢視不需要分頁
            "total_count": total,
            "current_user": current_user,
            "get_status_color": get_status_color,
            "get_status_name": get_status_name,
            "filtered_rack_id": filtered_rack_id  # 新增篩選資訊
        })

    @router.get("/carriers/create", response_class=HTMLResponse)
    async def carrier_create_form(request: Request, rack_id: int = None, rack_index: int = None):
        """顯示載具創建表單"""
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_create(request):
            raise HTTPException(status_code=403, detail="權限不足")

        # 獲取選項資料
        rooms = get_all_rooms()
        racks = get_all_racks()
        ports = get_all_eqp_ports()
        carrier_statuses = get_carrier_status_list()

        return templates.TemplateResponse("carrier_form.html", {
            "request": request,
            "active_tab": "carriers",
            "current_user": current_user,
            "carrier": None,
            "rooms": rooms,
            "racks": racks,
            "ports": ports,
            "carrier_statuses": carrier_statuses,
            "form_title": "新增載具",
            "form_action": "/carriers/create",
            "preset_rack_id": rack_id,  # 新增預設貨架 ID
            "preset_rack_index": rack_index  # 新增預設格位索引
        })

    @router.post("/carriers/create")
    async def carrier_create(
        request: Request,
        room_id: str = Form(""),
        rack_id: str = Form(""),
        port_id: str = Form(""),
        rack_index: str = Form(""),
        status_id: str = Form("1")
    ):
        """處理載具創建請求"""
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_create(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            carrier_data = {}

            # 處理位置設定（只能選擇一個）
            if room_id and room_id.isdigit():
                carrier_data["room_id"] = int(room_id)
            elif rack_id and rack_id.isdigit():
                carrier_data["rack_id"] = int(rack_id)
                if rack_index and rack_index.isdigit():
                    carrier_data["rack_index"] = int(rack_index)
            elif port_id and port_id.isdigit():
                carrier_data["port_id"] = int(port_id)

            # 處理狀態
            if status_id and status_id.isdigit():
                carrier_data["status_id"] = int(status_id)

            new_carrier = create_carrier(carrier_data)
            return RedirectResponse(url="/carriers", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"創建載具失敗: {str(e)}")

    @router.get("/carriers/{carrier_id}/edit", response_class=HTMLResponse)
    async def carrier_edit_form(request: Request, carrier_id: int):
        """顯示載具編輯表單"""
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_edit(request):
            raise HTTPException(status_code=403, detail="權限不足")

        carrier = get_carrier_by_id(carrier_id)
        if not carrier:
            raise HTTPException(status_code=404, detail="載具不存在")

        # 獲取選項資料
        rooms = get_all_rooms()
        racks = get_all_racks()
        ports = get_all_eqp_ports()
        carrier_statuses = get_carrier_status_list()

        # 狀態顏色和名稱映射函數
        def get_status_color(status_id):
            status_colors = {
                1: 'is-success', 2: 'is-warning', 3: 'is-danger',
                4: 'is-info', 5: 'is-primary', 6: 'is-dark',
                7: 'is-light', 8: 'is-link'
            }
            return status_colors.get(status_id, 'is-light')

        def get_status_name(status_id):
            status_names = {
                1: '空閒', 2: '使用中', 3: '故障', 4: '待處理',
                5: '處理中', 6: 'NG', 7: '維護中', 8: '已完成'
            }
            return status_names.get(status_id, '未知')

        return templates.TemplateResponse("carrier_form.html", {
            "request": request,
            "active_tab": "carriers",
            "current_user": current_user,
            "carrier": carrier,
            "rooms": rooms,
            "racks": racks,
            "ports": ports,
            "carrier_statuses": carrier_statuses,
            "form_title": "編輯載具",
            "form_action": f"/carriers/{carrier_id}/edit",
            "get_status_color": get_status_color,
            "get_status_name": get_status_name
        })

    @router.post("/carriers/{carrier_id}/edit")
    async def carrier_edit(
        request: Request,
        carrier_id: int,
        room_id: str = Form(""),
        rack_id: str = Form(""),
        port_id: str = Form(""),
        rack_index: str = Form(""),
        status_id: str = Form("1")
    ):
        """處理載具編輯請求"""
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_edit(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            carrier_data = {}

            # 清除所有位置設定
            carrier_data["room_id"] = None
            carrier_data["rack_id"] = None
            carrier_data["port_id"] = None
            carrier_data["rack_index"] = None

            # 設定新的位置（只能選擇一個）
            if room_id and room_id.isdigit():
                carrier_data["room_id"] = int(room_id)
            elif rack_id and rack_id.isdigit():
                carrier_data["rack_id"] = int(rack_id)
                if rack_index and rack_index.isdigit():
                    carrier_data["rack_index"] = int(rack_index)
            elif port_id and port_id.isdigit():
                carrier_data["port_id"] = int(port_id)

            # 處理狀態
            if status_id and status_id.isdigit():
                carrier_data["status_id"] = int(status_id)

            updated_carrier = update_carrier(carrier_id, carrier_data)
            if not updated_carrier:
                raise HTTPException(status_code=404, detail="載具不存在")

            return RedirectResponse(url="/carriers", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"更新載具失敗: {str(e)}")

    @router.post("/carriers/{carrier_id}/delete")
    async def carrier_delete(request: Request, carrier_id: int):
        """刪除載具"""
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 檢查權限
        if not can_delete(request):
            raise HTTPException(status_code=403, detail="權限不足")

        try:
            success = delete_carrier(carrier_id)
            if not success:
                raise HTTPException(status_code=404, detail="載具不存在")

            return RedirectResponse(url="/carriers", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"刪除載具失敗: {str(e)}")

    return router
