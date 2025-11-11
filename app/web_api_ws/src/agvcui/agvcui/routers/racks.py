from datetime import datetime
from zoneinfo import ZoneInfo
from typing import List
from fastapi import APIRouter, Request, Form, HTTPException
from fastapi.responses import HTMLResponse, RedirectResponse, JSONResponse
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel
from agvcui.db import (
    get_racks, count_racks, get_rack_by_id,
    create_rack, update_rack, delete_rack,
    get_all_products, get_all_agvs, get_all_locations, get_all_rack_statuses
)
from agvcui.database.rack_ops import (
    get_rack_with_bitmap_status,
    set_slot_occupied,
    set_slot_empty,
    toggle_slot_enabled,
    clear_all_slots,
    enable_all_slots,
    disable_all_slots
)
from agvcui.services.kuka_sync_service import KukaContainerSyncService
from agvcui.database.connection import connection_pool


# Pydantic 模型用於 API 請求
class BatchSyncRequest(BaseModel):
    """批次同步請求"""
    rack_ids: List[int]
    force: bool = False  # 強制同步即使狀態未變更


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

    # === KUKA 同步 API 端點 ===

    @router.post("/api/racks/{rack_id}/sync-to-kuka")
    async def sync_rack_to_kuka(rack_id: int):
        """
        手動同步單一 Rack 到 KUKA Fleet Manager

        Args:
            rack_id: Rack ID

        Returns:
            JSONResponse: 同步結果
                - success: bool
                - action: str (entry, exit, update, skip)
                - message: str
                - rack_id: int
        """
        try:
            # 取得 rack 資料
            rack = get_rack_by_id(rack_id)
            if not rack:
                raise HTTPException(status_code=404, detail=f"Rack {rack_id} 不存在")

            # 執行同步
            with connection_pool.get_session() as session:
                kuka_sync = KukaContainerSyncService()

                # 手動同步：將當前 rack 視為新狀態，無舊狀態（強制判斷）
                # 如果 is_in_map=1，將觸發 container entry
                sync_result = kuka_sync.sync_rack_to_kuka(None, rack, session)

                return JSONResponse(content={
                    "success": sync_result["success"],
                    "action": sync_result.get("action", "unknown"),
                    "message": sync_result.get("result", {}).get("message") if sync_result["success"]
                    else sync_result.get("error", "Unknown error"),
                    "rack_id": rack_id,
                    "rack_name": rack.name
                })

        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"同步失敗: {str(e)}"
            )

    @router.post("/api/racks/batch-sync-to-kuka")
    async def batch_sync_racks_to_kuka(request_data: BatchSyncRequest):
        """
        批次同步多個 Rack 到 KUKA Fleet Manager

        Args:
            request_data: BatchSyncRequest
                - rack_ids: List[int] - Rack ID 列表
                - force: bool - 強制同步（預設: False）

        Returns:
            JSONResponse: 批次同步結果
                - success: bool - 整體是否成功
                - total: int - 總數
                - succeeded: int - 成功數量
                - failed: int - 失敗數量
                - results: List[dict] - 每個 rack 的同步結果
        """
        try:
            results = []
            succeeded_count = 0
            failed_count = 0

            with connection_pool.get_session() as session:
                kuka_sync = KukaContainerSyncService()

                for rack_id in request_data.rack_ids:
                    try:
                        # 取得 rack
                        rack = get_rack_by_id(rack_id)
                        if not rack:
                            results.append({
                                "rack_id": rack_id,
                                "success": False,
                                "error": "Rack 不存在"
                            })
                            failed_count += 1
                            continue

                        # 執行同步
                        sync_result = kuka_sync.sync_rack_to_kuka(None, rack, session)

                        results.append({
                            "rack_id": rack_id,
                            "rack_name": rack.name,
                            "success": sync_result["success"],
                            "action": sync_result.get("action", "unknown"),
                            "message": sync_result.get("result", {}).get("message") if sync_result["success"]
                            else sync_result.get("error", "Unknown error")
                        })

                        if sync_result["success"]:
                            succeeded_count += 1
                        else:
                            failed_count += 1

                    except Exception as e:
                        results.append({
                            "rack_id": rack_id,
                            "success": False,
                            "error": str(e)
                        })
                        failed_count += 1

            return JSONResponse(content={
                "success": failed_count == 0,
                "total": len(request_data.rack_ids),
                "succeeded": succeeded_count,
                "failed": failed_count,
                "results": results
            })

        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"批次同步失敗: {str(e)}"
            )

    # ============================================================================
    # Carrier Bitmap API 端點
    # ============================================================================

    @router.get("/api/racks/{rack_id}/bitmap-status")
    async def get_bitmap_status(rack_id: int):
        """
        獲取 Rack 的 bitmap 狀態

        Args:
            rack_id: Rack ID

        Returns:
            JSONResponse: Rack 資料及 bitmap 狀態
        """
        try:
            result = get_rack_with_bitmap_status(rack_id)
            if not result:
                raise HTTPException(status_code=404, detail=f"Rack {rack_id} 不存在")

            return JSONResponse(content={
                "success": True,
                "data": result
            })

        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"獲取 bitmap 狀態失敗: {str(e)}"
            )

    @router.post("/racks/{rack_id}/slot/{slot_index}/occupy")
    async def occupy_slot(rack_id: int, slot_index: int):
        """
        設置指定格位為有貨

        Args:
            rack_id: Rack ID
            slot_index: 格位索引 (1-32)

        Returns:
            JSONResponse: 操作結果
        """
        try:
            if not 1 <= slot_index <= 32:
                raise HTTPException(
                    status_code=400,
                    detail=f"無效的格位索引: {slot_index}，必須在 1-32 之間"
                )

            success = set_slot_occupied(rack_id, slot_index)

            if success:
                return JSONResponse(content={
                    "success": True,
                    "message": f"Rack {rack_id} 格位 {slot_index} 已設為有貨",
                    "rack_id": rack_id,
                    "slot_index": slot_index
                })
            else:
                raise HTTPException(
                    status_code=500,
                    detail=f"設置格位失敗"
                )

        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"設置格位有貨失敗: {str(e)}"
            )

    @router.post("/racks/{rack_id}/slot/{slot_index}/empty")
    async def empty_slot(rack_id: int, slot_index: int):
        """
        設置指定格位為空

        Args:
            rack_id: Rack ID
            slot_index: 格位索引 (1-32)

        Returns:
            JSONResponse: 操作結果
        """
        try:
            if not 1 <= slot_index <= 32:
                raise HTTPException(
                    status_code=400,
                    detail=f"無效的格位索引: {slot_index}，必須在 1-32 之間"
                )

            success = set_slot_empty(rack_id, slot_index)

            if success:
                return JSONResponse(content={
                    "success": True,
                    "message": f"Rack {rack_id} 格位 {slot_index} 已清空",
                    "rack_id": rack_id,
                    "slot_index": slot_index
                })
            else:
                raise HTTPException(
                    status_code=500,
                    detail=f"清空格位失敗"
                )

        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"清空格位失敗: {str(e)}"
            )

    @router.post("/racks/{rack_id}/slot/{slot_index}/toggle-enable")
    async def toggle_enable_slot(rack_id: int, slot_index: int):
        """
        切換指定格位的啟用狀態

        Args:
            rack_id: Rack ID
            slot_index: 格位索引 (1-32)

        Returns:
            JSONResponse: 操作結果
        """
        try:
            if not 1 <= slot_index <= 32:
                raise HTTPException(
                    status_code=400,
                    detail=f"無效的格位索引: {slot_index}，必須在 1-32 之間"
                )

            success = toggle_slot_enabled(rack_id, slot_index)

            if success:
                # 獲取最新狀態
                rack_status = get_rack_with_bitmap_status(rack_id)
                is_enabled = rack_status["bitmap_status"][slot_index]["enabled"] if rack_status else False

                return JSONResponse(content={
                    "success": True,
                    "message": f"Rack {rack_id} 格位 {slot_index} 已{'啟用' if is_enabled else '禁用'}",
                    "rack_id": rack_id,
                    "slot_index": slot_index,
                    "enabled": is_enabled
                })
            else:
                raise HTTPException(
                    status_code=500,
                    detail=f"切換格位狀態失敗"
                )

        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"切換格位啟用狀態失敗: {str(e)}"
            )

    @router.post("/racks/{rack_id}/slots/clear-all")
    async def clear_all_rack_slots(rack_id: int):
        """
        清空所有格位 (設置為無貨)

        Args:
            rack_id: Rack ID

        Returns:
            JSONResponse: 操作結果
        """
        try:
            success = clear_all_slots(rack_id)

            if success:
                return JSONResponse(content={
                    "success": True,
                    "message": f"Rack {rack_id} 所有格位已清空",
                    "rack_id": rack_id
                })
            else:
                raise HTTPException(
                    status_code=500,
                    detail=f"清空所有格位失敗"
                )

        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"清空所有格位失敗: {str(e)}"
            )

    @router.post("/racks/{rack_id}/slots/enable-all")
    async def enable_all_rack_slots(rack_id: int, product_size: str = "S"):
        """
        啟用所有格位

        Args:
            rack_id: Rack ID
            product_size: 產品尺寸 ('S'=32格, 'L'=16格)

        Returns:
            JSONResponse: 操作結果
        """
        try:
            success = enable_all_slots(rack_id, product_size)

            if success:
                return JSONResponse(content={
                    "success": True,
                    "message": f"Rack {rack_id} 所有格位已啟用",
                    "rack_id": rack_id,
                    "product_size": product_size
                })
            else:
                raise HTTPException(
                    status_code=500,
                    detail=f"啟用所有格位失敗"
                )

        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"啟用所有格位失敗: {str(e)}"
            )

    @router.post("/racks/{rack_id}/slots/disable-all")
    async def disable_all_rack_slots(rack_id: int):
        """
        禁用所有格位

        Args:
            rack_id: Rack ID

        Returns:
            JSONResponse: 操作結果
        """
        try:
            success = disable_all_slots(rack_id)

            if success:
                return JSONResponse(content={
                    "success": True,
                    "message": f"Rack {rack_id} 所有格位已禁用",
                    "rack_id": rack_id
                })
            else:
                raise HTTPException(
                    status_code=500,
                    detail=f"禁用所有格位失敗"
                )

        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"禁用所有格位失敗: {str(e)}"
            )

    return router
