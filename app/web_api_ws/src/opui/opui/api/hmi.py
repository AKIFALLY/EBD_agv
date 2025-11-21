"""
HMI API 路由模組
處理 HMI 介面的 Rack 操作（Add/Edit/Remove）和權限管理
"""

from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime, timezone
import logging
from opui.database.operations import connection_pool

logger = logging.getLogger(__name__)
from db_proxy.models import Location, Rack, LocationStatus, License, Product, RackStatus
from sqlmodel import select, delete

# 複用 AGVCUI 的 Bitmap 操作函數
from agvcui.database.rack_ops import (
    parse_bitmap,
    bitmap_to_str,
    is_slot_occupied,
    is_slot_enabled,
    set_slot_occupied,
    set_slot_empty,
    toggle_slot_enabled,
    clear_all_slots,
    enable_all_slots,
    disable_all_slots,
    get_rack_with_bitmap_status,
    count_occupied_slots,
    count_enabled_slots
)

# 複用 AGVCUI 的 CRUD
from agvcui.database.connection import rack_crud, product_crud

# 直接使用 db_proxy 的 CRUD（License 和 Location）
from db_proxy.crud.base_crud import BaseCRUD
license_crud = BaseCRUD(License, id_column="id")
location_crud = BaseCRUD(Location, id_column="id")

router = APIRouter(prefix="/api/hmi", tags=["hmi"])


class RemoveRackRequest(BaseModel):
    location_id: int
    device_id: str
    operator_note: Optional[str] = None


class RemoveRackResponse(BaseModel):
    success: bool
    message: str
    rack_name: Optional[str] = None
    location_name: Optional[str] = None
    timestamp: Optional[datetime] = None


@router.post("/remove_rack", response_model=RemoveRackResponse)
async def remove_rack(request: RemoveRackRequest):
    """
    從指定 Location 移出 Rack

    操作邏輯：
    1. 驗證權限: permissions.can_remove_rack
    2. 查詢 Location 和關聯的 Rack
    3. 設定 Rack.location_id = None
    4. 設定 Location.rack_id = None
    5. 更新 Location.location_status_id = UNOCCUPIED
    6. 記錄操作日誌
    """

    try:
        # 【新增】驗證權限
        verify_hmi_permission(request.device_id, "can_remove_rack")

        with connection_pool.get_session() as session:
            # 1. 查詢 Location
            location = session.exec(
                select(Location).where(Location.id == request.location_id)
            ).first()
            
            if not location:
                raise HTTPException(
                    status_code=404,
                    detail=f"Location {request.location_id} 不存在"
                )
            
            # 2. 查詢是否有 Rack 在這個 Location
            rack = session.exec(
                select(Rack).where(Rack.location_id == request.location_id)
            ).first()
            
            if not rack:
                return RemoveRackResponse(
                    success=False,
                    message=f"Location {location.name} 沒有 Rack",
                    location_name=location.name,
                    timestamp=datetime.now(timezone.utc)
                )
            
            # 3. 移出 Rack
            rack_name = rack.name
            rack_id = rack.id

            # 更新 Rack - 清除 location_id 並標記為不在地圖中
            rack.location_id = None
            rack.is_in_map = 0  # 標記為不在地圖中（觸發 KUKA 出場）

            # 🆕 KUKA 容器出場同步
            try:
                from opui.services.kuka_sync_service import OpuiKukaContainerSync
                kuka_sync = OpuiKukaContainerSync()
                sync_result = kuka_sync.sync_container_exit(rack)
                if sync_result.get("success"):
                    print(f"✅ KUKA 容器出場成功: {rack_name}")
                else:
                    print(f"⚠️ KUKA 容器出場失敗: {sync_result.get('message')}")
            except Exception as kuka_error:
                print(f"⚠️ KUKA 同步異常（不影響 Rack 移出）: {str(kuka_error)}")

            # 更新 Location 狀態
            location.location_status_id = LocationStatus.UNOCCUPIED
            
            # 5. 記錄操作日誌
            # TODO: 加入操作日誌記錄
            # log_entry = OperationLog(
            #     device_id=request.device_id,
            #     action="REMOVE_RACK",
            #     location_id=request.location_id,
            #     rack_id=rack.id,
            #     note=request.operator_note,
            #     timestamp=datetime.now(timezone.utc)
            # )
            # session.add(log_entry)

            # 6. 提交變更
            session.commit()

            # 建立回應訊息
            message = f"成功從 Location {location.name} 移出 Rack {rack_name}"
            
            return RemoveRackResponse(
                success=True,
                message=message,
                rack_name=rack_name,
                location_name=location.name,
                timestamp=datetime.now(timezone.utc)
            )
            
    except HTTPException:
        raise
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(
            status_code=500,
            detail=f"移出 Rack 失敗: {str(e)}"
        )


@router.get("/location/{location_id}/status")
async def get_location_status(location_id: int):
    """取得指定 Location 的狀態"""
    
    try:
        with connection_pool.get_session() as session:
            location = session.exec(
                select(Location).where(Location.id == location_id)
            ).first()
            
            if not location:
                raise HTTPException(
                    status_code=404,
                    detail=f"Location {location_id} 不存在"
                )
            
            # 查詢 Rack 資訊 - 使用 Rack.location_id
            rack = session.exec(
                select(Rack).where(Rack.location_id == location.id)
            ).first()
            
            return {
                "location_id": location.id,
                "location_name": location.name,
                "has_rack": rack is not None,
                "rack_name": rack.name if rack else None,
                "rack_id": rack.id if rack else None,
                "status": "OCCUPIED" if rack else "UNOCCUPIED"
            }
            
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"查詢 Location 狀態失敗: {str(e)}"
        )


# ============================================================================
# 權限驗證中間件
# ============================================================================

def verify_hmi_permission(device_id: str, permission_name: str) -> bool:
    """
    驗證 HMI 設備權限

    Args:
        device_id: 設備 ID
        permission_name: 權限名稱 (如 'can_add_rack', 'can_edit_rack', 'can_remove_rack')

    Returns:
        bool: 是否有權限

    Raises:
        HTTPException: 404 (設備未授權), 403 (設備停用或無權限)
    """
    with connection_pool.get_session() as session:
        license = license_crud.get_by_field(session, "device_id", device_id)

        if not license:
            raise HTTPException(status_code=404, detail=f"設備 {device_id} 未授權")

        if not license.active:
            raise HTTPException(status_code=403, detail=f"設備 {device_id} 已停用")

        permissions = license.permissions or {}
        if not permissions.get(permission_name, False):
            raise HTTPException(
                status_code=403,
                detail=f"設備 {device_id} 無權限: {permission_name}"
            )

        return True


# ============================================================================
# 請求/響應模型
# ============================================================================

class AddRackRequest(BaseModel):
    """加入 Rack 請求"""
    location_id: int
    rack_name: str
    device_id: str


class AddRackResponse(BaseModel):
    """加入 Rack 響應"""
    success: bool
    message: str
    rack_id: Optional[int] = None
    rack_name: Optional[str] = None
    location_name: Optional[str] = None
    timestamp: Optional[datetime] = None


class EditRackRequest(BaseModel):
    """編輯 Rack 請求"""
    location_id: int
    rack_id: int
    device_id: str
    product_id: Optional[int] = None
    carrier_bitmap: Optional[str] = None           # "FFFFFFFF" 格式
    carrier_enable_bitmap: Optional[str] = None    # "FFFFFFFF" 格式
    direction: Optional[int] = None                 # 0-3
    status_id: Optional[int] = None


class EditRackResponse(BaseModel):
    """編輯 Rack 響應"""
    success: bool
    message: str
    rack_id: Optional[int] = None
    timestamp: Optional[datetime] = None


class AvailableRack(BaseModel):
    """可用 Rack 資訊"""
    id: int
    name: str
    product_name: Optional[str] = None
    carrier_count: int


class DeviceRequest(BaseModel):
    """通用設備請求"""
    device_id: str


# ============================================================================
# Add Rack API
# ============================================================================

@router.post("/add_rack", response_model=AddRackResponse)
async def add_rack(request: AddRackRequest):
    """
    加入 Rack 到指定 Location

    操作邏輯：
    1. 驗證權限: permissions.can_add_rack
    2. 檢查 Location 是否為空
    3. 查詢 Rack 記錄
    4. 更新 Rack: location_id, is_in_map=1
    5. 更新 Location: location_status_id=OCCUPIED
    6. 同步 KUKA（如已整合）
    """
    start_time = datetime.now(timezone.utc)
    logger.info(f"[HMI_ADD_RACK] 開始 | location_id={request.location_id} rack_name={request.rack_name} device_id={request.device_id} timestamp={start_time.isoformat()}")

    try:
        # 1. 驗證權限
        verify_hmi_permission(request.device_id, "can_add_rack")
        logger.info(f"[HMI_ADD_RACK] 權限驗證通過 | device_id={request.device_id}")

        with connection_pool.get_session() as session:
            # 2. 查詢 Location
            location = session.exec(
                select(Location).where(Location.id == request.location_id)
            ).first()

            if not location:
                logger.error(f"[HMI_ADD_RACK] Location 不存在 | location_id={request.location_id}")
                raise HTTPException(
                    status_code=404,
                    detail=f"Location {request.location_id} 不存在"
                )

            logger.info(f"[HMI_ADD_RACK] Location 查詢成功 | location_id={location.id} location_name={location.name}")

            # 3. 檢查 Location 是否已有 Rack
            existing_rack = session.exec(
                select(Rack).where(Rack.location_id == request.location_id)
            ).first()

            if existing_rack:
                logger.warning(f"[HMI_ADD_RACK] Location 已有 Rack | location_id={request.location_id} existing_rack={existing_rack.name}")
                return AddRackResponse(
                    success=False,
                    message=f"Location {location.name} 已有 Rack {existing_rack.name}",
                    location_name=location.name,
                    timestamp=datetime.now(timezone.utc)
                )

            # 4. 查詢 Rack
            rack = session.exec(
                select(Rack).where(Rack.name == request.rack_name)
            ).first()

            if not rack:
                logger.error(f"[HMI_ADD_RACK] Rack 不存在 | rack_name={request.rack_name}")
                return AddRackResponse(
                    success=False,
                    message=f"Rack {request.rack_name} 不存在於系統中",
                    timestamp=datetime.now(timezone.utc)
                )

            # 記錄更新前的狀態
            old_location_id = rack.location_id
            old_is_in_map = rack.is_in_map
            logger.info(f"[HMI_ADD_RACK] 更新前狀態 | rack_id={rack.id} rack_name={rack.name} old_location_id={old_location_id} old_is_in_map={old_is_in_map}")

            # 5. 更新 Rack
            rack.location_id = request.location_id
            rack.is_in_map = 1  # 標記為在地圖中

            logger.info(f"[HMI_ADD_RACK] 更新後狀態 | rack_id={rack.id} rack_name={rack.name} new_location_id={rack.location_id} new_is_in_map={rack.is_in_map}")

            # 6. 更新 Location 狀態
            location.location_status_id = LocationStatus.OCCUPIED
            logger.info(f"[HMI_ADD_RACK] Location 狀態已更新 | location_id={location.id} status=OCCUPIED")

            # 7. KUKA 容器入場同步
            try:
                from opui.services.kuka_sync_service import OpuiKukaContainerSync
                kuka_sync = OpuiKukaContainerSync()
                logger.info(f"[HMI_ADD_RACK] 開始 KUKA 同步 | rack_id={rack.id} rack_name={rack.name} location_id={request.location_id}")
                sync_result = kuka_sync.sync_container_entry(rack, request.location_id, session)
                if sync_result.get("success"):
                    logger.info(f"[HMI_ADD_RACK] KUKA 容器入場成功 | rack_name={rack.name} result={sync_result}")
                    print(f"✅ KUKA 容器入場成功: {rack.name}")
                else:
                    logger.warning(f"[HMI_ADD_RACK] KUKA 容器入場失敗 | rack_name={rack.name} result={sync_result}")
                    print(f"⚠️ KUKA 容器入場失敗: {sync_result.get('message')}")
            except Exception as kuka_error:
                logger.error(f"[HMI_ADD_RACK] KUKA 同步異常 | rack_name={rack.name} error={str(kuka_error)}", exc_info=True)
                print(f"⚠️ KUKA 同步異常（不影響 Rack 加入）: {str(kuka_error)}")

            # 8. 提交變更
            session.commit()
            commit_time = datetime.now(timezone.utc)
            logger.info(f"[HMI_ADD_RACK] 資料庫提交成功 | rack_id={rack.id} location_id={rack.location_id} is_in_map={rack.is_in_map} commit_time={commit_time.isoformat()}")

            return AddRackResponse(
                success=True,
                message=f"成功將 Rack {rack.name} 加入到 Location {location.name}",
                rack_id=rack.id,
                rack_name=rack.name,
                location_name=location.name,
                timestamp=commit_time
            )

    except HTTPException:
        raise
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(
            status_code=500,
            detail=f"加入 Rack 失敗: {str(e)}"
        )


# ============================================================================
# Edit Rack API
# ============================================================================

@router.put("/edit_rack", response_model=EditRackResponse)
async def edit_rack(request: EditRackRequest):
    """
    編輯 Rack 屬性

    操作邏輯：
    1. 驗證權限: permissions.can_edit_rack
    2. 查詢 Rack 記錄
    3. 驗證字段合法性
    4. 更新字段: product_id, carrier_bitmap, carrier_enable_bitmap, direction, status_id
    5. 返回更新後的數據
    """
    try:
        # 1. 驗證權限
        verify_hmi_permission(request.device_id, "can_edit_rack")

        with connection_pool.get_session() as session:
            # 2. 查詢 Rack
            rack = rack_crud.get_by_id(session, request.rack_id)

            if not rack:
                raise HTTPException(
                    status_code=404,
                    detail=f"Rack {request.rack_id} 不存在"
                )

            # 3. 更新字段
            if request.product_id is not None:
                rack.product_id = request.product_id

            if request.carrier_bitmap is not None:
                # 驗證格式
                try:
                    int(request.carrier_bitmap, 16)
                    rack.carrier_bitmap = request.carrier_bitmap.upper()
                except ValueError:
                    raise HTTPException(
                        status_code=400,
                        detail=f"無效的 carrier_bitmap 格式: {request.carrier_bitmap}"
                    )

            if request.carrier_enable_bitmap is not None:
                try:
                    int(request.carrier_enable_bitmap, 16)
                    rack.carrier_enable_bitmap = request.carrier_enable_bitmap.upper()
                except ValueError:
                    raise HTTPException(
                        status_code=400,
                        detail=f"無效的 carrier_enable_bitmap 格式: {request.carrier_enable_bitmap}"
                    )

            if request.direction is not None:
                if not 0 <= request.direction <= 3:
                    raise HTTPException(
                        status_code=400,
                        detail=f"無效的 direction 值: {request.direction}，必須為 0-3"
                    )
                rack.direction = request.direction

            if request.status_id is not None:
                rack.status_id = request.status_id

            # 4. 提交變更
            session.add(rack)
            session.commit()

            return EditRackResponse(
                success=True,
                message=f"成功更新 Rack {rack.name} 的資料",
                rack_id=rack.id,
                timestamp=datetime.now(timezone.utc)
            )

    except HTTPException:
        raise
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(
            status_code=500,
            detail=f"編輯 Rack 失敗: {str(e)}"
        )


# ============================================================================
# Available Racks API
# ============================================================================

@router.get("/available_racks")
async def get_available_racks(device_id: str = Query(...)):
    """
    查詢可用的 Rack 列表（用於 Add Rack 下拉列表）

    條件: is_in_map=0 且不在任何 Location

    Returns:
        {
            "success": bool,
            "racks": [
                {
                    "id": int,
                    "name": str,
                    "product_name": str,
                    "carrier_count": int
                }
            ]
        }
    """
    try:
        # 驗證設備授權（不需要特定權限，只需授權即可）
        with connection_pool.get_session() as session:
            license = license_crud.get_by_field(session, "device_id", device_id)
            if not license:
                raise HTTPException(status_code=404, detail=f"設備 {device_id} 未授權")
            if not license.active:
                raise HTTPException(status_code=403, detail=f"設備 {device_id} 已停用")

            # 查詢可用 Rack
            racks = session.exec(
                select(Rack).where(
                    (Rack.is_in_map == 0) | (Rack.location_id == None)
                )
            ).all()

            available_racks = []
            for rack in racks:
                # 獲取產品資訊
                product = product_crud.get_by_id(session, rack.product_id) if rack.product_id else None

                # 計算載具數量
                carrier_bitmap = rack.carrier_bitmap or "00000000"
                carrier_count = count_occupied_slots(carrier_bitmap)

                available_racks.append({
                    "id": rack.id,
                    "name": rack.name,
                    "product_name": product.name if product else "無產品",
                    "carrier_count": carrier_count
                })

            return {
                "success": True,
                "racks": available_racks
            }

    except HTTPException:
        raise
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(
            status_code=500,
            detail=f"查詢可用 Rack 失敗: {str(e)}"
        )


@router.get("/products")
async def get_all_products(device_id: str = Query(...)):
    """
    獲取所有產品列表（用於 Edit Rack 產品下拉選單）

    Returns:
        {
            "success": bool,
            "products": [
                {
                    "id": int,
                    "name": str,
                    "size": str
                }
            ]
        }
    """
    try:
        # 驗證設備授權
        with connection_pool.get_session() as session:
            license = license_crud.get_by_field(session, "device_id", device_id)
            if not license:
                raise HTTPException(status_code=404, detail=f"設備 {device_id} 未授權")
            if not license.active:
                raise HTTPException(status_code=403, detail=f"設備 {device_id} 已停用")

            # 查詢所有產品
            products = session.exec(select(Product)).all()

            products_list = []
            for product in products:
                products_list.append({
                    "id": product.id,
                    "name": product.name,
                    "size": product.size or ""
                })

            return {
                "success": True,
                "products": products_list
            }

    except HTTPException:
        raise
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(
            status_code=500,
            detail=f"查詢產品列表失敗: {str(e)}"
        )


@router.get("/rack_statuses")
async def get_all_rack_statuses(device_id: str = Query(...)):
    """
    獲取所有 Rack 狀態列表（用於 Edit Rack 狀態下拉選單）

    Returns:
        {
            "success": bool,
            "statuses": [
                {
                    "id": int,
                    "name": str,
                    "description": str
                }
            ]
        }
    """
    try:
        # 驗證設備授權
        with connection_pool.get_session() as session:
            license = license_crud.get_by_field(session, "device_id", device_id)
            if not license:
                raise HTTPException(status_code=404, detail=f"設備 {device_id} 未授權")
            if not license.active:
                raise HTTPException(status_code=403, detail=f"設備 {device_id} 已停用")

            # 查詢所有 Rack 狀態
            statuses = session.exec(select(RackStatus)).all()

            statuses_list = []
            for status in statuses:
                statuses_list.append({
                    "id": status.id,
                    "name": status.name,
                    "description": status.description or ""
                })

            return {
                "success": True,
                "statuses": statuses_list
            }

    except HTTPException:
        raise
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(
            status_code=500,
            detail=f"查詢 Rack 狀態列表失敗: {str(e)}"
        )


# ============================================================================
# Bitmap Management APIs (複用 AGVCUI 實作)
# ============================================================================

@router.get("/racks/{rack_id}/bitmap-status")
async def get_rack_bitmap_status(rack_id: int, device_id: str = Query(...)):
    """
    獲取 Rack 的格位狀態

    Returns:
        {
            "success": bool,
            "data": {
                "id": int,
                "name": str,
                "carrier_bitmap": str,
                "carrier_enable_bitmap": str,
                "product_size": str,
                "product_name": str,
                "max_slots": int,
                "bitmap_status": {
                    "1": {"occupied": bool, "enabled": bool},
                    ...
                },
                "stats": {
                    "occupied_count": int,
                    "enabled_count": int,
                    "empty_enabled_count": int,
                    "empty_enabled_slots": [int]
                }
            }
        }
    """
    try:
        # 驗證設備授權
        with connection_pool.get_session() as session:
            license = license_crud.get_by_field(session, "device_id", device_id)
            if not license:
                raise HTTPException(status_code=404, detail=f"設備 {device_id} 未授權")

        # 獲取 Rack bitmap 狀態
        data = get_rack_with_bitmap_status(rack_id)

        if not data:
            raise HTTPException(status_code=404, detail=f"Rack {rack_id} 不存在")

        # 轉換 bitmap_status 的鍵為字符串（JSON 相容）
        bitmap_status_str = {str(k): v for k, v in data["bitmap_status"].items()}
        data["bitmap_status"] = bitmap_status_str

        return {
            "success": True,
            "data": data
        }

    except HTTPException:
        raise
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(
            status_code=500,
            detail=f"獲取 Bitmap 狀態失敗: {str(e)}"
        )


@router.post("/racks/{rack_id}/slot/{slot_index}/occupy")
async def occupy_slot(rack_id: int, slot_index: int, request: DeviceRequest):
    """設置格位為有貨"""
    try:
        # 驗證權限
        verify_hmi_permission(request.device_id, "can_edit_rack")

        # 執行操作
        success = set_slot_occupied(rack_id, slot_index)

        if not success:
            raise HTTPException(status_code=400, detail="操作失敗")

        return {
            "success": True,
            "message": f"Rack {rack_id} 格位 {slot_index} 已設為有貨",
            "rack_id": rack_id,
            "slot_index": slot_index
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"操作失敗: {str(e)}")


@router.post("/racks/{rack_id}/slot/{slot_index}/empty")
async def empty_slot(rack_id: int, slot_index: int, request: DeviceRequest):
    """設置格位為空"""
    try:
        verify_hmi_permission(request.device_id, "can_edit_rack")

        success = set_slot_empty(rack_id, slot_index)

        if not success:
            raise HTTPException(status_code=400, detail="操作失敗")

        return {
            "success": True,
            "message": f"Rack {rack_id} 格位 {slot_index} 已設為空",
            "rack_id": rack_id,
            "slot_index": slot_index
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"操作失敗: {str(e)}")


@router.post("/racks/{rack_id}/slot/{slot_index}/toggle-enable")
async def toggle_slot_enable(rack_id: int, slot_index: int, request: DeviceRequest):
    """切換格位啟用狀態"""
    try:
        verify_hmi_permission(request.device_id, "can_edit_rack")

        success = toggle_slot_enabled(rack_id, slot_index)

        if not success:
            raise HTTPException(status_code=400, detail="操作失敗")

        # 獲取當前狀態
        data = get_rack_with_bitmap_status(rack_id)
        enabled = data["bitmap_status"][slot_index]["enabled"]

        return {
            "success": True,
            "message": f"Rack {rack_id} 格位 {slot_index} 已{'啟用' if enabled else '禁用'}",
            "rack_id": rack_id,
            "slot_index": slot_index,
            "enabled": enabled
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"操作失敗: {str(e)}")


@router.post("/racks/{rack_id}/slots/clear-all")
async def clear_all_rack_slots(rack_id: int, request: DeviceRequest):
    """清空所有格位"""
    try:
        verify_hmi_permission(request.device_id, "can_edit_rack")

        success = clear_all_slots(rack_id)

        if not success:
            raise HTTPException(status_code=400, detail="操作失敗")

        return {
            "success": True,
            "message": f"Rack {rack_id} 所有格位已清空",
            "rack_id": rack_id
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"操作失敗: {str(e)}")


@router.post("/racks/{rack_id}/slots/enable-all")
async def enable_all_rack_slots(rack_id: int, request: DeviceRequest, product_size: str = Query("S")):
    """啟用所有格位"""
    try:
        verify_hmi_permission(request.device_id, "can_edit_rack")

        success = enable_all_slots(rack_id, product_size)

        if not success:
            raise HTTPException(status_code=400, detail="操作失敗")

        return {
            "success": True,
            "message": f"Rack {rack_id} 所有格位已啟用（產品尺寸: {product_size}）",
            "rack_id": rack_id,
            "product_size": product_size
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"操作失敗: {str(e)}")


@router.post("/racks/{rack_id}/slots/disable-all")
async def disable_all_rack_slots(rack_id: int, request: DeviceRequest):
    """禁用所有格位"""
    try:
        verify_hmi_permission(request.device_id, "can_edit_rack")

        success = disable_all_slots(rack_id)

        if not success:
            raise HTTPException(status_code=400, detail="操作失敗")

        return {
            "success": True,
            "message": f"Rack {rack_id} 所有格位已禁用",
            "rack_id": rack_id
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"操作失敗: {str(e)}")