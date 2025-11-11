"""
HMI API 路由模組
處理 HMI 介面的 Rack 移出操作
"""

from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel
from typing import Optional
from datetime import datetime, timezone
from opui.database.operations import connection_pool
from db_proxy.models import Location, Rack, LocationStatus
from sqlmodel import select, delete

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
    1. 查詢 Location 和關聯的 Rack
    2. 設定 Rack.location_id = None
    3. 設定 Location.rack_id = None
    4. 更新 Location.location_status_id = UNOCCUPIED
    5. 記錄操作日誌
    """
    
    try:
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