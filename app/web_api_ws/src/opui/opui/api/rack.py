"""
Rack API 路由模組
處理料架相關的 API 請求
"""

from fastapi import APIRouter, HTTPException, Query
from typing import List, Optional
from pydantic import BaseModel
from opui.database.operations import connection_pool
from db_proxy.models import Rack
from sqlmodel import select

router = APIRouter(prefix="/api/rack", tags=["rack"])


class RackInfo(BaseModel):
    """料架資訊模型"""
    id: int
    name: str
    product_id: Optional[int] = None
    location_id: Optional[int] = None


class AvailableRacksResponse(BaseModel):
    """可用料架回應模型"""
    success: bool
    racks: List[RackInfo]
    message: Optional[str] = None


@router.get("/available", response_model=AvailableRacksResponse)
async def get_available_racks():
    """
    獲取所有可用的料架（location_id = null）
    
    Returns:
        AvailableRacksResponse: 包含所有可用料架的列表
    """
    try:
        with connection_pool.get_session() as session:
            # 查詢所有 location_id 為 null 的料架
            available_racks = session.exec(
                select(Rack).where(Rack.location_id == None).order_by(Rack.name)
            ).all()
            
            # 轉換為回應格式
            rack_list = [
                RackInfo(
                    id=rack.id,
                    name=rack.name,
                    product_id=rack.product_id,
                    location_id=rack.location_id
                )
                for rack in available_racks
            ]
            
            return AvailableRacksResponse(
                success=True,
                racks=rack_list,
                message=f"找到 {len(rack_list)} 個可用料架"
            )
            
    except Exception as e:
        print(f"Error fetching available racks: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"獲取可用料架失敗: {str(e)}"
        )


@router.get("/{rack_id}", response_model=RackInfo)
async def get_rack_by_id(rack_id: int):
    """
    根據 ID 獲取料架資訊
    
    Args:
        rack_id: 料架 ID
        
    Returns:
        RackInfo: 料架資訊
    """
    try:
        with connection_pool.get_session() as session:
            rack = session.exec(
                select(Rack).where(Rack.id == rack_id)
            ).first()
            
            if not rack:
                raise HTTPException(
                    status_code=404,
                    detail=f"料架 ID {rack_id} 不存在"
                )
            
            return RackInfo(
                id=rack.id,
                name=rack.name,
                product_id=rack.product_id,
                location_id=rack.location_id
            )
            
    except HTTPException:
        raise
    except Exception as e:
        print(f"Error fetching rack {rack_id}: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"獲取料架資訊失敗: {str(e)}"
        )


@router.get("/by-name/{rack_name}", response_model=RackInfo)
async def get_rack_by_name(rack_name: str):
    """
    根據名稱獲取料架資訊
    
    Args:
        rack_name: 料架名稱
        
    Returns:
        RackInfo: 料架資訊
    """
    try:
        with connection_pool.get_session() as session:
            rack = session.exec(
                select(Rack).where(Rack.name == rack_name)
            ).first()
            
            if not rack:
                raise HTTPException(
                    status_code=404,
                    detail=f"料架 {rack_name} 不存在"
                )
            
            return RackInfo(
                id=rack.id,
                name=rack.name,
                product_id=rack.product_id,
                location_id=rack.location_id
            )
            
    except HTTPException:
        raise
    except Exception as e:
        print(f"Error fetching rack {rack_name}: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"獲取料架資訊失敗: {str(e)}"
        )