from fastapi import APIRouter, HTTPException
from typing import List, Optional
from opui.database.operations import connection_pool
from db_proxy.models import License
from db_proxy.crud.license_crud import license_crud
from pydantic import BaseModel

router = APIRouter(tags=["License"])


class LicenseRequest(BaseModel):
    device_id: str
    active: int


class LicenseResponse(BaseModel):
    success: bool
    data: Optional[dict] = None
    message: str = ""


@router.get("/licenses", response_model=List[License])
def list_licenses():
    """取得所有 License 資料"""
    with connection_pool.get_session() as session:
        return license_crud.get_all(session)


@router.get("/licenses/{license_id}", response_model=License)
def get_license(license_id: int):
    """根據 ID 取得 License 資料"""
    with connection_pool.get_session() as session:
        license_data = license_crud.get_by_id(session, license_id)
        if not license_data:
            raise HTTPException(status_code=404, detail="License not found")
        return license_data


@router.get("/license/device/{device_id}")
def get_license_by_device_id(device_id: str):
    """根據 device_id 查詢該設備的 active 狀態"""
    with connection_pool.get_session() as session:
        license_data = license_crud.get_by_device_id(session, device_id)
        
        if not license_data:
            return LicenseResponse(
                success=False,
                data=None,
                message=f"Device ID '{device_id}' not found"
            ).model_dump()
        
        return LicenseResponse(
            success=True,
            data={
                "device_id": license_data.device_id,
                "active": license_data.active
            },
            message="License found successfully"
        ).model_dump()


@router.post("/licenses", response_model=License)
def create_license(request: LicenseRequest):
    """建立新的 License 資料"""
    with connection_pool.get_session() as session:
        # 檢查是否已存在相同的 device_id
        existing_license = license_crud.get_by_device_id(session, request.device_id)
        if existing_license:
            raise HTTPException(
                status_code=400, 
                detail=f"License with device_id '{request.device_id}' already exists"
            )
        
        new_license = License(
            device_id=request.device_id,
            active=request.active
        )
        license_data = license_crud.create(session, new_license)
        return license_data


@router.put("/licenses/{license_id}", response_model=License)
def update_license(license_id: int, request: LicenseRequest):
    """更新 License 資料"""
    with connection_pool.get_session() as session:
        updated = license_crud.update(session, license_id, request.model_dump())
        if not updated:
            raise HTTPException(status_code=404, detail="License not found")
        return updated


@router.delete("/licenses/{license_id}")
def delete_license(license_id: int):
    """刪除 License 資料"""
    with connection_pool.get_session() as session:
        deleted = license_crud.delete(session, license_id)
        if not deleted:
            raise HTTPException(status_code=404, detail="License not found")
        return {"message": "License deleted successfully"}
