import re
from typing import Optional
from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse, HTMLResponse
from fastapi.templating import Jinja2Templates
from starlette.middleware.base import BaseHTTPMiddleware
from opui.database.operations import connection_pool
from db_proxy.crud.license_crud import license_crud
import os


class DeviceAuthMiddleware(BaseHTTPMiddleware):
    """設備授權驗證中間件"""

    def __init__(self, app, included_paths: Optional[list] = None):
        super().__init__(app)
        # 需要設備授權驗證的路徑（只有這些路徑需要授權）
        self.included_paths = included_paths or [
            "/home",
            "/settings",
            "/rack"
        ]
    
    async def dispatch(self, request: Request, call_next):
        """處理請求的中間件邏輯"""

        # 檢查是否為需要授權的路徑
        if not self._is_included_path(request.url.path):
            return await call_next(request)

        # 驗證設備授權
        auth_result = await self._validate_device_authorization(request)
        if not auth_result["success"]:
            return JSONResponse(
                status_code=auth_result["status_code"],
                content={
                    "error": auth_result["error"],
                    "message": auth_result["message"]
                }
            )

        # 授權通過，繼續處理請求
        return await call_next(request)
    
    def _is_included_path(self, path: str) -> bool:
        """檢查路徑是否需要設備授權驗證"""
        for included_path in self.included_paths:
            if path.startswith(included_path):
                return True
        return False
    
    async def _validate_device_authorization(self, request: Request) -> dict:
        """驗證設備授權"""
        
        # 1. 檢查是否有 deviceId 參數
        device_id = request.query_params.get("deviceId")
        if not device_id:
            return {
                "success": False,
                "status_code": 400,
                "error": "Missing deviceId parameter",
                "message": "請求必須包含 deviceId 參數"
            }
        
        # 2. 驗證 deviceId 格式
        if not self._validate_device_id_format(device_id):
            return {
                "success": False,
                "status_code": 400,
                "error": "Invalid deviceId format",
                "message": "deviceId 必須為恰好16個字符的英數字串"
            }
        
        # 3. 查詢資料庫驗證授權
        try:
            with connection_pool.get_session() as session:
                license_data = license_crud.get_by_device_id(session, device_id)
                
                if not license_data:
                    return {
                        "success": False,
                        "status_code": 403,
                        "error": "Device not authorized",
                        "message": f"設備 {device_id} 未授權"
                    }
                
                if license_data.active != 1:
                    return {
                        "success": False,
                        "status_code": 403,
                        "error": "Device license disabled",
                        "message": f"設備 {device_id} 授權已停用"
                    }
                
                return {
                    "success": True,
                    "device_id": device_id,
                    "license_data": license_data
                }
                
        except Exception as e:
            return {
                "success": False,
                "status_code": 500,
                "error": "Database error",
                "message": f"資料庫查詢錯誤: {str(e)}"
            }
    
    def _validate_device_id_format(self, device_id: str) -> bool:
        """驗證 deviceId 格式：恰好16個字符，只能包含英文字母和數字"""
        if len(device_id) != 16:
            return False
        
        # 檢查是否只包含英文字母（大小寫）和數字
        pattern = r'^[a-zA-Z0-9]{16}$'
        return bool(re.match(pattern, device_id))


def validate_device_id_format(device_id: str) -> bool:
    """獨立的 deviceId 格式驗證函數"""
    if len(device_id) != 16:
        return False
    
    pattern = r'^[a-zA-Z0-9]{16}$'
    return bool(re.match(pattern, device_id))


async def check_device_authorization(device_id: str) -> dict:
    """獨立的設備授權檢查函數"""
    
    # 驗證格式
    if not validate_device_id_format(device_id):
        return {
            "success": False,
            "error": "Invalid deviceId format",
            "message": "deviceId 必須為恰好16個字符的英數字串"
        }
    
    # 查詢資料庫
    try:
        with connection_pool.get_session() as session:
            license_data = license_crud.get_by_device_id(session, device_id)
            
            if not license_data:
                return {
                    "success": False,
                    "error": "Device not authorized",
                    "message": f"設備 {device_id} 未授權"
                }
            
            if license_data.active != 1:
                return {
                    "success": False,
                    "error": "Device license disabled",
                    "message": f"設備 {device_id} 授權已停用"
                }
            
            return {
                "success": True,
                "device_id": device_id,
                "license_data": license_data
            }
            
    except Exception as e:
        return {
            "success": False,
            "error": "Database error",
            "message": f"資料庫查詢錯誤: {str(e)}"
        }
