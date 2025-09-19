"""
Flow Functions API Router
[DEPRECATED] - 原本提供 flow_wcs 函數庫資訊給 Linear Flow Designer
現已改用 TAFL 系統，此檔案暫時保留但已停用
"""

from fastapi import APIRouter, HTTPException
from typing import Dict, List, Any

router = APIRouter()

@router.get("/api/flow/functions")
async def get_flow_functions() -> Dict[str, Any]:
    """
    [DEPRECATED] - 此 API 已停用
    原本從 flow_wcs 獲取函數庫，現已改用 TAFL 系統

    Returns:
        錯誤訊息，提示系統已升級
    """
    raise HTTPException(
        status_code=410,  # Gone
        detail="Flow WCS API has been deprecated. Please use TAFL WCS system instead."
    )

@router.get("/api/flow/functions/flat")
async def get_flat_functions() -> Dict[str, Any]:
    """
    [DEPRECATED] - 此 API 已停用
    """
    raise HTTPException(
        status_code=410,
        detail="Flow WCS API has been deprecated. Please use TAFL WCS system instead."
    )

@router.get("/api/flow/categories")
async def get_function_categories() -> Dict[str, Any]:
    """
    [DEPRECATED] - 此 API 已停用
    """
    raise HTTPException(
        status_code=410,
        detail="Flow WCS API has been deprecated. Please use TAFL WCS system instead."
    )

@router.get("/api/flow/functions/{category}")
async def get_functions_by_category(category: str) -> Dict[str, Any]:
    """
    [DEPRECATED] - 此 API 已停用
    """
    raise HTTPException(
        status_code=410,
        detail="Flow WCS API has been deprecated. Please use TAFL WCS system instead."
    )

@router.get("/api/flow/function/{function_name}")
async def get_function_details(function_name: str) -> Dict[str, Any]:
    """
    [DEPRECATED] - 此 API 已停用
    """
    raise HTTPException(
        status_code=410,
        detail="Flow WCS API has been deprecated. Please use TAFL WCS system instead."
    )

@router.post("/api/flow/validate")
async def validate_function_call(request: Dict[str, Any]) -> Dict[str, Any]:
    """
    [DEPRECATED] - 此 API 已停用
    """
    raise HTTPException(
        status_code=410,
        detail="Flow WCS API has been deprecated. Please use TAFL WCS system instead."
    )