from datetime import datetime
from zoneinfo import ZoneInfo
from typing import Optional
from fastapi import APIRouter, Request, Query, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel
from agvcui.database import (
    create_audit_log, get_audit_logs, count_audit_logs,
    get_user_audit_logs, get_resource_audit_logs
)


class AuditLogCreate(BaseModel):
    operation_id: str
    session_id: str
    timestamp: str
    operation_type: str
    resource_type: str
    resource_id: Optional[str] = None
    user: dict
    details: Optional[dict] = None
    user_agent: Optional[str] = None
    url: Optional[str] = None


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.post("/api/audit_logs")
    async def create_audit_log_api(audit_data: AuditLogCreate):
        """接收前端發送的審計日誌"""
        try:
            # 轉換時間格式
            timestamp = datetime.fromisoformat(audit_data.timestamp.replace('Z', '+00:00'))
            
            # 準備資料
            operation_data = {
                "operation_id": audit_data.operation_id,
                "session_id": audit_data.session_id,
                "timestamp": timestamp,
                "operation_type": audit_data.operation_type,
                "resource_type": audit_data.resource_type,
                "resource_id": audit_data.resource_id,
                "username": audit_data.user.get("username"),
                "user_role": audit_data.user.get("role"),
                "is_logged_in": audit_data.user.get("isLoggedIn", False),
                "details": audit_data.details,
                "user_agent": audit_data.user_agent,
                "url": audit_data.url
            }
            
            success = create_audit_log(operation_data)
            
            if success:
                return {"status": "success", "message": "Audit log created"}
            else:
                raise HTTPException(status_code=500, detail="Failed to create audit log")
                
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"Invalid data: {str(e)}")

    @router.get("/audit_logs", response_class=HTMLResponse)
    async def audit_logs_page(
        request: Request,
        page: int = Query(1, ge=1),
        operation_type: Optional[str] = Query(None),
        resource_type: Optional[str] = Query(None),
        username: Optional[str] = Query(None),
        start_time: Optional[str] = Query(None),
        end_time: Optional[str] = Query(None)
    ):
        """審計日誌頁面"""
        from agvcui.middleware import get_current_user_from_request
        
        # 處理時間篩選
        start_datetime = None
        end_datetime = None
        if start_time:
            try:
                start_datetime = datetime.strptime(start_time, "%Y-%m-%d").replace(
                    tzinfo=ZoneInfo("Asia/Taipei"))
            except ValueError:
                pass
        
        if end_time:
            try:
                end_datetime = datetime.strptime(end_time, "%Y-%m-%d").replace(
                    hour=23, minute=59, second=59, tzinfo=ZoneInfo("Asia/Taipei"))
            except ValueError:
                pass

        limit = 20
        offset = (page - 1) * limit

        # 獲取審計日誌
        logs = get_audit_logs(
            offset=offset,
            limit=limit,
            operation_type=operation_type,
            resource_type=resource_type,
            username=username,
            start_time=start_datetime,
            end_time=end_datetime
        )

        total = count_audit_logs(
            operation_type=operation_type,
            resource_type=resource_type,
            username=username,
            start_time=start_datetime,
            end_time=end_datetime
        )

        total_pages = (total + limit - 1) // limit
        current_user = get_current_user_from_request(request)

        # 轉換時區
        for log in logs:
            if log.timestamp:
                log.timestamp = log.timestamp.replace(
                    tzinfo=ZoneInfo('UTC')).astimezone(ZoneInfo('Asia/Taipei'))

        return templates.TemplateResponse("audit_logs.html", {
            "request": request,
            "active_tab": "audit_logs",
            "logs": logs,
            "current_page": page,
            "total_pages": total_pages,
            "total": total,
            "current_user": current_user,
            "filters": {
                "operation_type": operation_type,
                "resource_type": resource_type,
                "username": username,
                "start_time": start_time,
                "end_time": end_time
            }
        })

    @router.get("/api/audit_logs")
    async def get_audit_logs_api(
        page: int = Query(1, ge=1),
        operation_type: Optional[str] = Query(None),
        resource_type: Optional[str] = Query(None),
        username: Optional[str] = Query(None)
    ):
        """API 獲取審計日誌"""
        limit = 20
        offset = (page - 1) * limit
        
        logs = get_audit_logs(
            offset=offset,
            limit=limit,
            operation_type=operation_type,
            resource_type=resource_type,
            username=username
        )
        
        total = count_audit_logs(
            operation_type=operation_type,
            resource_type=resource_type,
            username=username
        )
        
        # 轉換為字典格式
        logs_data = []
        for log in logs:
            log_dict = {
                "id": log.id,
                "operation_id": log.operation_id,
                "session_id": log.session_id,
                "timestamp": log.timestamp.isoformat() if log.timestamp else None,
                "operation_type": log.operation_type,
                "resource_type": log.resource_type,
                "resource_id": log.resource_id,
                "username": log.username,
                "user_role": log.user_role,
                "is_logged_in": log.is_logged_in,
                "details": log.details,
                "user_agent": log.user_agent,
                "url": log.url,
                "created_at": log.created_at.isoformat() if log.created_at else None
            }
            logs_data.append(log_dict)
        
        return {
            "logs": logs_data,
            "total": total,
            "page": page,
            "limit": limit
        }

    return router
