from datetime import datetime
from zoneinfo import ZoneInfo
from fastapi import APIRouter, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from typing import Optional
from agvcui.db import (
    get_signals, count_signals, get_signals_by_eqp_id,
    count_signals_by_eqp_id, get_eqps_with_signal_counts, get_eqp_by_id
)


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/signals", response_class=HTMLResponse)
    async def signals_list(request: Request, page: int = 1, eqp_id: Optional[int] = None):
        from agvcui.middleware import get_current_user_from_request

        limit = 20
        offset = (page - 1) * limit

        # 獲取所有設備及其信號數量
        eqps_with_counts = get_eqps_with_signal_counts()

        # 根據是否指定設備ID來獲取信號
        if eqp_id:
            signals = get_signals_by_eqp_id(eqp_id, offset=offset, limit=limit)
            total = count_signals_by_eqp_id(eqp_id)
            selected_eqp = get_eqp_by_id(eqp_id)
        else:
            signals = get_signals(offset=offset, limit=limit)
            total = count_signals()
            selected_eqp = None

        total_pages = (total + limit - 1) // limit
        taipei_time = datetime.now(ZoneInfo("Asia/Taipei"))
        latest_query = taipei_time.strftime("%Y/%m/%d %H:%M:%S")
        current_user = get_current_user_from_request(request)

        return templates.TemplateResponse("signals.html", {
            "request": request,
            "signals": signals,
            "eqps_with_counts": eqps_with_counts,
            "selected_eqp": selected_eqp,
            "selected_eqp_id": eqp_id,
            "active_tab": "signals",
            "latest_query": latest_query,
            "current_page": page,
            "total_pages": total_pages,
            "total_count": total,
            "current_user": current_user
        })
    return router
