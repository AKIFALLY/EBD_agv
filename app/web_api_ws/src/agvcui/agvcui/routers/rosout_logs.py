from datetime import datetime
from zoneinfo import ZoneInfo
from typing import Optional
from fastapi import APIRouter, Request, Query
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from agvcui.db import get_rosout_logs, count_rosout_logs, get_rosout_node_names


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/rosout_logs", response_class=HTMLResponse)
    async def rosout_logs(
        request: Request,
        page: int = 1,
        level: Optional[str] = Query(None, description="日誌級別篩選"),
        node_name: Optional[str] = Query(None, description="節點名稱篩選"),
        start_time: Optional[str] = Query(None, description="開始時間篩選"),
        end_time: Optional[str] = Query(None, description="結束時間篩選"),
        message_filter: Optional[str] = Query(None, description="消息內容篩選")
    ):
        from agvcui.middleware import get_current_user_from_request

        # 處理級別篩選
        level_int = None
        if level and level.strip():
            try:
                level_int = int(level)
            except ValueError:
                pass

        # 處理節點名稱篩選
        node_name_clean = node_name.strip() if node_name else None
        if node_name_clean == "":
            node_name_clean = None

        # 處理消息篩選
        message_filter_clean = message_filter.strip() if message_filter else None
        if message_filter_clean == "":
            message_filter_clean = None

        # 處理時間篩選
        start_datetime = None
        end_datetime = None
        if start_time:
            try:
                start_datetime = datetime.fromisoformat(
                    start_time.replace('Z', '+00:00'))
            except ValueError:
                pass
        if end_time:
            try:
                end_datetime = datetime.fromisoformat(
                    end_time.replace('Z', '+00:00'))
            except ValueError:
                pass

        limit = 20
        offset = (page - 1) * limit

        # 使用篩選條件查詢
        logs = get_rosout_logs(
            offset=offset,
            limit=limit,
            level=level_int,
            node_name=node_name_clean,
            start_time=start_datetime,
            end_time=end_datetime,
            message_filter=message_filter_clean
        )

        total = count_rosout_logs(
            level=level_int,
            node_name=node_name_clean,
            start_time=start_datetime,
            end_time=end_datetime,
            message_filter=message_filter_clean
        )

        total_pages = (total + limit - 1) // limit
        taipei_time = datetime.now(ZoneInfo("Asia/Taipei"))
        latest_query = taipei_time.strftime("%Y/%m/%d %H:%M:%S")
        current_user = get_current_user_from_request(request)

        # 獲取所有節點名稱用於篩選選項
        node_names = get_rosout_node_names()

        # 構建分頁 URL 的輔助函數
        def build_pagination_url(page_num):
            params = [f"page={page_num}"]
            if level_int is not None:
                params.append(f"level={level}")
            if node_name_clean:
                params.append(f"node_name={node_name}")
            if start_time and start_time.strip():
                params.append(f"start_time={start_time}")
            if end_time and end_time.strip():
                params.append(f"end_time={end_time}")
            if message_filter_clean:
                params.append(f"message_filter={message_filter}")
            return f"/rosout_logs?{'&'.join(params)}"

        return templates.TemplateResponse("rosout_logs.html", {
            "request": request,
            "logs": logs,
            "active_tab": "rosout_logs",
            "latest_query": latest_query,
            "current_page": page,
            "total_pages": total_pages,
            "total_count": total,
            "current_user": current_user,
            "node_names": node_names,
            "filters": {
                "level": level,
                "node_name": node_name,
                "start_time": start_time,
                "end_time": end_time,
                "message_filter": message_filter
            },
            "build_pagination_url": build_pagination_url
        })

    return router
