from datetime import datetime
from zoneinfo import ZoneInfo
from typing import Optional
from fastapi import APIRouter, Request, Query
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from agvcui.db import get_runtime_logs, count_runtime_logs, get_runtime_node_names


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/runtime_logs", response_class=HTMLResponse)
    async def runtime_logs(
        request: Request,
        page: int = Query(1, ge=1),
        level: Optional[str] = Query(None),
        node_name: Optional[str] = Query(None),
        start_time: Optional[str] = Query(None),
        end_time: Optional[str] = Query(None),
        message_filter: Optional[str] = Query(None)
    ):
        from agvcui.middleware import get_current_user_from_request

        try:
            # 解析級別參數
            level_int = None
            if level and level.strip():
                try:
                    level_int = int(level)
                except ValueError:
                    pass

            # 處理空字符串參數
            node_name = node_name.strip() if node_name else None
            if node_name == "":
                node_name = None

            message_filter = message_filter.strip() if message_filter else None
            if message_filter == "":
                message_filter = None

            # 解析時間參數
            start_datetime = None
            end_datetime = None
            if start_time and start_time.strip():
                try:
                    start_datetime = datetime.fromisoformat(start_time)
                except ValueError:
                    pass
            if end_time and end_time.strip():
                try:
                    end_datetime = datetime.fromisoformat(end_time)
                except ValueError:
                    pass

            limit = 20
            offset = (page - 1) * limit

            # 獲取日誌數據
            logs = get_runtime_logs(
                offset=offset,
                limit=limit,
                level=level_int,
                node_name=node_name,
                start_time=start_datetime,
                end_time=end_datetime,
                message_filter=message_filter
            )

            total = count_runtime_logs(
                level=level_int,
                node_name=node_name,
                start_time=start_datetime,
                end_time=end_datetime,
                message_filter=message_filter
            )

            total_pages = (total + limit - 1) // limit
            taipei_time = datetime.now(ZoneInfo("Asia/Taipei"))
            latest_query = taipei_time.strftime("%Y/%m/%d %H:%M:%S")
            current_user = get_current_user_from_request(request)

            # 轉換時區
            for log in logs:
                if log.timestamp:
                    log.timestamp = log.timestamp.replace(
                        tzinfo=ZoneInfo('UTC')).astimezone(ZoneInfo('Asia/Taipei'))

            # 獲取所有節點名稱用於篩選下拉選單
            node_names = get_runtime_node_names()

            # 構建分頁 URL 的函數
            def build_pagination_url(page_num):
                params = {"page": page_num}
                if level_int is not None:
                    params["level"] = level_int
                if node_name:
                    params["node_name"] = node_name
                if start_time:
                    params["start_time"] = start_time
                if end_time:
                    params["end_time"] = end_time
                if message_filter:
                    params["message_filter"] = message_filter

                query_string = "&".join(
                    [f"{k}={v}" for k, v in params.items()])
                return f"/runtime_logs?{query_string}"

            return templates.TemplateResponse("runtime_logs.html", {
                "request": request,
                "logs": logs,
                "active_tab": "runtime_logs",
                "latest_query": latest_query,
                "current_page": page,
                "total_pages": total_pages,
                "total_count": total,
                "node_names": node_names,
                "filters": {
                    "level": level_int,
                    "node_name": node_name,
                    "start_time": start_time,
                    "end_time": end_time,
                    "message_filter": message_filter
                },
                "build_pagination_url": build_pagination_url,
                "current_user": current_user
            })

        except Exception as e:
            print(f"Error in runtime_logs: {e}")
            return templates.TemplateResponse("runtime_logs.html", {
                "request": request,
                "logs": [],
                "active_tab": "runtime_logs",
                "latest_query": datetime.now(ZoneInfo("Asia/Taipei")).strftime("%Y/%m/%d %H:%M:%S"),
                "current_page": 1,
                "total_pages": 1,
                "total_count": 0,
                "node_names": [],
                "filters": {},
                "build_pagination_url": lambda x: f"/runtime_logs?page={x}",
                "current_user": get_current_user_from_request(request)
            })

    return router
