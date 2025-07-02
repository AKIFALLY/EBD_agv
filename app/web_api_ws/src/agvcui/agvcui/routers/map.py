from fastapi import APIRouter, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/map", response_class=HTMLResponse)
    async def map_page(request: Request):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)
        return templates.TemplateResponse("map.html", {
            "request": request,
            "active_tab": "map",
            "current_user": current_user
        })

    return router
