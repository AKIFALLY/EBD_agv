from datetime import datetime
from zoneinfo import ZoneInfo
from fastapi import APIRouter, Request, Form, HTTPException
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.templating import Jinja2Templates
from agvcui.db import (
    get_products, count_products, get_product_by_id,
    create_product, update_product, delete_product,
    get_all_process_settings
)


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/products", response_class=HTMLResponse)
    async def products_list(
        request: Request,
        page: int = 1,
        search: str = "",
        size_filter: str = "",
        sort_by: str = "id"
    ):
        from agvcui.middleware import get_current_user_from_request

        limit = 20
        offset = (page - 1) * limit

        # 構建篩選條件
        filters = {}
        if search:
            filters['search'] = search
        if size_filter:
            filters['size'] = size_filter

        products = get_products(
            offset=offset,
            limit=limit,
            filters=filters,
            sort_by=sort_by
        )
        total = count_products(filters=filters)
        total_pages = (total + limit - 1) // limit
        taipei_time = datetime.now(ZoneInfo("Asia/Taipei"))
        latest_query = taipei_time.strftime("%Y/%m/%d %H:%M:%S")
        current_user = get_current_user_from_request(request)

        return templates.TemplateResponse("products.html", {
            "request": request,
            "products": products,
            "active_tab": "products",
            "latest_query": latest_query,
            "current_page": page,
            "total_pages": total_pages,
            "total_count": total,
            "current_user": current_user
        })

    @router.get("/products/create", response_class=HTMLResponse)
    async def product_create_form(request: Request):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 獲取所有製程設置
        process_settings = get_all_process_settings()

        return templates.TemplateResponse("product_form.html", {
            "request": request,
            "active_tab": "products",
            "current_user": current_user,
            "process_settings": process_settings,
            "product": None,  # 新增時為 None
            "form_title": "新增產品",
            "form_action": "/products/create"
        })

    @router.post("/products/create")
    async def product_create(
        request: Request,
        name: str = Form(...),
        size: str = Form(...),
        process_settings_id: int = Form(...)
    ):
        try:
            product_data = {
                "name": name,
                "size": size,
                "process_settings_id": process_settings_id
            }
            create_product(product_data)
            return RedirectResponse(url="/products", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=str(e))

    @router.get("/products/{product_id}/edit", response_class=HTMLResponse)
    async def product_edit_form(request: Request, product_id: int):
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)

        # 獲取產品資料
        product = get_product_by_id(product_id)
        if not product:
            raise HTTPException(status_code=404, detail="產品未找到")

        # 獲取所有製程設置
        process_settings = get_all_process_settings()

        return templates.TemplateResponse("product_form.html", {
            "request": request,
            "active_tab": "products",
            "current_user": current_user,
            "process_settings": process_settings,
            "product": product,
            "form_title": "編輯產品",
            "form_action": f"/products/{product_id}/edit"
        })

    @router.post("/products/{product_id}/edit")
    async def product_edit(
        request: Request,
        product_id: int,
        name: str = Form(...),
        size: str = Form(...),
        process_settings_id: int = Form(...)
    ):
        try:
            product_data = {
                "name": name,
                "size": size,
                "process_settings_id": process_settings_id
            }
            update_product(product_id, product_data)
            return RedirectResponse(url="/products", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=str(e))

    @router.post("/products/{product_id}/delete")
    async def product_delete(product_id: int):
        try:
            delete_product(product_id)
            return RedirectResponse(url="/products", status_code=303)
        except Exception as e:
            raise HTTPException(status_code=400, detail=str(e))

    return router
