# routers/product_router.py
from fastapi import APIRouter, HTTPException
from typing import List
from opui.db import connection_pool
from db_proxy.models import Product
from db_proxy.crud.product_crud import product_crud
from pydantic import BaseModel
from datetime import datetime, timezone
from typing import Optional

router = APIRouter(tags=["Product"])


class ProductRequest(BaseModel):
    name: str
    size: str
    process_settings_id: int


@router.get("/products", response_model=List[Product])
def list_products():
    with connection_pool.get_session() as session:
        return product_crud.get_all(session)


@router.get("/products/{product_id}", response_model=Product)
def get_product(product_id: int):
    with connection_pool.get_session() as session:
        product = product_crud.get_by_id(session, product_id)
        if not product:
            raise HTTPException(status_code=404, detail="Product not found")
        return product


@router.post("/products", response_model=Product)
def create_product(request: ProductRequest):
    with connection_pool.get_session() as session:
        now = datetime.now(timezone.utc)
        new_product = Product()
        new_product.name = request.name
        new_product.size = request.size
        new_product.process_settings_id = request.process_settings_id
        new_product.created_at = now
        new_product.updated_at = now
        product = product_crud.create(session, new_product)
        return product


@router.put("/products/{product_id}", response_model=Product)
def update_product(product_id: int, request: ProductRequest):
    with connection_pool.get_session() as session:
        now = datetime.now(timezone.utc)
        update_product = Product()
        update_product.name = request.name
        update_product.size = request.size
        update_product.process_settings_id = request.process_settings_id
        update_product.updated_at = now
        update_product.id = product_id
        updated = product_crud.update(session, product_id,update_product)
        if not updated:
            raise HTTPException(status_code=404, detail="Product not found")
        return updated


@router.delete("/products/{product_id}")
def delete_product(product_id: int):
    with connection_pool.get_session() as session:
        success = product_crud.delete(session, product_id)
        if not success:
            raise HTTPException(status_code=404, detail="Product not found")
        return {"message": "Product deleted successfully"}


@router.post("/products/create-or-update/{product_id}", response_model=Product)
def create_or_update_product(product_id: int, request: ProductRequest):
    with connection_pool.get_session() as session:
        now = datetime.now(timezone.utc)
        update_product = Product()
        update_product.name = request.name
        update_product.size = request.size
        update_product.process_settings_id = request.process_settings_id
        update_product.updated_at = now
        update_product["id"] = product_id
        # 不要傳入 created_at
        product = product_crud.create_or_update(session, update_product)
        return product
