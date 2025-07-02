# database/utils.py
"""
資料庫通用工具函數
"""

from typing import Callable
from .connection import connection_pool


def fetch_all(crud_handler: Callable) -> list[dict]:
    """通用的獲取所有資料函數"""
    with connection_pool.get_session() as session:
        results = crud_handler.get_all(session)
        return [item.model_dump() for item in results]
