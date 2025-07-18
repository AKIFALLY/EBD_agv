# db_proxy/services/__init__.py
"""
通用 Service 層
提供跨專案使用的基礎業務邏輯
"""

from .product_service import ProductService
from .location_service import LocationService
from .machine_service import MachineService
from .room_service import RoomService

__all__ = [
    "ProductService",
    "LocationService", 
    "MachineService",
    "RoomService"
]
