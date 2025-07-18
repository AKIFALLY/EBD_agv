# wcs/services/__init__.py
"""
WCS 專案特定的 Service 層
"""

from .wcs_task_service import WcsTaskService

__all__ = [
    "WcsTaskService"
]
