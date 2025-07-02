"""
權限檢查工具函數
用於檢查用戶是否有執行特定操作的權限
"""

from typing import Optional
from agvcui.middleware import get_current_user_from_request
from fastapi import Request


def has_permission(request: Request, required_role: str = "user") -> bool:
    """
    檢查用戶是否有指定角色的權限
    
    Args:
        request: FastAPI Request 對象
        required_role: 需要的最低角色 ("user", "operator", "admin")
    
    Returns:
        bool: 是否有權限
    """
    current_user = get_current_user_from_request(request)
    
    if not current_user or not current_user.is_active:
        return False
    
    # 角色層級
    role_levels = {
        "user": 1,
        "operator": 2,
        "admin": 3
    }
    
    user_level = role_levels.get(current_user.role, 0)
    required_level = role_levels.get(required_role, 0)
    
    return user_level >= required_level


def can_create(request: Request) -> bool:
    """檢查是否可以創建資源"""
    return has_permission(request, "operator")


def can_edit(request: Request) -> bool:
    """檢查是否可以編輯資源"""
    return has_permission(request, "operator")


def can_delete(request: Request) -> bool:
    """檢查是否可以刪除資源"""
    return has_permission(request, "admin")


def can_manage_users(request: Request) -> bool:
    """檢查是否可以管理用戶"""
    return has_permission(request, "admin")


def get_user_permissions(request: Request) -> dict:
    """
    獲取當前用戶的所有權限
    
    Returns:
        dict: 包含各種權限的字典
    """
    current_user = get_current_user_from_request(request)
    
    if not current_user or not current_user.is_active:
        return {
            "can_view": False,
            "can_create": False,
            "can_edit": False,
            "can_delete": False,
            "can_manage_users": False,
            "is_logged_in": False,
            "role": None
        }
    
    return {
        "can_view": True,  # 所有登入用戶都可以查看
        "can_create": can_create(request),
        "can_edit": can_edit(request),
        "can_delete": can_delete(request),
        "can_manage_users": can_manage_users(request),
        "is_logged_in": True,
        "role": current_user.role,
        "username": current_user.username
    }


def require_permission(required_role: str = "user"):
    """
    裝飾器：要求特定權限
    
    Args:
        required_role: 需要的最低角色
    
    Returns:
        裝飾器函數
    """
    def decorator(func):
        def wrapper(request: Request, *args, **kwargs):
            if not has_permission(request, required_role):
                from fastapi import HTTPException, status
                raise HTTPException(
                    status_code=status.HTTP_403_FORBIDDEN,
                    detail=f"需要 {required_role} 或更高權限"
                )
            return func(request, *args, **kwargs)
        return wrapper
    return decorator


# 權限常量
class Permissions:
    VIEW = "user"
    CREATE = "operator"
    EDIT = "operator"
    DELETE = "admin"
    MANAGE_USERS = "admin"


# 角色常量
class Roles:
    USER = "user"
    OPERATOR = "operator"
    ADMIN = "admin"
    
    @classmethod
    def get_all(cls):
        return [cls.USER, cls.OPERATOR, cls.ADMIN]
    
    @classmethod
    def get_display_name(cls, role: str) -> str:
        display_names = {
            cls.USER: "用戶",
            cls.OPERATOR: "操作員",
            cls.ADMIN: "管理員"
        }
        return display_names.get(role, role)
