from fastapi import Request
from agvcui.middleware import get_current_user_from_request


def get_template_context(request: Request, active_tab: str = None, **kwargs):
    """
    獲取模板的基本上下文，包括當前用戶信息
    
    Args:
        request: FastAPI Request 對象
        active_tab: 當前活躍的標籤頁
        **kwargs: 其他需要傳遞給模板的參數
    
    Returns:
        dict: 包含所有模板需要的上下文數據
    """
    current_user = get_current_user_from_request(request)
    
    context = {
        "request": request,
        "current_user": current_user,
    }
    
    if active_tab:
        context["active_tab"] = active_tab
    
    # 添加其他參數
    context.update(kwargs)
    
    return context


def require_auth_context(request: Request, active_tab: str = None, **kwargs):
    """
    獲取需要認證的模板上下文
    如果用戶未登入，會拋出異常
    
    Args:
        request: FastAPI Request 對象
        active_tab: 當前活躍的標籤頁
        **kwargs: 其他需要傳遞給模板的參數
    
    Returns:
        dict: 包含所有模板需要的上下文數據
        
    Raises:
        HTTPException: 如果用戶未登入
    """
    from fastapi import HTTPException, status
    
    current_user = get_current_user_from_request(request)
    
    if not current_user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="需要登入才能訪問此頁面"
        )
    
    context = {
        "request": request,
        "current_user": current_user,
    }
    
    if active_tab:
        context["active_tab"] = active_tab
    
    # 添加其他參數
    context.update(kwargs)
    
    return context


def require_admin_context(request: Request, active_tab: str = None, **kwargs):
    """
    獲取需要管理員權限的模板上下文
    
    Args:
        request: FastAPI Request 對象
        active_tab: 當前活躍的標籤頁
        **kwargs: 其他需要傳遞給模板的參數
    
    Returns:
        dict: 包含所有模板需要的上下文數據
        
    Raises:
        HTTPException: 如果用戶未登入或不是管理員
    """
    from fastapi import HTTPException, status
    
    current_user = get_current_user_from_request(request)
    
    if not current_user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="需要登入才能訪問此頁面"
        )
    
    if current_user.role != "admin":
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="需要管理員權限才能訪問此頁面"
        )
    
    context = {
        "request": request,
        "current_user": current_user,
    }
    
    if active_tab:
        context["active_tab"] = active_tab
    
    # 添加其他參數
    context.update(kwargs)
    
    return context
