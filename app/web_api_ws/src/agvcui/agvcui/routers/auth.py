from datetime import timedelta
from fastapi import APIRouter, Request, Form, HTTPException, status, Depends, Cookie
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.templating import Jinja2Templates
from agvcui.auth import (
    authenticate_user, create_access_token, get_current_user_from_token_strict,
    UserLogin, ACCESS_TOKEN_EXPIRE_MINUTES, create_default_admin
)
from agvcui.db import update_user_last_login
from typing import Optional


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/login", response_class=HTMLResponse)
    async def login_page(request: Request, redirect: Optional[str] = None):
        """顯示登入頁面"""
        return templates.TemplateResponse("login.html", {
            "request": request,
            "redirect": redirect or "/",
            "error": None
        })

    @router.post("/login")
    async def login_for_access_token(
        request: Request,
        username: str = Form(...),
        password: str = Form(...),
        redirect: str = Form(default="/")
    ):
        """處理登入請求"""
        user = authenticate_user(username, password)
        if not user:
            return templates.TemplateResponse("login.html", {
                "request": request,
                "redirect": redirect,
                "error": "用戶名或密碼錯誤"
            })

        # 更新最後登入時間
        update_user_last_login(user.id)

        # 創建 access token（永不過期）
        access_token = create_access_token(
            data={"sub": user.username, "role": user.role},
            expires_delta=None  # 不設置過期時間
        )

        print(f"✅ 登入成功: {user.username}, 重定向到: {redirect}")
        print(f"🔑 Token: {access_token[:50]}...")

        # 重定向到目標頁面並設置 cookie（永不過期）
        response = RedirectResponse(url=redirect, status_code=302)
        response.set_cookie(
            key="access_token",
            value=access_token,
            # max_age=ACCESS_TOKEN_EXPIRE_MINUTES * 60,  # 移除過期時間
            path="/",  # 明確設置路徑為根路徑
            httponly=True,
            secure=False,  # 在生產環境中應該設為 True
            samesite="lax"
        )
        return response

    @router.get("/logout")
    async def logout(request: Request):
        """登出"""
        # 檢查當前是否有 token
        current_token = request.cookies.get("access_token")
        print(f"🚪 登出請求: 當前 token {'存在' if current_token else '不存在'}")
        if current_token:
            print(f"🔑 要刪除的 token: {current_token[:30]}...")

        response = RedirectResponse(url="/login", status_code=302)

        # 更強健的 cookie 刪除
        response.delete_cookie(
            key="access_token",
            path="/",
            domain=None,  # 使用默認域名
            secure=False,  # 與設置時保持一致
            httponly=True,  # 與設置時保持一致
            samesite="lax"  # 與設置時保持一致
        )

        # 額外的清理：設置過期時間為過去
        response.set_cookie(
            key="access_token",
            value="",
            path="/",
            expires="Thu, 01 Jan 1970 00:00:00 GMT",
            max_age=0,
            httponly=True,
            secure=False,
            samesite="lax"
        )

        print(f"✅ 已刪除 access_token cookie，重定向到登入頁面")
        return response

    @router.get("/api/auth/me")
    async def get_current_user_api(request: Request):
        """API 端點：獲取當前用戶信息"""
        from agvcui.auth import get_current_user_from_token
        from fastapi.responses import JSONResponse

        # 從 cookie 中獲取 token
        token = request.cookies.get("access_token")
        if not token:
            return JSONResponse(
                status_code=401,
                content={"error": "未登入"}
            )

        # 驗證 token 並獲取用戶
        user = get_current_user_from_token(token)
        if not user:
            return JSONResponse(
                status_code=401,
                content={"error": "無效的登入狀態"}
            )

        # 返回用戶信息
        return {
            "id": user.id,
            "username": user.username,
            "role": user.role,
            "full_name": user.full_name,
            "is_active": user.is_active
        }

    @router.get("/init-admin")
    async def init_admin():
        """初始化管理員用戶"""
        try:
            admin_user = create_default_admin()
            return {
                "success": True,
                "message": f"管理員用戶已創建或已存在: {admin_user.username}",
                "username": admin_user.username
            }
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

    return router


# 認證中間件依賴
async def get_current_user(access_token: Optional[str] = Cookie(None)):
    """從 cookie 獲取當前用戶"""
    if not access_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="未登入",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return get_current_user_from_token_strict(access_token)


async def get_current_active_user(current_user=Depends(get_current_user)):
    """獲取當前活躍用戶"""
    if not current_user.is_active:
        raise HTTPException(status_code=400, detail="用戶已被停用")
    return current_user


async def require_admin(current_user=Depends(get_current_active_user)):
    """要求管理員權限"""
    if current_user.role != "admin":
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="需要管理員權限"
        )
    return current_user


# 可選的認證依賴（不強制登入）
async def get_current_user_optional(access_token: Optional[str] = Cookie(None)):
    """可選的用戶認證，不會拋出異常"""
    if not access_token:
        return None

    try:
        return get_current_user_from_token_strict(access_token)
    except:
        return None
