from fastapi import Request, HTTPException, status
from fastapi.responses import RedirectResponse
from starlette.middleware.base import BaseHTTPMiddleware
from agvcui.auth import get_current_user_from_token
from typing import List


class AuthMiddleware(BaseHTTPMiddleware):
    """認證中間件"""

    def __init__(self, app, protected_paths: List[str] = None):
        super().__init__(app)
        # 需要認證的路徑（現在只有管理功能需要認證）
        self.protected_paths = protected_paths or [
            "/admin", "/users"  # 只有管理頁面需要認證
        ]
        # 不需要認證的路徑（現在幾乎所有路徑都是公開的）
        self.public_paths = [
            "/", "/login", "/logout", "/init-admin", "/static", "/favicon.ico",
            "/map", "/tasks", "/devices", "/signals", "/clients",
            "/racks", "/products", "/carriers", "/agvs", "/rosout_logs", "/runtime_logs"
        ]

    async def dispatch(self, request: Request, call_next):
        path = request.url.path
        print(f"🔍 中間件處理路徑: {path}")

        # 檢查是否為公開路徑（精確匹配或以路徑開頭，但排除根路徑的特殊情況）
        is_public = False
        for public_path in self.public_paths:
            if public_path == "/" and path == "/":
                is_public = True
                print(f"✅ 路徑 {path} 匹配公開路徑 {public_path} (根路徑)")
                break
            elif public_path != "/" and path.startswith(public_path):
                is_public = True
                print(f"✅ 路徑 {path} 匹配公開路徑 {public_path}")
                break

        if is_public:
            print(f"🔓 公開路徑: {path}")

            # 即使是公開路徑，也嘗試獲取用戶信息（如果有的話）
            access_token = request.cookies.get("access_token")
            if access_token:
                try:
                    user = get_current_user_from_token(access_token)
                    if user and user.is_active:
                        request.state.current_user = user
                        print(f"✅ 公開路徑，但檢測到已登入用戶: {user.username}")
                    else:
                        print(f"🔓 公開路徑，無有效用戶")
                except Exception as e:
                    print(f"🔓 公開路徑，token 驗證失敗: {e}")
            else:
                print(f"🔓 公開路徑，無 token")

            response = await call_next(request)
            return response

        # 檢查是否為需要保護的路徑
        is_protected = any(path.startswith(protected_path)
                           for protected_path in self.protected_paths)
        print(f"🔒 路徑 {path} 是否受保護: {is_protected}")

        if is_protected:
            # 從 cookie 獲取 token
            access_token = request.cookies.get("access_token")
            print(f"🍪 Cookie access_token: {'存在' if access_token else '不存在'}")
            if access_token:
                print(f"🔑 Token 內容: {access_token[:30]}...")

            # 打印所有 cookies 用於調試
            all_cookies = dict(request.cookies)
            print(f"📋 所有 Cookies: {list(all_cookies.keys())}")

            if not access_token:
                # 未登入，重定向到登入頁面
                print(f"❌ 無 token，重定向到登入頁面: {path}")
                return RedirectResponse(url=f"/login?redirect={path}", status_code=302)

            try:
                # 驗證 token
                user = get_current_user_from_token(access_token)
                if not user or not user.is_active:
                    # token 無效或用戶已停用
                    print(
                        f"❌ Token 驗證失敗: user={user}, is_active={user.is_active if user else 'N/A'}")
                    response = RedirectResponse(
                        url=f"/login?redirect={path}", status_code=302)
                    response.delete_cookie(key="access_token", path="/")
                    return response

                # 將用戶信息添加到請求中
                request.state.current_user = user
                print(f"✅ 用戶認證成功: {user.username}, 訪問路徑: {path}")

                # 認證成功，繼續處理請求
                response = await call_next(request)
                return response

            except Exception as e:
                # token 驗證失敗
                print(f"❌ Token 驗證異常: {e}")
                response = RedirectResponse(
                    url=f"/login?redirect={path}", status_code=302)
                response.delete_cookie(key="access_token", path="/")
                return response

        # 如果不是受保護路徑，也嘗試獲取用戶信息（如果有的話）
        print(f"🔓 非受保護路徑: {path}")

        access_token = request.cookies.get("access_token")
        if access_token:
            try:
                user = get_current_user_from_token(access_token)
                if user and user.is_active:
                    request.state.current_user = user
                    print(f"✅ 非受保護路徑，但檢測到已登入用戶: {user.username}")
                else:
                    print(f"🔓 非受保護路徑，無有效用戶")
            except Exception as e:
                print(f"🔓 非受保護路徑，token 驗證失敗: {e}")
        else:
            print(f"🔓 非受保護路徑，無 token")

        response = await call_next(request)
        return response


def get_current_user_from_request(request: Request):
    """從請求中獲取當前用戶"""
    return getattr(request.state, 'current_user', None)


def require_auth(request: Request):
    """要求認證的裝飾器函數"""
    user = get_current_user_from_request(request)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="需要登入"
        )
    return user


def require_admin_auth(request: Request):
    """要求管理員認證的裝飾器函數"""
    user = require_auth(request)
    if user.role != "admin":
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="需要管理員權限"
        )
    return user
