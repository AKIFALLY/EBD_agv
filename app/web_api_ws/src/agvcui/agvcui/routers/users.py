from datetime import datetime, timezone
from zoneinfo import ZoneInfo
from fastapi import APIRouter, Request, Form, HTTPException, status
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.templating import Jinja2Templates
from agvcui.middleware import get_current_user_from_request, require_admin_auth
from agvcui.auth import hash_password
from agvcui.db import (
    get_users, count_users, create_user, get_user_by_username,
    get_user_by_email, update_user_last_login
)


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/users", response_class=HTMLResponse)
    async def users_list(request: Request, page: int = 1):
        """用戶管理頁面 - 需要管理員權限"""
        current_user = get_current_user_from_request(request)

        # 檢查管理員權限
        if not current_user or current_user.role != "admin":
            return RedirectResponse(url="/login?redirect=/users", status_code=302)

        limit = 20
        offset = (page - 1) * limit
        users = get_users(offset=offset, limit=limit)
        total = count_users()
        total_pages = (total + limit - 1) // limit
        taipei_time = datetime.now(ZoneInfo("Asia/Taipei"))
        latest_query = taipei_time.strftime("%Y/%m/%d %H:%M:%S")

        return templates.TemplateResponse("users.html", {
            "request": request,
            "users": users,
            "active_tab": "users",
            "latest_query": latest_query,
            "current_page": page,
            "total_pages": total_pages,
            "total_count": total,
            "current_user": current_user
        })

    @router.get("/users/create", response_class=HTMLResponse)
    async def create_user_page(request: Request):
        """創建用戶頁面"""
        current_user = get_current_user_from_request(request)

        # 檢查管理員權限
        if not current_user or current_user.role != "admin":
            return RedirectResponse(url="/login?redirect=/users/create", status_code=302)

        return templates.TemplateResponse("user_create.html", {
            "request": request,
            "active_tab": "users",
            "current_user": current_user
        })

    @router.post("/users/create")
    async def create_user_action(
        request: Request,
        username: str = Form(...),
        password: str = Form(...),
        email: str = Form(default=""),
        full_name: str = Form(default=""),
        role: str = Form(default="user")
    ):
        """處理創建用戶請求"""
        current_user = get_current_user_from_request(request)

        # 檢查管理員權限
        if not current_user or current_user.role != "admin":
            return RedirectResponse(url="/login?redirect=/users/create", status_code=302)

        try:
            # 檢查用戶名是否已存在
            existing_user = get_user_by_username(username)
            if existing_user:
                return templates.TemplateResponse("user_create.html", {
                    "request": request,
                    "active_tab": "users",
                    "current_user": current_user,
                    "error": "用戶名已存在"
                })

            # 檢查郵箱是否已存在
            if email and get_user_by_email(email):
                return templates.TemplateResponse("user_create.html", {
                    "request": request,
                    "active_tab": "users",
                    "current_user": current_user,
                    "error": "郵箱已存在"
                })

            # 創建用戶
            hashed_password = hash_password(password)
            new_user = create_user(
                username=username,
                password_hash=hashed_password,
                email=email if email else None,
                full_name=full_name if full_name else None,
                role=role
            )

            print(f"✅ 管理員 {current_user.username} 創建了新用戶: {new_user.username}")
            return RedirectResponse(url="/users", status_code=302)

        except Exception as e:
            print(f"❌ 創建用戶失敗: {e}")
            return templates.TemplateResponse("user_create.html", {
                "request": request,
                "active_tab": "users",
                "current_user": current_user,
                "error": f"創建用戶失敗: {str(e)}"
            })

    @router.get("/users/{user_id}/edit", response_class=HTMLResponse)
    async def edit_user_page(request: Request, user_id: int):
        """編輯用戶頁面"""
        current_user = get_current_user_from_request(request)

        # 檢查管理員權限
        if not current_user or current_user.role != "admin":
            return RedirectResponse(url="/login?redirect=/users", status_code=302)

        # 獲取要編輯的用戶
        from agvcui.db import connection_pool
        from db_proxy.crud.user_crud import user_crud

        with connection_pool.get_session() as session:
            edit_user = user_crud.get_by_id(session, user_id)

        if not edit_user:
            return RedirectResponse(url="/users", status_code=302)

        return templates.TemplateResponse("user_edit.html", {
            "request": request,
            "active_tab": "users",
            "current_user": current_user,
            "edit_user": edit_user
        })

    @router.post("/users/{user_id}/edit")
    async def edit_user_action(
        request: Request,
        user_id: int,
        username: str = Form(...),
        email: str = Form(default=""),
        full_name: str = Form(default=""),
        role: str = Form(...),
        is_active: str = Form(default="")
    ):
        """處理編輯用戶請求"""
        current_user = get_current_user_from_request(request)

        # 檢查管理員權限
        if not current_user or current_user.role != "admin":
            return RedirectResponse(url="/login?redirect=/users", status_code=302)

        try:
            from agvcui.db import connection_pool
            from db_proxy.crud.user_crud import user_crud

            print(f"📝 收到編輯用戶請求: user_id={user_id}")
            print(
                f"📝 表單數據: username={username}, email={email}, full_name={full_name}, role={role}, is_active='{is_active}'")

            with connection_pool.get_session() as session:
                edit_user = user_crud.get_by_id(session, user_id)
                if not edit_user:
                    print(f"❌ 找不到用戶 ID: {user_id}")
                    return RedirectResponse(url="/users", status_code=302)

                print(
                    f"📝 編輯前用戶信息: username={edit_user.username}, email={edit_user.email}, role={edit_user.role}, is_active={edit_user.is_active}")

                # 保護 admin 用戶：禁止停用
                if edit_user.username == "admin" and not bool(is_active):
                    print(f"⚠️ 禁止停用 admin 用戶")
                    # 返回錯誤訊息（這裡簡化處理，實際可以添加 flash message）
                    return RedirectResponse(url="/users?error=cannot_disable_admin", status_code=302)

                # 更新用戶信息
                edit_user.username = username
                edit_user.email = email if email else None
                edit_user.full_name = full_name if full_name else None
                edit_user.role = role
                # checkbox: 有值表示勾選，空字符串表示未勾選
                edit_user.is_active = bool(is_active)
                edit_user.updated_at = datetime.now(timezone.utc)

                print(
                    f"📝 編輯後用戶信息: username={edit_user.username}, email={edit_user.email}, role={edit_user.role}, is_active={edit_user.is_active}")

                session.add(edit_user)
                session.commit()
                session.refresh(edit_user)

                print(f"💾 數據庫更新完成")

            print(f"✅ 管理員 {current_user.username} 更新了用戶: {username}")
            return RedirectResponse(url="/users", status_code=302)

        except Exception as e:
            print(f"❌ 更新用戶失敗: {e}")
            import traceback
            traceback.print_exc()
            return RedirectResponse(url="/users", status_code=302)

    @router.post("/users/{user_id}/delete")
    async def delete_user_action(request: Request, user_id: int):
        """刪除用戶"""
        current_user = get_current_user_from_request(request)

        # 檢查管理員權限
        if not current_user or current_user.role != "admin":
            return RedirectResponse(url="/login?redirect=/users", status_code=302)

        # 不能刪除自己
        if current_user.id == user_id:
            return RedirectResponse(url="/users", status_code=302)

        try:
            from agvcui.db import connection_pool
            from db_proxy.crud.user_crud import user_crud

            with connection_pool.get_session() as session:
                delete_user = user_crud.get_by_id(session, user_id)
                if delete_user:
                    # 保護 admin 用戶：禁止刪除
                    if delete_user.username == "admin":
                        print(f"⚠️ 禁止刪除 admin 用戶")
                        return RedirectResponse(url="/users?error=cannot_delete_admin", status_code=302)

                    session.delete(delete_user)
                    session.commit()
                    print(
                        f"✅ 管理員 {current_user.username} 刪除了用戶: {delete_user.username}")

            return RedirectResponse(url="/users", status_code=302)

        except Exception as e:
            print(f"❌ 刪除用戶失敗: {e}")
            return RedirectResponse(url="/users", status_code=302)

    return router
