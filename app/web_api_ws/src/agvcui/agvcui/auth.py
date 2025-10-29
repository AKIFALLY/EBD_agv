import os
import hashlib
import hmac
import secrets
from datetime import datetime, timedelta, timezone
from typing import Optional, Dict, Any
from fastapi import HTTPException, status
from pydantic import BaseModel
import json
import base64


# JWT 設定
SECRET_KEY = os.getenv(
    "SECRET_KEY", "your-secret-key-change-this-in-production")
ALGORITHM = "HS256"
# 移除自動過期 - token 永不過期，由用戶手動登出
ACCESS_TOKEN_EXPIRE_MINUTES = None


class Token(BaseModel):
    access_token: str
    token_type: str


class TokenData(BaseModel):
    username: Optional[str] = None


class UserLogin(BaseModel):
    username: str
    password: str


class UserCreate(BaseModel):
    username: str
    password: str
    email: Optional[str] = None
    full_name: Optional[str] = None
    role: str = "user"


def hash_password(password: str) -> str:
    """對密碼進行哈希處理"""
    salt = secrets.token_hex(16)
    password_hash = hashlib.pbkdf2_hmac(
        'sha256', password.encode('utf-8'), salt.encode('utf-8'), 100000)
    return f"{salt}:{password_hash.hex()}"


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """驗證密碼"""
    try:
        salt, stored_hash = hashed_password.split(':')
        password_hash = hashlib.pbkdf2_hmac(
            'sha256', plain_password.encode('utf-8'), salt.encode('utf-8'), 100000)
        return hmac.compare_digest(stored_hash, password_hash.hex())
    except Exception:
        return False


def create_access_token(data: Dict[str, Any], expires_delta: Optional[timedelta] = None) -> str:
    """創建簡單的 access token（永不過期）"""
    to_encode = data.copy()

    # 移除過期時間設置，讓 token 永不過期
    # 用戶登入狀態由 Socket.IO 和 userStore 管理
    # if expires_delta:
    #     expire = datetime.now(timezone.utc) + expires_delta
    # else:
    #     expire = datetime.now(timezone.utc) + \
    #         timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    # to_encode.update({"exp": expire.timestamp()})

    # 簡單的 token 實現
    token_data = json.dumps(to_encode)
    token_bytes = token_data.encode('utf-8')
    signature = hmac.new(SECRET_KEY.encode('utf-8'),
                         token_bytes, hashlib.sha256).hexdigest()

    token = base64.b64encode(token_bytes).decode('utf-8')
    return f"{token}.{signature}"


def verify_token(token: str) -> Optional[TokenData]:
    """驗證 token"""
    try:
        token_part, signature = token.split('.')
        token_bytes = base64.b64decode(token_part.encode('utf-8'))

        # 驗證簽名
        expected_signature = hmac.new(SECRET_KEY.encode(
            'utf-8'), token_bytes, hashlib.sha256).hexdigest()
        if not hmac.compare_digest(signature, expected_signature):
            return None

        # 解析 token 數據
        token_data = json.loads(token_bytes.decode('utf-8'))

        # 移除過期時間檢查，token 永不過期
        # if token_data.get("exp", 0) < datetime.now(timezone.utc).timestamp():
        #     return None

        username = token_data.get("sub")
        if username is None:
            return None

        return TokenData(username=username)
    except Exception:
        return None


def authenticate_user(username: str, password: str):
    """驗證用戶登入

    返回值：
    - (True, user): 登入成功
    - (False, "user_not_found"): 用戶不存在
    - (False, "invalid_password"): 密碼錯誤
    - (False, "user_inactive"): 用戶已被停用
    """
    from agvcui.db import get_user_by_username

    user = get_user_by_username(username)
    if not user:
        return (False, "user_not_found")
    if not verify_password(password, user.password_hash):
        return (False, "invalid_password")
    if not user.is_active:
        return (False, "user_inactive")
    return (True, user)


def get_current_user_from_token(token: str):
    """從 token 獲取當前用戶"""
    from agvcui.db import get_user_by_username

    try:
        token_data = verify_token(token)
        if token_data is None:
            return None

        user = get_user_by_username(username=token_data.username)
        return user
    except Exception:
        return None


def get_current_user_from_token_strict(token: str):
    """從 token 獲取當前用戶（嚴格模式，會拋出異常）"""
    from agvcui.db import get_user_by_username

    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    token_data = verify_token(token)
    if token_data is None:
        raise credentials_exception

    user = get_user_by_username(username=token_data.username)
    if user is None:
        raise credentials_exception

    return user


def create_default_admin():
    """創建默認管理員用戶

    如果 admin 用戶已存在但被停用，會自動修復為啟用狀態
    """
    from agvcui.db import get_user_by_username, create_user, connection_pool

    admin_username = "admin"
    admin_password = "admin123"  # 在生產環境中應該使用更強的密碼

    # 檢查是否已存在管理員用戶
    existing_admin = get_user_by_username(admin_username)
    if existing_admin:
        # 檢查並修復 is_active 狀態
        if not existing_admin.is_active:
            print(f"⚠️ 管理員用戶 '{admin_username}' 已被停用，正在修復...")
            with connection_pool.get_session() as session:
                existing_admin.is_active = True
                session.add(existing_admin)
                session.commit()
                session.refresh(existing_admin)
            print(f"✅ 已修復管理員用戶 '{admin_username}' 的啟用狀態")
        else:
            print(f"✅ 管理員用戶 '{admin_username}' 已存在且狀態正常")
        return existing_admin

    # 創建管理員用戶
    hashed_password = hash_password(admin_password)
    admin_user = create_user(
        username=admin_username,
        password_hash=hashed_password,
        email="admin@agvc.local",
        full_name="系統管理員",
        role="admin"
    )

    print(f"✅ 已創建默認管理員用戶: {admin_username} / {admin_password}")
    return admin_user
