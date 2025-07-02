#!/usr/bin/env python3
"""
測試 Cookie 問題的腳本
"""

def test_cookie_creation():
    """測試 Cookie 創建"""
    print("=== 測試 Cookie 創建 ===")
    
    try:
        from agvcui.auth import create_access_token
        from fastapi.responses import RedirectResponse
        
        # 模擬登入成功後的 token 創建
        token_data = {"sub": "admin", "role": "admin"}
        access_token = create_access_token(token_data)
        
        print(f"Token 創建成功: {access_token[:50]}...")
        
        # 模擬設置 cookie
        response = RedirectResponse(url="/", status_code=302)
        response.set_cookie(
            key="access_token",
            value=access_token,
            max_age=30 * 60,  # 30 分鐘
            httponly=True,
            secure=False,
            samesite="lax"
        )
        
        print("Cookie 設置參數:")
        print(f"  key: access_token")
        print(f"  value: {access_token[:30]}...")
        print(f"  max_age: {30 * 60} 秒")
        print(f"  httponly: True")
        print(f"  secure: False")
        print(f"  samesite: lax")
        
        return True
        
    except Exception as e:
        print(f"❌ Cookie 創建測試失敗: {e}")
        return False


def test_token_verification():
    """測試 Token 驗證"""
    print("\n=== 測試 Token 驗證 ===")
    
    try:
        from agvcui.auth import create_access_token, get_current_user_from_token
        
        # 創建 token
        token_data = {"sub": "admin", "role": "admin"}
        access_token = create_access_token(token_data)
        
        print(f"測試 Token: {access_token[:30]}...")
        
        # 驗證 token
        user = get_current_user_from_token(access_token)
        
        if user:
            print(f"✅ Token 驗證成功")
            print(f"   用戶名: {user.username}")
            print(f"   角色: {user.role}")
            print(f"   是否活躍: {user.is_active}")
            return True
        else:
            print("❌ Token 驗證失敗: 返回 None")
            return False
            
    except Exception as e:
        print(f"❌ Token 驗證測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_middleware_logic():
    """測試中間件邏輯"""
    print("\n=== 測試中間件邏輯 ===")
    
    try:
        from agvcui.middleware import AuthMiddleware
        from agvcui.auth import create_access_token, get_current_user_from_token
        
        middleware = AuthMiddleware(None)
        
        # 創建有效的 token
        token_data = {"sub": "admin", "role": "admin"}
        access_token = create_access_token(token_data)
        
        # 測試受保護路徑
        protected_paths = ["/map", "/tasks", "/devices", "/signals"]
        
        for path in protected_paths:
            print(f"\n測試路徑: {path}")
            
            # 檢查是否為受保護路徑
            is_protected = any(path.startswith(protected_path) 
                             for protected_path in middleware.protected_paths)
            print(f"  是否受保護: {is_protected}")
            
            if is_protected:
                # 模擬有 token 的情況
                print(f"  模擬有 token 的情況...")
                user = get_current_user_from_token(access_token)
                if user and user.is_active:
                    print(f"  ✅ 應該允許訪問: 用戶 {user.username}")
                else:
                    print(f"  ❌ 應該拒絕訪問: user={user}")
                
                # 模擬沒有 token 的情況
                print(f"  模擬沒有 token 的情況...")
                print(f"  ❌ 應該重定向到登入頁面")
        
        return True
        
    except Exception as e:
        print(f"❌ 中間件邏輯測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_cookie_path_issue():
    """測試 Cookie 路徑問題"""
    print("\n=== 測試 Cookie 路徑問題 ===")
    
    print("可能的 Cookie 問題:")
    print("1. Cookie 路徑設置問題")
    print("   - 如果 cookie 設置在 /login 路徑下，其他路徑可能無法訪問")
    print("   - 解決方案: 明確設置 path='/'")
    
    print("\n2. Cookie 域名問題")
    print("   - 如果域名不匹配，cookie 可能無法發送")
    print("   - 檢查瀏覽器開發者工具中的 cookie 設置")
    
    print("\n3. Cookie 安全設置問題")
    print("   - httponly=True 是正確的")
    print("   - secure=False 在 HTTP 環境下是正確的")
    print("   - samesite='lax' 是合適的設置")
    
    print("\n4. 瀏覽器緩存問題")
    print("   - 嘗試清除瀏覽器緩存和 cookies")
    print("   - 使用無痕模式測試")
    
    return True


def main():
    """主測試函數"""
    print("開始測試 Cookie 相關問題...\n")
    
    tests = [
        test_cookie_creation,
        test_token_verification,
        test_middleware_logic,
        test_cookie_path_issue
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
    
    print(f"\n=== 測試結果 ===")
    print(f"通過: {passed}/{total}")
    
    print(f"\n=== 調試建議 ===")
    print("1. 檢查瀏覽器開發者工具:")
    print("   - Network 標籤: 查看請求和響應")
    print("   - Application 標籤: 查看 Cookies")
    print("   - Console 標籤: 查看 JavaScript 錯誤")
    
    print("\n2. 檢查服務器日誌:")
    print("   - 查看中間件的調試輸出")
    print("   - 確認 token 驗證過程")
    
    print("\n3. 手動測試:")
    print("   - 登入後立即檢查 cookie 是否設置")
    print("   - 手動訪問受保護頁面 URL")
    print("   - 檢查重定向行為")
    
    return passed == total


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
