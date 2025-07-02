#!/usr/bin/env python3
"""
調試認證問題的腳本
"""

def test_token_flow():
    """測試完整的 token 流程"""
    print("=== 測試 Token 流程 ===")
    
    try:
        from agvcui.auth import create_access_token, verify_token, get_current_user_from_token
        
        # 1. 創建 token
        print("1. 創建 Token...")
        token_data = {"sub": "admin", "role": "admin"}
        token = create_access_token(token_data)
        print(f"   Token 創建成功: {token[:50]}...")
        
        # 2. 驗證 token
        print("2. 驗證 Token...")
        verified = verify_token(token)
        print(f"   Token 驗證結果: {verified}")
        
        if verified:
            print(f"   用戶名: {verified.username}")
        else:
            print("   ❌ Token 驗證失敗")
            return False
        
        # 3. 測試 get_current_user_from_token (不會拋出異常的版本)
        print("3. 測試 get_current_user_from_token...")
        try:
            user = get_current_user_from_token(token)
            if user:
                print(f"   ✅ 用戶獲取成功: {user}")
            else:
                print("   ❌ 用戶獲取失敗: 返回 None")
                return False
        except Exception as e:
            print(f"   ❌ 用戶獲取異常: {e}")
            return False
        
        print("✅ Token 流程測試通過!")
        return True
        
    except Exception as e:
        print(f"❌ Token 流程測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_middleware_paths():
    """測試中間件路徑配置"""
    print("\n=== 測試中間件路徑配置 ===")
    
    try:
        from agvcui.middleware import AuthMiddleware
        
        middleware = AuthMiddleware(None)
        
        print("受保護路徑:")
        for path in middleware.protected_paths:
            print(f"  - {path}")
        
        print("\n公開路徑:")
        for path in middleware.public_paths:
            print(f"  - {path}")
        
        # 測試路徑匹配
        test_paths = [
            ("/", "應該是公開的"),
            ("/login", "應該是公開的"),
            ("/map", "應該是受保護的"),
            ("/static/css/style.css", "應該是公開的"),
            ("/tasks", "應該是受保護的")
        ]
        
        print("\n路徑測試:")
        for path, expected in test_paths:
            is_public = any(path.startswith(public_path) for public_path in middleware.public_paths)
            is_protected = any(path.startswith(protected_path) for protected_path in middleware.protected_paths)
            
            if is_public:
                status = "公開"
            elif is_protected:
                status = "受保護"
            else:
                status = "未分類"
            
            print(f"  {path} -> {status} ({expected})")
        
        print("✅ 中間件路徑配置測試完成!")
        return True
        
    except Exception as e:
        print(f"❌ 中間件路徑配置測試失敗: {e}")
        return False


def test_cookie_simulation():
    """模擬 cookie 處理"""
    print("\n=== 模擬 Cookie 處理 ===")
    
    try:
        from agvcui.auth import create_access_token, get_current_user_from_token
        
        # 模擬登入後的 token
        token = create_access_token({"sub": "admin", "role": "admin"})
        print(f"模擬 Cookie Token: {token[:30]}...")
        
        # 模擬中間件驗證
        user = get_current_user_from_token(token)
        if user:
            print(f"✅ Cookie 驗證成功: 用戶 {user}")
        else:
            print("❌ Cookie 驗證失敗")
            return False
        
        # 測試無效 token
        invalid_token = "invalid.token.here"
        user = get_current_user_from_token(invalid_token)
        if user is None:
            print("✅ 無效 Token 正確返回 None")
        else:
            print("❌ 無效 Token 應該返回 None")
            return False
        
        print("✅ Cookie 處理測試通過!")
        return True
        
    except Exception as e:
        print(f"❌ Cookie 處理測試失敗: {e}")
        return False


def main():
    """主測試函數"""
    print("開始調試認證問題...\n")
    
    tests = [
        test_token_flow,
        test_middleware_paths,
        test_cookie_simulation
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
    
    print(f"\n=== 調試結果 ===")
    print(f"通過: {passed}/{total}")
    
    if passed == total:
        print("🎉 所有測試都通過了!")
        print("\n可能的問題:")
        print("1. 資料庫連接問題 - 檢查 get_user_by_username 是否正常工作")
        print("2. 瀏覽器 Cookie 問題 - 檢查瀏覽器開發者工具")
        print("3. 重定向循環 - 檢查服務器日誌中的調試信息")
    else:
        print("❌ 部分測試失敗，請檢查上述錯誤")
    
    return passed == total


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
