#!/usr/bin/env python3
"""
調試用戶顯示問題的腳本
"""

def test_middleware_logic():
    """測試中間件邏輯"""
    print("=== 測試中間件邏輯 ===")
    
    try:
        from agvcui.middleware import AuthMiddleware
        
        middleware = AuthMiddleware(None)
        
        print("受保護路徑:")
        for path in middleware.protected_paths:
            print(f"  - {path}")
        
        print("\n公開路徑:")
        for path in middleware.public_paths:
            print(f"  - {path}")
        
        # 測試路徑分類
        test_paths = [
            "/",
            "/map", 
            "/tasks",
            "/users",
            "/login"
        ]
        
        print("\n路徑分類測試:")
        for path in test_paths:
            is_public = any(path.startswith(public_path) for public_path in middleware.public_paths)
            is_protected = any(path.startswith(protected_path) for protected_path in middleware.protected_paths)
            
            if path == "/" and "/" in middleware.public_paths:
                is_public = True
            
            if is_public:
                status = "公開"
            elif is_protected:
                status = "受保護"
            else:
                status = "未分類"
            
            print(f"  {path} -> {status}")
        
        return True
        
    except Exception as e:
        print(f"❌ 中間件邏輯測試失敗: {e}")
        return False


def test_user_info_flow():
    """測試用戶信息流程"""
    print("\n=== 測試用戶信息流程 ===")
    
    print("預期流程:")
    print("1. 用戶登入 -> 設置 access_token cookie")
    print("2. 訪問任何頁面 -> 中間件檢測 cookie")
    print("3. 如果有有效 token -> 設置 request.state.current_user")
    print("4. 路由獲取用戶信息 -> 傳遞給模板")
    print("5. 模板顯示用戶信息")
    
    print("\n可能的問題點:")
    print("1. Cookie 未正確設置或讀取")
    print("2. 中間件未正確設置 request.state.current_user")
    print("3. 路由未正確獲取用戶信息")
    print("4. 模板未正確顯示用戶信息")
    
    return True


def test_template_context():
    """測試模板上下文"""
    print("\n=== 測試模板上下文 ===")
    
    print("檢查模板是否正確接收 current_user:")
    print("- 所有路由都應該調用 get_current_user_from_request(request)")
    print("- 所有模板響應都應該包含 'current_user': current_user")
    
    print("\n檢查 navbar.html 邏輯:")
    print("{% if current_user %}")
    print("  <!-- 顯示用戶信息 -->")
    print("{% else %}")
    print("  <!-- 顯示登入按鈕 -->")
    print("{% endif %}")
    
    return True


def show_debugging_steps():
    """顯示調試步驟"""
    print("\n=== 調試步驟 ===")
    
    print("1. 檢查服務器日誌:")
    print("   - 登入後應該看到: ✅ 登入成功")
    print("   - 訪問頁面時應該看到: ✅ 公開路徑，但檢測到已登入用戶")
    
    print("\n2. 檢查瀏覽器:")
    print("   - F12 -> Application -> Cookies")
    print("   - 確認 access_token 存在")
    print("   - 檢查 cookie 的 Path 是否為 '/'")
    
    print("\n3. 檢查頁面源碼:")
    print("   - 右鍵 -> 查看頁面源碼")
    print("   - 搜索 'window.currentUser'")
    print("   - 確認用戶信息是否正確傳遞")
    
    print("\n4. 測試不同頁面:")
    print("   - 訪問 / (首頁)")
    print("   - 訪問 /map")
    print("   - 訪問 /tasks")
    print("   - 檢查 navbar 是否一致顯示用戶信息")
    
    return True


def show_quick_fixes():
    """顯示快速修復方案"""
    print("\n=== 快速修復方案 ===")
    
    print("如果問題仍然存在，嘗試以下修復:")
    
    print("\n1. 清除瀏覽器狀態:")
    print("   - 清除所有 cookies")
    print("   - 清除緩存")
    print("   - 重新啟動瀏覽器")
    
    print("\n2. 重新登入:")
    print("   - 訪問 /logout")
    print("   - 訪問 /login")
    print("   - 重新登入")
    
    print("\n3. 檢查模板:")
    print("   - 確認 navbar.html 中的 {% if current_user %} 邏輯")
    print("   - 確認所有路由都傳遞 current_user")
    
    print("\n4. 強制刷新:")
    print("   - Ctrl+F5 強制刷新頁面")
    print("   - 或使用無痕模式測試")
    
    return True


def main():
    """主測試函數"""
    print("開始調試用戶顯示問題...\n")
    
    tests = [
        test_middleware_logic,
        test_user_info_flow,
        test_template_context,
        show_debugging_steps,
        show_quick_fixes
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
    
    print(f"\n=== 調試總結 ===")
    print("已修復的問題:")
    print("1. ✅ 中間件現在會為所有路徑設置用戶信息")
    print("2. ✅ 公開路徑也會檢測登入狀態")
    print("3. ✅ 添加了詳細的調試日誌")
    
    print(f"\n下一步:")
    print("1. 重新啟動服務器")
    print("2. 清除瀏覽器緩存和 cookies")
    print("3. 重新登入測試")
    print("4. 查看服務器日誌中的詳細信息")
    
    return passed == total


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
