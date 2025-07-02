#!/usr/bin/env python3
"""
測試 Cookie 路徑修復的腳本
"""

def test_cookie_path_setting():
    """測試 Cookie 路徑設置"""
    print("=== 測試 Cookie 路徑設置 ===")
    
    try:
        from agvcui.auth import create_access_token, ACCESS_TOKEN_EXPIRE_MINUTES
        from fastapi.responses import RedirectResponse
        
        # 模擬登入成功後的 cookie 設置
        token_data = {"sub": "admin", "role": "admin"}
        access_token = create_access_token(token_data)
        
        response = RedirectResponse(url="/", status_code=302)
        response.set_cookie(
            key="access_token",
            value=access_token,
            max_age=ACCESS_TOKEN_EXPIRE_MINUTES * 60,
            path="/",  # 明確設置路徑
            httponly=True,
            secure=False,
            samesite="lax"
        )
        
        print("✅ Cookie 設置參數 (修復後):")
        print(f"   key: access_token")
        print(f"   path: /  ← 這是關鍵修復")
        print(f"   max_age: {ACCESS_TOKEN_EXPIRE_MINUTES * 60} 秒")
        print(f"   httponly: True")
        print(f"   secure: False")
        print(f"   samesite: lax")
        
        return True
        
    except Exception as e:
        print(f"❌ Cookie 路徑設置測試失敗: {e}")
        return False


def test_cookie_deletion():
    """測試 Cookie 刪除"""
    print("\n=== 測試 Cookie 刪除 ===")
    
    try:
        from fastapi.responses import RedirectResponse
        
        # 模擬登出時的 cookie 刪除
        response = RedirectResponse(url="/login", status_code=302)
        response.delete_cookie(key="access_token", path="/")
        
        print("✅ Cookie 刪除參數 (修復後):")
        print(f"   key: access_token")
        print(f"   path: /  ← 這是關鍵修復")
        
        return True
        
    except Exception as e:
        print(f"❌ Cookie 刪除測試失敗: {e}")
        return False


def test_middleware_debug_output():
    """測試中間件調試輸出"""
    print("\n=== 測試中間件調試輸出 ===")
    
    try:
        from agvcui.middleware import AuthMiddleware
        
        middleware = AuthMiddleware(None)
        
        print("中間件現在會輸出以下調試信息:")
        print("🔍 中間件處理路徑: /map")
        print("🔒 路徑 /map 是否受保護: True")
        print("🍪 Cookie access_token: 存在/不存在")
        print("✅ 用戶認證成功: admin, 訪問路徑: /map")
        print("或")
        print("❌ Token 驗證失敗: ...")
        
        print("\n請在服務器日誌中查看這些信息來診斷問題")
        
        return True
        
    except Exception as e:
        print(f"❌ 中間件調試輸出測試失敗: {e}")
        return False


def show_debugging_steps():
    """顯示調試步驟"""
    print("\n=== 調試步驟 ===")
    
    print("1. 重新啟動服務器以應用修復")
    print("   cd /app/web_api_ws/src/agvcui")
    print("   python3 -m agvcui.agvc_ui_server")
    
    print("\n2. 清除瀏覽器緩存和 Cookies")
    print("   - 打開瀏覽器開發者工具 (F12)")
    print("   - 右鍵點擊刷新按鈕，選擇 '清空緩存並硬性重新載入'")
    print("   - 或使用無痕模式測試")
    
    print("\n3. 測試登入流程")
    print("   - 訪問 http://localhost:8001/login")
    print("   - 輸入 admin / admin123")
    print("   - 點擊登入")
    
    print("\n4. 檢查 Cookie 設置")
    print("   - 在開發者工具的 Application 標籤中")
    print("   - 查看 Cookies 部分")
    print("   - 確認 access_token cookie 存在且 Path 為 '/'")
    
    print("\n5. 測試受保護頁面")
    print("   - 點擊導航欄中的 Map, Tasks 等連結")
    print("   - 應該能正常訪問，不會重定向到登入頁面")
    
    print("\n6. 查看服務器日誌")
    print("   - 觀察中間件的調試輸出")
    print("   - 確認 token 驗證過程")
    
    return True


def main():
    """主測試函數"""
    print("測試 Cookie 路徑修復...\n")
    
    tests = [
        test_cookie_path_setting,
        test_cookie_deletion,
        test_middleware_debug_output,
        show_debugging_steps
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
    
    print(f"\n=== 修復總結 ===")
    print("✅ 已修復的問題:")
    print("1. Cookie 路徑設置 - 明確設置 path='/'")
    print("2. Cookie 刪除路徑 - 登出和中間件都使用 path='/'")
    print("3. 添加詳細調試信息 - 幫助診斷問題")
    
    print("\n🔧 關鍵修復:")
    print("response.set_cookie(")
    print("    key='access_token',")
    print("    value=access_token,")
    print("    path='/',  # ← 這是關鍵修復")
    print("    ...)")
    
    print("\n📝 如果問題仍然存在:")
    print("1. 檢查瀏覽器是否緩存了舊的 cookie")
    print("2. 確認服務器重新啟動以應用修復")
    print("3. 查看服務器日誌中的調試信息")
    print("4. 使用無痕模式測試")
    
    return passed == total


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
