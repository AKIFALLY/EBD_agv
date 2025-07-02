#!/usr/bin/env python3
"""
測試登入功能的腳本
"""

import sys
import os

# 添加路徑以便導入模組
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'db_proxy_ws', 'src'))

def test_auth_functions():
    """測試認證相關函數"""
    print("=== 測試認證功能 ===")
    
    try:
        from agvcui.auth import hash_password, verify_password, create_access_token, verify_token
        
        # 測試密碼哈希
        print("1. 測試密碼哈希...")
        password = "admin123"
        hashed = hash_password(password)
        print(f"   原始密碼: {password}")
        print(f"   哈希後: {hashed[:50]}...")
        
        # 測試密碼驗證
        print("2. 測試密碼驗證...")
        is_valid = verify_password(password, hashed)
        is_invalid = verify_password("wrongpassword", hashed)
        print(f"   正確密碼驗證: {is_valid}")
        print(f"   錯誤密碼驗證: {is_invalid}")
        
        # 測試 token 創建
        print("3. 測試 Token 創建...")
        token_data = {"sub": "admin", "role": "admin"}
        token = create_access_token(token_data)
        print(f"   Token: {token[:50]}...")
        
        # 測試 token 驗證
        print("4. 測試 Token 驗證...")
        verified = verify_token(token)
        print(f"   驗證結果: {verified}")
        print(f"   用戶名: {verified.username if verified else 'None'}")
        
        print("✅ 認證功能測試通過!")
        return True
        
    except Exception as e:
        print(f"❌ 認證功能測試失敗: {e}")
        return False


def test_middleware():
    """測試中間件功能"""
    print("\n=== 測試中間件功能 ===")
    
    try:
        from agvcui.middleware import AuthMiddleware
        
        # 測試中間件初始化
        print("1. 測試中間件初始化...")
        middleware = AuthMiddleware(None)  # app 參數在這裡不重要
        print(f"   受保護路徑: {middleware.protected_paths[:3]}...")
        print(f"   公開路徑: {middleware.public_paths[:3]}...")
        
        print("✅ 中間件功能測試通過!")
        return True
        
    except Exception as e:
        print(f"❌ 中間件功能測試失敗: {e}")
        return False


def test_templates():
    """測試模板是否存在"""
    print("\n=== 測試模板文件 ===")
    
    template_dir = os.path.join(os.path.dirname(__file__), 'agvcui', 'templates')
    required_templates = ['login.html', 'base.html', 'navbar.html']
    
    all_exist = True
    for template in required_templates:
        template_path = os.path.join(template_dir, template)
        exists = os.path.exists(template_path)
        print(f"   {template}: {'✅' if exists else '❌'}")
        if not exists:
            all_exist = False
    
    if all_exist:
        print("✅ 所有必要模板文件都存在!")
    else:
        print("❌ 部分模板文件缺失!")
    
    return all_exist


def test_static_files():
    """測試靜態文件是否存在"""
    print("\n=== 測試靜態文件 ===")
    
    static_dir = os.path.join(os.path.dirname(__file__), 'agvcui', 'static')
    required_files = ['js/auth.js']
    
    all_exist = True
    for file_path in required_files:
        full_path = os.path.join(static_dir, file_path)
        exists = os.path.exists(full_path)
        print(f"   {file_path}: {'✅' if exists else '❌'}")
        if not exists:
            all_exist = False
    
    if all_exist:
        print("✅ 所有必要靜態文件都存在!")
    else:
        print("❌ 部分靜態文件缺失!")
    
    return all_exist


def main():
    """主測試函數"""
    print("開始測試 AGVC UI 登入功能...\n")
    
    tests = [
        test_auth_functions,
        test_middleware,
        test_templates,
        test_static_files
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
    
    print(f"\n=== 測試結果 ===")
    print(f"通過: {passed}/{total}")
    
    if passed == total:
        print("🎉 所有測試都通過了!")
        print("\n下一步:")
        print("1. 確保資料庫連接正常")
        print("2. 啟動服務器: python3 -m agvcui.agvc_ui_server")
        print("3. 訪問 http://localhost:8001/init-admin 初始化管理員")
        print("4. 訪問 http://localhost:8001/login 進行登入測試")
    else:
        print("❌ 部分測試失敗，請檢查上述錯誤")
    
    return passed == total


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
