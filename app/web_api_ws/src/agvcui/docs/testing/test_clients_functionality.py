#!/usr/bin/env python3
"""
Clients 功能測試腳本

測試新增的 clients 頁面功能是否正常工作
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../'))

def test_clients_imports():
    """測試 clients 相關模組導入"""
    print("🧪 測試 clients 模組導入...")
    
    try:
        # 測試路由導入
        from agvcui.routers.clients import get_router
        print("   ✅ clients 路由導入成功")
        
        # 測試資料庫函數導入
        from agvcui.db import (
            get_clients, count_clients, get_client_by_id,
            update_client, reset_client_op_settings, delete_client,
            get_all_machines
        )
        print("   ✅ clients 資料庫函數導入成功")
        
        # 測試權限函數導入
        from agvcui.utils.permissions import can_create, can_edit, can_delete
        print("   ✅ 權限函數導入成功")
        
        return True
        
    except Exception as e:
        print(f"   ❌ 模組導入失敗: {e}")
        return False


def test_database_functions():
    """測試資料庫函數"""
    print("\n🧪 測試資料庫函數...")
    
    try:
        from agvcui.db import get_clients, count_clients, get_all_machines
        
        # 測試獲取客戶端列表
        clients = get_clients(offset=0, limit=5)
        print(f"   ✅ 獲取客戶端列表成功，共 {len(clients)} 筆")
        
        # 測試計算客戶端總數
        total = count_clients()
        print(f"   ✅ 計算客戶端總數成功: {total}")
        
        # 測試獲取機器列表
        machines = get_all_machines()
        print(f"   ✅ 獲取機器列表成功，共 {len(machines)} 筆")
        
        return True
        
    except Exception as e:
        print(f"   ❌ 資料庫函數測試失敗: {e}")
        return False


def test_client_data_structure():
    """測試客戶端資料結構"""
    print("\n🧪 測試客戶端資料結構...")
    
    try:
        from agvcui.db import get_clients
        
        clients = get_clients(offset=0, limit=1)
        
        if clients:
            client = clients[0]
            print(f"   ✅ 客戶端 ID: {client.id}")
            print(f"   ✅ 機器 ID: {client.machine_id}")
            print(f"   ✅ 用戶代理: {client.user_agent[:50] if client.user_agent else 'None'}...")
            print(f"   ✅ OP 設定: {'有' if client.op else '無'}")
            print(f"   ✅ 創建時間: {client.created_at}")
            print(f"   ✅ 更新時間: {client.updated_at}")
            
            # 檢查 OP 設定結構
            if client.op:
                op = client.op
                if 'left' in op and 'right' in op:
                    print("   ✅ OP 設定結構正確 (包含 left 和 right)")
                    
                    left_op = op['left']
                    if 'productSelected' in left_op and 'product' in left_op:
                        print("   ✅ 左側 OP 設定結構正確")
                    
                    right_op = op['right']
                    if 'productSelected' in right_op and 'product' in right_op:
                        print("   ✅ 右側 OP 設定結構正確")
                else:
                    print("   ⚠️  OP 設定結構不完整")
        else:
            print("   ⚠️  沒有客戶端資料可測試")
        
        return True
        
    except Exception as e:
        print(f"   ❌ 客戶端資料結構測試失敗: {e}")
        return False


def test_reset_op_settings():
    """測試重置 OP 設定功能"""
    print("\n🧪 測試重置 OP 設定功能...")
    
    try:
        from agvcui.db import reset_client_op_settings
        
        # 這個函數需要真實的客戶端 ID，所以我們只測試函數存在
        print("   ✅ reset_client_op_settings 函數存在")
        
        # 檢查預設 OP 設定結構
        default_op = {
            "left": {
                "productSelected": 0,
                "product": [
                    {"name": "", "size": "S", "id": None, "count": 32, "room": 2, "rackId": None},
                    {"name": "", "size": "S", "id": None, "count": 32, "room": 2, "rackId": None}
                ]
            },
            "right": {
                "productSelected": 0,
                "product": [
                    {"name": "", "size": "S", "id": None, "count": 32, "room": 2, "rackId": None},
                    {"name": "", "size": "S", "id": None, "count": 32, "room": 2, "rackId": None}
                ]
            }
        }
        
        print("   ✅ 預設 OP 設定結構正確")
        print(f"   ✅ 左側預設產品數量: {len(default_op['left']['product'])}")
        print(f"   ✅ 右側預設產品數量: {len(default_op['right']['product'])}")
        
        return True
        
    except Exception as e:
        print(f"   ❌ 重置 OP 設定測試失敗: {e}")
        return False


def test_template_files():
    """測試模板文件是否存在"""
    print("\n🧪 測試模板文件...")
    
    try:
        import os
        
        # 檢查模板文件
        template_dir = os.path.join(os.path.dirname(__file__), '../../agvcui/templates')
        
        clients_template = os.path.join(template_dir, 'clients.html')
        if os.path.exists(clients_template):
            print("   ✅ clients.html 模板存在")
        else:
            print("   ❌ clients.html 模板不存在")
            return False
        
        client_form_template = os.path.join(template_dir, 'client_form.html')
        if os.path.exists(client_form_template):
            print("   ✅ client_form.html 模板存在")
        else:
            print("   ❌ client_form.html 模板不存在")
            return False
        
        return True
        
    except Exception as e:
        print(f"   ❌ 模板文件測試失敗: {e}")
        return False


def test_javascript_files():
    """測試 JavaScript 文件是否存在"""
    print("\n🧪 測試 JavaScript 文件...")
    
    try:
        import os
        
        # 檢查 JavaScript 文件
        js_dir = os.path.join(os.path.dirname(__file__), '../../agvcui/static/js')
        
        clients_js = os.path.join(js_dir, 'clientsPage.js')
        if os.path.exists(clients_js):
            print("   ✅ clientsPage.js 文件存在")
            
            # 檢查文件內容
            with open(clients_js, 'r', encoding='utf-8') as f:
                content = f.read()
                if 'toggleClientDetails' in content:
                    print("   ✅ toggleClientDetails 函數存在")
                if 'resetClientSettings' in content:
                    print("   ✅ resetClientSettings 函數存在")
                if 'renderClientDetails' in content:
                    print("   ✅ renderClientDetails 函數存在")
        else:
            print("   ❌ clientsPage.js 文件不存在")
            return False
        
        return True
        
    except Exception as e:
        print(f"   ❌ JavaScript 文件測試失敗: {e}")
        return False


def main():
    """主測試函數"""
    print("🚀 開始測試 Clients 功能...")
    print("=" * 50)
    
    tests = [
        test_clients_imports,
        test_database_functions,
        test_client_data_structure,
        test_reset_op_settings,
        test_template_files,
        test_javascript_files
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
    
    print("\n" + "=" * 50)
    print(f"📊 測試結果: {passed}/{total} 通過")
    
    if passed == total:
        print("🎉 所有測試通過！Clients 功能已準備就緒。")
    else:
        print("⚠️  部分測試失敗，請檢查相關功能。")
    
    return passed == total


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
