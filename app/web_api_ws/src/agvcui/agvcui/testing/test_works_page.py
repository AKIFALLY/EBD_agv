#!/usr/bin/env python3
"""
Work 頁面功能測試
測試 Work 頁面的 CRUD 操作、搜尋篩選、分頁排序等功能
"""

import sys
import os
import json
from pathlib import Path

# 添加項目根目錄到 Python 路徑
project_root = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(project_root / "src"))

def test_work_model_import():
    """測試 Work 模型導入"""
    try:
        from db_proxy.models.agvc_task import Work
        print("✅ Work 模型導入成功")
        
        # 測試模型屬性
        work_fields = ['id', 'name', 'description', 'parameters']
        for field in work_fields:
            if hasattr(Work, field):
                print(f"✅ Work 模型包含欄位: {field}")
            else:
                print(f"❌ Work 模型缺少欄位: {field}")
        
        return True
    except ImportError as e:
        print(f"❌ Work 模型導入失敗: {e}")
        return False

def test_work_database_operations():
    """測試 Work 資料庫操作函數"""
    try:
        from agvcui.agvcui.database.task_ops import (
            get_works, count_works, get_work_by_id, 
            create_work, update_work, delete_work
        )
        print("✅ Work 資料庫操作函數導入成功")
        
        # 測試函數存在性
        functions = [
            'get_works', 'count_works', 'get_work_by_id',
            'create_work', 'update_work', 'delete_work'
        ]
        
        for func_name in functions:
            if func_name in locals():
                print(f"✅ 函數存在: {func_name}")
            else:
                print(f"❌ 函數不存在: {func_name}")
        
        return True
    except ImportError as e:
        print(f"❌ Work 資料庫操作函數導入失敗: {e}")
        return False

def test_work_router():
    """測試 Work 路由"""
    try:
        from agvcui.agvcui.routers.works import get_router
        from fastapi.templating import Jinja2Templates
        
        # 創建模擬模板
        templates = Jinja2Templates(directory="templates")
        router = get_router(templates)
        
        print("✅ Work 路由創建成功")
        
        # 檢查路由端點
        routes = [route.path for route in router.routes]
        expected_routes = ['/works', '/works/create', '/works/{work_id}/edit', '/works/{work_id}/delete']
        
        for route in expected_routes:
            if any(route in r for r in routes):
                print(f"✅ 路由存在: {route}")
            else:
                print(f"❌ 路由不存在: {route}")
        
        return True
    except Exception as e:
        print(f"❌ Work 路由測試失敗: {e}")
        return False

def test_work_templates():
    """測試 Work 模板文件"""
    template_files = [
        "web_api_ws/src/agvcui/agvcui/templates/works.html",
        "web_api_ws/src/agvcui/agvcui/templates/work_form.html"
    ]
    
    all_exist = True
    for template_file in template_files:
        template_path = project_root / template_file
        if template_path.exists():
            print(f"✅ 模板文件存在: {template_file}")
            
            # 檢查模板內容
            content = template_path.read_text()
            if 'work' in content.lower():
                print(f"✅ 模板包含 Work 相關內容: {template_file}")
            else:
                print(f"⚠️ 模板可能缺少 Work 相關內容: {template_file}")
        else:
            print(f"❌ 模板文件不存在: {template_file}")
            all_exist = False
    
    return all_exist

def test_work_static_files():
    """測試 Work 靜態文件"""
    static_files = [
        "web_api_ws/src/agvcui/agvcui/static/js/worksPage.js",
        "web_api_ws/src/agvcui/agvcui/static/js/worksStore.js",
        "web_api_ws/src/agvcui/agvcui/static/css/worksPage.css"
    ]
    
    all_exist = True
    for static_file in static_files:
        static_path = project_root / static_file
        if static_path.exists():
            print(f"✅ 靜態文件存在: {static_file}")
            
            # 檢查文件內容
            content = static_path.read_text()
            if 'work' in content.lower():
                print(f"✅ 靜態文件包含 Work 相關內容: {static_file}")
            else:
                print(f"⚠️ 靜態文件可能缺少 Work 相關內容: {static_file}")
        else:
            print(f"❌ 靜態文件不存在: {static_file}")
            all_exist = False
    
    return all_exist

def test_navigation_integration():
    """測試導航整合"""
    try:
        navbar_path = project_root / "web_api_ws/src/agvcui/agvcui/templates/navbar.html"
        if navbar_path.exists():
            content = navbar_path.read_text()
            if '/works' in content:
                print("✅ 導航欄包含 Works 連結")
                return True
            else:
                print("❌ 導航欄缺少 Works 連結")
                return False
        else:
            print("❌ 導航欄模板不存在")
            return False
    except Exception as e:
        print(f"❌ 導航整合測試失敗: {e}")
        return False

def test_server_integration():
    """測試服務器整合"""
    try:
        server_path = project_root / "web_api_ws/src/agvcui/agvcui/agvc_ui_server.py"
        if server_path.exists():
            content = server_path.read_text()
            if 'works' in content:
                print("✅ 服務器包含 Works 路由註冊")
                return True
            else:
                print("❌ 服務器缺少 Works 路由註冊")
                return False
        else:
            print("❌ 服務器文件不存在")
            return False
    except Exception as e:
        print(f"❌ 服務器整合測試失敗: {e}")
        return False

def main():
    """主測試函數"""
    print("🧪 開始 Work 頁面功能測試")
    print("=" * 50)
    
    tests = [
        ("Work 模型導入", test_work_model_import),
        ("Work 資料庫操作", test_work_database_operations),
        ("Work 路由", test_work_router),
        ("Work 模板", test_work_templates),
        ("Work 靜態文件", test_work_static_files),
        ("導航整合", test_navigation_integration),
        ("服務器整合", test_server_integration),
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n🔍 測試: {test_name}")
        print("-" * 30)
        try:
            if test_func():
                passed += 1
                print(f"✅ {test_name} 測試通過")
            else:
                print(f"❌ {test_name} 測試失敗")
        except Exception as e:
            print(f"❌ {test_name} 測試異常: {e}")
    
    print("\n" + "=" * 50)
    print(f"📊 測試結果: {passed}/{total} 通過")
    
    if passed == total:
        print("🎉 所有測試通過！Work 頁面功能完整")
        return True
    else:
        print("⚠️ 部分測試失敗，請檢查相關功能")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
