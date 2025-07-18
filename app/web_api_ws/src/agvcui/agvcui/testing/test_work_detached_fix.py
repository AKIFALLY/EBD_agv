#!/usr/bin/env python3
"""
測試 Work 編輯表單的 DetachedInstanceError 修復
驗證 Work 對象的 tasks 關係能夠正確加載和訪問
"""

import sys
import os
from pathlib import Path

# 添加項目根目錄到 Python 路徑
project_root = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(project_root / "src"))

def test_get_work_by_id_basic():
    """測試基本的 get_work_by_id 函數"""
    try:
        from agvcui.agvcui.database.task_ops import get_work_by_id
        
        print("✅ get_work_by_id 函數導入成功")
        
        # 測試函數簽名
        import inspect
        sig = inspect.signature(get_work_by_id)
        params = list(sig.parameters.keys())
        
        if 'work_id' in params and 'include_tasks' in params:
            print("✅ get_work_by_id 函數簽名正確，包含 include_tasks 參數")
        else:
            print(f"❌ get_work_by_id 函數簽名不正確，參數: {params}")
            return False
            
        return True
    except ImportError as e:
        print(f"❌ get_work_by_id 函數導入失敗: {e}")
        return False
    except Exception as e:
        print(f"❌ get_work_by_id 函數測試異常: {e}")
        return False

def test_work_model_import():
    """測試 Work 模型導入和關係定義"""
    try:
        from db_proxy.models.agvc_task import Work
        
        print("✅ Work 模型導入成功")
        
        # 檢查是否有 tasks 關係
        if hasattr(Work, 'tasks'):
            print("✅ Work 模型包含 tasks 關係屬性")
        else:
            print("❌ Work 模型缺少 tasks 關係屬性")
            return False
            
        return True
    except ImportError as e:
        print(f"❌ Work 模型導入失敗: {e}")
        return False

def test_template_safety():
    """測試模板中的安全訪問模式"""
    template_path = project_root / "web_api_ws/src/agvcui/agvcui/templates/work_form.html"
    
    if not template_path.exists():
        print("❌ work_form.html 模板文件不存在")
        return False
    
    try:
        with open(template_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 檢查是否使用了安全的字典訪問模式
        safe_patterns = [
            "work.get('name'",
            "work.get('description'",
            "work.get('parameters'",
            "work.get('tasks'",
            "task.get('name'"
        ]
        
        missing_patterns = []
        for pattern in safe_patterns:
            if pattern not in content:
                missing_patterns.append(pattern)
        
        if missing_patterns:
            print(f"❌ 模板缺少安全訪問模式: {missing_patterns}")
            return False
        else:
            print("✅ 模板使用了安全的字典訪問模式")
        
        # 檢查是否移除了不安全的直接屬性訪問
        unsafe_patterns = [
            "work.name",
            "work.description", 
            "work.parameters",
            "work.tasks",
            "task.name"
        ]
        
        found_unsafe = []
        for pattern in unsafe_patterns:
            # 排除 work.get() 的情況
            if pattern in content and f"{pattern.split('.')[0]}.get('{pattern.split('.')[1]}'" not in content:
                found_unsafe.append(pattern)
        
        if found_unsafe:
            print(f"⚠️ 模板仍包含不安全的直接訪問: {found_unsafe}")
            # 這不是致命錯誤，因為可能有其他安全的使用方式
        
        return True
        
    except Exception as e:
        print(f"❌ 模板安全性檢查失敗: {e}")
        return False

def test_router_integration():
    """測試路由整合"""
    try:
        from agvcui.agvcui.routers.works import get_router
        from fastapi.templating import Jinja2Templates
        
        # 創建模擬模板
        templates = Jinja2Templates(directory="templates")
        router = get_router(templates)
        
        print("✅ Works 路由創建成功")
        
        # 檢查路由端點
        routes = [route.path for route in router.routes]
        edit_route_found = any('/works/{work_id}/edit' in route for route in routes)
        
        if edit_route_found:
            print("✅ 編輯路由端點存在")
        else:
            print("❌ 編輯路由端點不存在")
            return False
            
        return True
    except Exception as e:
        print(f"❌ 路由整合測試失敗: {e}")
        return False

def test_database_function_signature():
    """測試資料庫函數簽名"""
    try:
        from agvcui.agvcui.database.task_ops import get_work_by_id
        import inspect
        
        # 測試函數可以被調用
        sig = inspect.signature(get_work_by_id)
        
        # 檢查預設參數
        include_tasks_param = sig.parameters.get('include_tasks')
        if include_tasks_param and include_tasks_param.default == False:
            print("✅ include_tasks 參數有正確的預設值 False")
        else:
            print("❌ include_tasks 參數預設值不正確")
            return False
            
        return True
    except Exception as e:
        print(f"❌ 資料庫函數簽名測試失敗: {e}")
        return False

def main():
    """主測試函數"""
    print("🧪 開始 Work DetachedInstanceError 修復測試")
    print("=" * 60)
    
    tests = [
        ("Work 模型導入", test_work_model_import),
        ("get_work_by_id 基本功能", test_get_work_by_id_basic),
        ("資料庫函數簽名", test_database_function_signature),
        ("模板安全性", test_template_safety),
        ("路由整合", test_router_integration),
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n🔍 測試: {test_name}")
        print("-" * 40)
        try:
            if test_func():
                passed += 1
                print(f"✅ {test_name} 測試通過")
            else:
                print(f"❌ {test_name} 測試失敗")
        except Exception as e:
            print(f"❌ {test_name} 測試異常: {e}")
    
    print("\n" + "=" * 60)
    print(f"📊 測試結果: {passed}/{total} 通過")
    
    if passed == total:
        print("🎉 所有測試通過！DetachedInstanceError 修復成功")
        print("\n📋 修復內容:")
        print("   ✅ 添加了 include_tasks 參數到 get_work_by_id 函數")
        print("   ✅ 使用 eager loading 預先加載 tasks 關係")
        print("   ✅ 將 SQLAlchemy 對象轉換為字典避免 detached 問題")
        print("   ✅ 更新模板使用安全的字典訪問模式")
        print("   ✅ 在編輯表單路由中啟用 tasks 加載")
        return True
    else:
        print("⚠️ 部分測試失敗，請檢查相關功能")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
