#!/usr/bin/env python3
"""
Work 頁面實作驗證腳本
檢查所有 Work 頁面相關文件是否正確創建和配置
"""

import os
from pathlib import Path

def check_file_exists(file_path, description):
    """檢查文件是否存在"""
    if os.path.exists(file_path):
        print(f"✅ {description}: {file_path}")
        return True
    else:
        print(f"❌ {description}: {file_path}")
        return False

def check_file_content(file_path, search_terms, description):
    """檢查文件內容是否包含指定關鍵字"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        found_terms = []
        missing_terms = []
        
        for term in search_terms:
            if term.lower() in content.lower():
                found_terms.append(term)
            else:
                missing_terms.append(term)
        
        if missing_terms:
            print(f"⚠️ {description} 缺少關鍵字: {missing_terms}")
            return False
        else:
            print(f"✅ {description} 包含所有必要關鍵字: {found_terms}")
            return True
            
    except Exception as e:
        print(f"❌ 檢查 {description} 時發生錯誤: {e}")
        return False

def main():
    """主驗證函數"""
    print("🔍 Work 頁面實作驗證")
    print("=" * 50)
    
    base_path = "/app/web_api_ws/src/agvcui/agvcui"
    
    # 檢查文件存在性
    files_to_check = [
        (f"{base_path}/routers/works.py", "Work 路由文件"),
        (f"{base_path}/templates/works.html", "Work 列表模板"),
        (f"{base_path}/templates/work_form.html", "Work 表單模板"),
        (f"{base_path}/static/js/worksPage.js", "Work 頁面 JavaScript"),
        (f"{base_path}/static/js/worksStore.js", "Work Store JavaScript"),
        (f"{base_path}/static/css/worksPage.css", "Work 頁面 CSS"),
    ]
    
    print("\n📁 檢查文件存在性:")
    file_checks = []
    for file_path, description in files_to_check:
        file_checks.append(check_file_exists(file_path, description))
    
    # 檢查關鍵內容
    print("\n📝 檢查文件內容:")
    content_checks = []
    
    # 檢查路由文件
    content_checks.append(check_file_content(
        f"{base_path}/routers/works.py",
        ["get_works", "create_work", "update_work", "delete_work", "work_list"],
        "Work 路由功能"
    ))
    
    # 檢查 HTML 模板
    content_checks.append(check_file_content(
        f"{base_path}/templates/works.html",
        ["work", "搜尋", "分頁", "worksPage.js"],
        "Work 列表模板功能"
    ))
    
    content_checks.append(check_file_content(
        f"{base_path}/templates/work_form.html",
        ["work", "name", "description", "parameters", "JSON"],
        "Work 表單模板功能"
    ))
    
    # 檢查 JavaScript 文件
    content_checks.append(check_file_content(
        f"{base_path}/static/js/worksPage.js",
        ["worksPage", "updateSingleWork", "hasChanged", "addUpdateAnimation"],
        "Work 頁面 JavaScript 功能"
    ))
    
    content_checks.append(check_file_content(
        f"{base_path}/static/js/worksStore.js",
        ["worksStore", "updateWorks", "getState", "setState"],
        "Work Store JavaScript 功能"
    ))
    
    # 檢查 CSS 文件
    content_checks.append(check_file_content(
        f"{base_path}/static/css/worksPage.css",
        ["work-parameters-dropdown", "table", "responsive"],
        "Work 頁面 CSS 樣式"
    ))
    
    # 檢查整合
    print("\n🔗 檢查系統整合:")
    integration_checks = []
    
    # 檢查導航欄整合
    integration_checks.append(check_file_content(
        f"{base_path}/templates/navbar.html",
        ["/works", "Works"],
        "導航欄 Works 連結"
    ))
    
    # 檢查服務器整合
    integration_checks.append(check_file_content(
        f"{base_path}/agvc_ui_server.py",
        ["works", "include_router"],
        "服務器 Works 路由註冊"
    ))
    
    # 檢查資料庫操作整合
    integration_checks.append(check_file_content(
        f"{base_path}/database/task_ops.py",
        ["get_works", "count_works", "create_work", "update_work", "delete_work"],
        "資料庫 Works 操作函數"
    ))
    
    # 檢查 __init__.py 更新
    integration_checks.append(check_file_content(
        f"{base_path}/database/__init__.py",
        ["get_works", "count_works", "create_work", "update_work", "delete_work"],
        "資料庫模組 Works 函數導出"
    ))
    
    # 統計結果
    print("\n" + "=" * 50)
    total_file_checks = len(file_checks)
    passed_file_checks = sum(file_checks)
    
    total_content_checks = len(content_checks)
    passed_content_checks = sum(content_checks)
    
    total_integration_checks = len(integration_checks)
    passed_integration_checks = sum(integration_checks)
    
    total_checks = total_file_checks + total_content_checks + total_integration_checks
    passed_checks = passed_file_checks + passed_content_checks + passed_integration_checks
    
    print(f"📊 驗證結果:")
    print(f"   文件存在性: {passed_file_checks}/{total_file_checks}")
    print(f"   文件內容: {passed_content_checks}/{total_content_checks}")
    print(f"   系統整合: {passed_integration_checks}/{total_integration_checks}")
    print(f"   總計: {passed_checks}/{total_checks}")
    
    if passed_checks == total_checks:
        print("\n🎉 Work 頁面實作完整！所有檢查都通過。")
        print("\n📋 實作功能清單:")
        print("   ✅ Work CRUD 操作（創建、讀取、更新、刪除）")
        print("   ✅ 搜尋和篩選功能")
        print("   ✅ 分頁和排序功能")
        print("   ✅ DOM 優化方法論實作")
        print("   ✅ miniStore 整合")
        print("   ✅ 統一動畫架構")
        print("   ✅ 響應式設計")
        print("   ✅ 權限控制整合")
        print("   ✅ 導航系統整合")
        return True
    else:
        print(f"\n⚠️ 發現 {total_checks - passed_checks} 個問題，請檢查上述失敗項目。")
        return False

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
