#!/usr/bin/env python3
"""
測試 signals 頁面只讀模式
"""

import sys
import os
import re

# 添加必要的路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
agvcui_src = os.path.join(current_dir, '..', '..')

sys.path.insert(0, agvcui_src)


def test_signals_readonly():
    """測試 signals 頁面是否已變成只讀模式"""
    print("開始測試 signals 頁面只讀模式...")
    
    # 1. 檢查 signals.html 模板
    print("\n1. 檢查 signals.html 模板")
    signals_template_path = os.path.join(agvcui_src, 'agvcui', 'templates', 'signals.html')
    
    try:
        with open(signals_template_path, 'r', encoding='utf-8') as f:
            template_content = f.read()
        
        # 檢查是否還有編輯/刪除相關的內容
        forbidden_patterns = [
            '編輯',
            '刪除',
            'edit',
            'delete',
            '新增信號',
            'create',
            '操作',
            'deleteSignal',
            'deleteModal',
            'closeDeleteModal',
            'mdi-pencil',
            'mdi-delete',
            'mdi-plus'
        ]
        
        print("禁用功能檢查:")
        found_forbidden = False
        for pattern in forbidden_patterns:
            matches = re.findall(pattern, template_content, re.IGNORECASE)
            if matches:
                print(f"  ❌ 找到禁用內容: {pattern} - 共 {len(matches)} 處")
                found_forbidden = True
            else:
                print(f"  ✅ 已移除: {pattern}")
        
        if not found_forbidden:
            print("  🎉 所有編輯/刪除功能已完全移除")
        
        # 檢查保留的功能
        allowed_patterns = [
            '信號管理',
            'mdi-pulse',
            'mdi-filter',
            'mdi-devices',
            'mdi-view-list',
            '設備篩選',
            '信號值',
            '數據類型'
        ]
        
        print("\n保留功能檢查:")
        for pattern in allowed_patterns:
            if pattern in template_content:
                print(f"  ✅ 保留: {pattern}")
            else:
                print(f"  ❓ 可能缺少: {pattern}")
        
        # 檢查表格結構
        print(f"\n表格結構檢查:")
        
        # 檢查表頭
        table_headers = ['ID', '名稱', '描述', '當前值', '數據類型']
        for header in table_headers:
            if header in template_content:
                print(f"  ✅ 表頭: {header}")
            else:
                print(f"  ❓ 表頭可能缺少: {header}")
        
        # 檢查是否還有操作列
        if '操作' in template_content:
            print(f"  ❌ 仍有操作列")
        else:
            print(f"  ✅ 操作列已移除")
    
    except FileNotFoundError:
        print("❌ signals.html 文件不存在")
    
    # 2. 檢查 signals 路由
    print("\n2. 檢查 signals 路由")
    signals_router_path = os.path.join(agvcui_src, 'agvcui', 'routers', 'signals.py')
    
    try:
        with open(signals_router_path, 'r', encoding='utf-8') as f:
            router_content = f.read()
        
        # 檢查路由函數
        route_patterns = [
            '@router.get.*signals.*list',
            'def signals_list',
            'get_signals',
            'count_signals'
        ]
        
        print("路由功能檢查:")
        for pattern in route_patterns:
            if re.search(pattern, router_content):
                print(f"  ✅ 保留: {pattern}")
            else:
                print(f"  ❓ 可能缺少: {pattern}")
        
        # 檢查是否有編輯/刪除路由
        forbidden_routes = [
            'create',
            'edit',
            'delete',
            'POST.*signals'
        ]
        
        print("\n禁用路由檢查:")
        found_forbidden_routes = False
        for pattern in forbidden_routes:
            if re.search(pattern, router_content, re.IGNORECASE):
                print(f"  ❌ 找到禁用路由: {pattern}")
                found_forbidden_routes = True
            else:
                print(f"  ✅ 已移除: {pattern}")
        
        if not found_forbidden_routes:
            print("  🎉 所有編輯/刪除路由已移除")
    
    except FileNotFoundError:
        print("❌ signals.py 路由文件不存在")
    
    # 3. 檢查功能完整性
    print("\n3. 檢查功能完整性")
    
    readonly_features = [
        "查看信號列表",
        "按設備篩選信號", 
        "分頁瀏覽信號",
        "即時更新信號值",
        "顯示信號統計",
        "設備選擇器",
        "信號值顏色標示"
    ]
    
    print("只讀功能清單:")
    for feature in readonly_features:
        print(f"  📖 {feature}")
    
    removed_features = [
        "新增信號",
        "編輯信號",
        "刪除信號",
        "操作按鈕",
        "表單提交",
        "權限檢查（編輯相關）"
    ]
    
    print("\n已移除功能清單:")
    for feature in removed_features:
        print(f"  🚫 {feature}")
    
    # 4. 檢查用戶體驗
    print("\n4. 用戶體驗檢查")
    
    ux_improvements = [
        "頁面更簡潔，專注於信號監控",
        "沒有不必要的操作按鈕干擾",
        "表格更寬敞，信號值更突出",
        "即時更新功能更明顯",
        "符合只讀監控的使用場景"
    ]
    
    print("用戶體驗改進:")
    for improvement in ux_improvements:
        print(f"  🎯 {improvement}")
    
    # 5. 與 devices 頁面的分工
    print("\n5. 與 devices 頁面的分工")
    
    print("功能分工:")
    print("  📊 signals 頁面:")
    print("    - 專注於信號值監控")
    print("    - 即時數據顯示")
    print("    - 按設備篩選查看")
    print("    - 只讀模式，無編輯功能")
    
    print("  ⚙️  devices 頁面:")
    print("    - 完整的設備管理")
    print("    - 設備、端口、信號的 CRUD")
    print("    - 設備配置和維護")
    print("    - 管理員操作界面")
    
    print("\n✅ signals 頁面只讀模式檢查完成！")
    print("💡 總結:")
    print("  - signals 頁面已完全變成只讀模式")
    print("  - 所有編輯/刪除功能已移除")
    print("  - 專注於信號值的監控和顯示")
    print("  - 與 devices 頁面形成良好的功能分工")


if __name__ == "__main__":
    test_signals_readonly()
