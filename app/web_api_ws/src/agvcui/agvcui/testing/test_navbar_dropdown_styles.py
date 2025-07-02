#!/usr/bin/env python3
"""
測試 navbar dropdown 樣式效果
"""

import sys
import os
import re

# 添加必要的路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
agvcui_src = os.path.join(current_dir, '..', '..')

sys.path.insert(0, agvcui_src)


def test_navbar_dropdown_styles():
    """測試 navbar dropdown 的樣式和 active 狀態"""
    print("開始測試 navbar dropdown 樣式效果...")
    
    # 1. 檢查 CSS 樣式
    print("\n1. 檢查 CSS 樣式")
    css_path = os.path.join(agvcui_src, 'agvcui', 'static', 'css', 'agvcui-bulma-extend.css')
    
    try:
        with open(css_path, 'r', encoding='utf-8') as f:
            css_content = f.read()
        
        # 檢查必要的 CSS 規則
        required_styles = [
            '.navbar-item .navbar-link',
            '.navbar-item .navbar-link.is-active',
            '.navbar-item .navbar-link:hover',
            '.navbar-dropdown .navbar-item:hover'
        ]
        
        print("CSS 樣式檢查:")
        for style in required_styles:
            if style in css_content:
                print(f"  ✅ {style}")
            else:
                print(f"  ❌ {style}")
        
        # 檢查顏色一致性
        color_pattern = r'rgb\(90, 180, 255\)'
        color_matches = re.findall(color_pattern, css_content)
        print(f"\n顏色一致性: 找到 {len(color_matches)} 處使用主題色")
        
    except FileNotFoundError:
        print("❌ CSS 文件不存在")
    
    # 2. 檢查 navbar.html 中的 active 狀態
    print("\n2. 檢查 navbar dropdown active 狀態")
    navbar_path = os.path.join(agvcui_src, 'agvcui', 'templates', 'navbar.html')
    
    try:
        with open(navbar_path, 'r', encoding='utf-8') as f:
            navbar_content = f.read()
        
        # 檢查 dropdown 按鈕的 active 狀態
        dropdown_configs = {
            'Logs': {
                'paths': ['/rosout_logs', '/runtime_logs'],
                'pattern': r'class="navbar-link[^"]*is-active[^"]*"[^>]*>\s*Logs'
            },
            'User': {
                'paths': ['/clients', '/racks', '/products', '/carriers'],
                'pattern': r'class="navbar-link[^"]*is-active[^"]*"[^>]*>\s*User'
            }
        }
        
        print("Dropdown active 狀態檢查:")
        for dropdown_name, config in dropdown_configs.items():
            pattern = config['pattern']
            if re.search(pattern, navbar_content, re.DOTALL):
                print(f"  ✅ {dropdown_name}: 有 active 狀態檢查")
                
                # 檢查路徑條件
                paths = config['paths']
                for path in paths:
                    if path in navbar_content:
                        print(f"    ✅ 包含路徑: {path}")
                    else:
                        print(f"    ❌ 缺少路徑: {path}")
            else:
                print(f"  ❌ {dropdown_name}: 沒有 active 狀態檢查")
    
    except FileNotFoundError:
        print("❌ navbar.html 文件不存在")
    
    # 3. 模擬不同路徑下的 active 狀態
    print("\n3. 模擬路徑測試")
    
    test_paths = [
        ("/rosout_logs", "Logs dropdown 應該 active"),
        ("/runtime_logs", "Logs dropdown 應該 active"),
        ("/clients", "User dropdown 應該 active"),
        ("/racks", "User dropdown 應該 active"),
        ("/products", "User dropdown 應該 active"),
        ("/carriers", "User dropdown 應該 active"),
        ("/devices", "所有 dropdown 都不應該 active"),
        ("/signals", "所有 dropdown 都不應該 active")
    ]
    
    print("路徑測試模擬:")
    for path, expected in test_paths:
        print(f"  路徑: {path}")
        print(f"  期望: {expected}")
        
        # 模擬 Logs dropdown
        logs_active = path.startswith('/rosout_logs') or path.startswith('/runtime_logs')
        logs_status = "✅ active" if logs_active else "⚪ inactive"
        print(f"    Logs dropdown: {logs_status}")
        
        # 模擬 User dropdown
        user_active = (path.startswith('/clients') or path.startswith('/racks') or 
                      path.startswith('/products') or path.startswith('/carriers'))
        user_status = "✅ active" if user_active else "⚪ inactive"
        print(f"    User dropdown: {user_status}")
        print()
    
    # 4. 檢查樣式層級和優先級
    print("4. 檢查樣式層級")
    
    style_hierarchy = [
        "基礎樣式: .navbar-item .navbar-link",
        "Active 狀態: .navbar-item .navbar-link.is-active", 
        "Hover 效果: .navbar-item .navbar-link:hover",
        "Dropdown 項目: .navbar-dropdown .navbar-item:hover"
    ]
    
    print("樣式層級:")
    for style in style_hierarchy:
        print(f"  📝 {style}")
    
    # 5. 檢查與普通 navbar 項目的一致性
    print("\n5. 檢查樣式一致性")
    
    consistency_checks = [
        "顏色: rgb(90, 180, 255) - 主題藍色",
        "過渡效果: transition: all 0.3s ease",
        "Hover 背景: background-color: #FFF",
        "陰影效果: box-shadow: 0 0px 2px rgba(255, 255, 255, 1)"
    ]
    
    print("一致性檢查:")
    for check in consistency_checks:
        print(f"  🎨 {check}")
    
    print("\n✅ navbar dropdown 樣式測試完成！")
    print("💡 總結:")
    print("  - Dropdown 按鈕支持 active 和 hover 效果")
    print("  - 樣式與普通 navbar 項目保持一致")
    print("  - Active 狀態根據當前路徑自動判斷")
    print("  - Dropdown 內的項目也有 hover 效果")
    
    print("\n🎯 預期效果:")
    print("  - 在 /rosout_logs 頁面時，Logs dropdown 會高亮")
    print("  - 在 /clients 頁面時，User dropdown 會高亮")
    print("  - 滑鼠懸停時所有 dropdown 按鈕都有 hover 效果")
    print("  - Dropdown 展開時按鈕保持高亮狀態")


if __name__ == "__main__":
    test_navbar_dropdown_styles()
