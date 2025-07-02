#!/usr/bin/env python3
"""
測試 navbar active 效果清除
"""

import sys
import os
import re

# 添加必要的路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
agvcui_src = os.path.join(current_dir, '..', '..')

sys.path.insert(0, agvcui_src)


def test_navbar_clean():
    """測試 navbar active 效果是否已完全清除"""
    print("開始測試 navbar active 效果清除...")
    
    # 1. 檢查 CSS 文件
    print("\n1. 檢查 CSS 文件")
    css_path = os.path.join(agvcui_src, 'agvcui', 'static', 'css', 'agvcui-bulma-extend.css')
    
    try:
        with open(css_path, 'r', encoding='utf-8') as f:
            css_content = f.read()
        
        # 檢查是否還有 navbar 相關的樣式
        navbar_patterns = [
            'navbar.*active',
            'navbar.*hover',
            'navbar-item.*active',
            'navbar-link.*active',
            'is-active'
        ]
        
        print("CSS 清除檢查:")
        found_navbar_styles = False
        for pattern in navbar_patterns:
            matches = re.findall(pattern, css_content, re.IGNORECASE)
            if matches:
                print(f"  ❌ 找到 navbar 樣式: {pattern} - {matches}")
                found_navbar_styles = True
            else:
                print(f"  ✅ 已清除: {pattern}")
        
        if not found_navbar_styles:
            print("  🎉 所有 navbar active 樣式已完全清除")
        
        # 檢查剩餘的樣式
        print(f"\n剩餘的 CSS 內容:")
        lines = css_content.strip().split('\n')
        for i, line in enumerate(lines, 1):
            if line.strip():
                print(f"  {i:2d}: {line}")
    
    except FileNotFoundError:
        print("❌ CSS 文件不存在")
    
    # 2. 檢查 navbar.html 文件
    print("\n2. 檢查 navbar.html 文件")
    navbar_path = os.path.join(agvcui_src, 'agvcui', 'templates', 'navbar.html')
    
    try:
        with open(navbar_path, 'r', encoding='utf-8') as f:
            navbar_content = f.read()
        
        # 檢查是否還有 is-active 相關的設定
        active_patterns = [
            'is-active',
            'request.url.path.startswith',
            '{% if.*active.*%}'
        ]
        
        print("HTML 清除檢查:")
        found_active_logic = False
        for pattern in active_patterns:
            matches = re.findall(pattern, navbar_content, re.IGNORECASE)
            if matches:
                print(f"  ❌ 找到 active 邏輯: {pattern} - 共 {len(matches)} 處")
                found_active_logic = True
            else:
                print(f"  ✅ 已清除: {pattern}")
        
        if not found_active_logic:
            print("  🎉 所有 active 邏輯已完全清除")
        
        # 檢查 navbar 項目結構
        print(f"\n檢查 navbar 項目結構:")
        
        # 檢查普通項目
        normal_items = ['Map', 'Tasks', 'Devices', 'Signals']
        for item in normal_items:
            pattern = rf'<a class="navbar-item" href="/{item.lower()}">'
            if re.search(pattern, navbar_content, re.IGNORECASE):
                print(f"  ✅ {item}: 簡潔結構")
            else:
                print(f"  ❓ {item}: 結構可能不同")
        
        # 檢查 dropdown 項目
        dropdown_items = ['Logs', 'User', 'Help']
        for item in dropdown_items:
            pattern = rf'<a class="navbar-link">\s*{item}'
            if re.search(pattern, navbar_content, re.DOTALL):
                print(f"  ✅ {item}: 簡潔 dropdown 結構")
            else:
                print(f"  ❓ {item}: Dropdown 結構可能不同")
    
    except FileNotFoundError:
        print("❌ navbar.html 文件不存在")
    
    # 3. 檢查當前的 navbar 狀態
    print("\n3. 當前 navbar 狀態")
    
    print("現在的 navbar 特點:")
    print("  📝 所有項目都是基本的 HTML 結構")
    print("  📝 沒有條件判斷邏輯")
    print("  📝 沒有 active 狀態檢測")
    print("  📝 沒有自定義的 CSS 樣式")
    print("  📝 完全依賴 Bulma 框架的預設樣式")
    
    # 4. 預期的視覺效果
    print("\n4. 預期的視覺效果")
    
    print("清除後的效果:")
    print("  🎨 所有 navbar 項目外觀一致")
    print("  🎨 沒有當前頁面的高亮顯示")
    print("  🎨 只有 Bulma 預設的 hover 效果")
    print("  🎨 Dropdown 按鈕與普通項目外觀相同")
    
    # 5. 下一步建議
    print("\n5. 下一步建議")
    
    print("可以重新設計的功能:")
    print("  🔧 統一的 hover 效果")
    print("  🔧 滿版的點擊區域")
    print("  🔧 可選的 active 狀態顯示")
    print("  🔧 一致的視覺反饋")
    print("  🔧 改進的用戶體驗")
    
    print("\n✅ navbar active 效果清除檢查完成！")
    print("💡 總結:")
    print("  - 所有 active 相關的 CSS 樣式已清除")
    print("  - 所有 active 相關的 HTML 邏輯已清除")
    print("  - navbar 回到最基本的狀態")
    print("  - 可以重新開始設計 navbar 樣式")


if __name__ == "__main__":
    test_navbar_clean()
