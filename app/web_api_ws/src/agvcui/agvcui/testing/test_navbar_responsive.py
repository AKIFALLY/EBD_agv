#!/usr/bin/env python3
"""
測試 Navbar 響應式設計功能
檢查漢堡選單和下拉選單的 HTML 結構
"""

import os
import sys
import re
from pathlib import Path

def test_navbar_structure():
    """測試 navbar.html 的結構是否正確"""
    
    # 找到 navbar.html 文件
    navbar_path = Path(__file__).parent.parent / "templates" / "navbar.html"
    
    if not navbar_path.exists():
        print(f"❌ 找不到 navbar.html 文件: {navbar_path}")
        return False
    
    try:
        with open(navbar_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        print("🔍 檢查 Navbar 結構...")
        
        # 檢查必要的元素
        checks = [
            ('navbar 容器', r'<nav[^>]*class="navbar'),
            ('navbar-brand', r'<div[^>]*class="navbar-brand'),
            ('漢堡選單按鈕', r'<a[^>]*class="navbar-burger'),
            ('漢堡選單線條', r'<span aria-hidden="true"></span>'),
            ('navbar-menu', r'<div[^>]*id="navbarBasicExample"[^>]*class="navbar-menu'),
            ('navbar-start', r'<div[^>]*class="navbar-start'),
            ('navbar-end', r'<div[^>]*class="navbar-end'),
            ('下拉選單', r'<div[^>]*class="navbar-item has-dropdown'),
            ('用戶下拉選單', r'id="user-dropdown"'),
        ]
        
        all_passed = True
        for name, pattern in checks:
            if re.search(pattern, content, re.IGNORECASE):
                print(f"  ✅ {name}: 找到")
            else:
                print(f"  ❌ {name}: 未找到")
                all_passed = False
        
        # 檢查漢堡選單的 data-target 屬性
        burger_match = re.search(r'data-target="([^"]+)"', content)
        menu_id_match = re.search(r'<div[^>]*id="([^"]+)"[^>]*class="navbar-menu', content)
        
        if burger_match and menu_id_match:
            burger_target = burger_match.group(1)
            menu_id = menu_id_match.group(1)
            if burger_target == menu_id:
                print(f"  ✅ 漢堡選單目標匹配: {burger_target}")
            else:
                print(f"  ❌ 漢堡選單目標不匹配: {burger_target} != {menu_id}")
                all_passed = False
        else:
            print("  ❌ 無法找到漢堡選單目標或選單 ID")
            all_passed = False
        
        return all_passed
        
    except Exception as e:
        print(f"❌ 讀取文件時發生錯誤: {e}")
        return False

def test_navbar_js():
    """測試 navbar.js 是否包含必要的功能"""
    
    js_path = Path(__file__).parent.parent / "static" / "js" / "navbar.js"
    
    if not js_path.exists():
        print(f"❌ 找不到 navbar.js 文件: {js_path}")
        return False
    
    try:
        with open(js_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        print("\n🔍 檢查 Navbar JavaScript...")
        
        # 檢查必要的功能
        checks = [
            ('setupBurgerMenu 函數', r'function setupBurgerMenu\(\)'),
            ('漢堡選單事件監聽', r'\.addEventListener\([\'"]click[\'"]'),
            ('navbar-burger 選擇器', r'\.navbar-burger'),
            ('is-active 類別切換', r'\.classList\.toggle\([\'"]is-active[\'"]'),
            ('setupDropdowns 函數', r'function setupDropdowns\(\)'),
            ('setup 函數調用 setupBurgerMenu', r'setupBurgerMenu\(\)'),
        ]
        
        all_passed = True
        for name, pattern in checks:
            if re.search(pattern, content, re.IGNORECASE):
                print(f"  ✅ {name}: 找到")
            else:
                print(f"  ❌ {name}: 未找到")
                all_passed = False
        
        return all_passed
        
    except Exception as e:
        print(f"❌ 讀取文件時發生錯誤: {e}")
        return False

def test_css_styles():
    """測試 CSS 樣式是否包含響應式設計"""
    
    css_path = Path(__file__).parent.parent / "static" / "css" / "agvcui-bulma-extend.css"
    
    if not css_path.exists():
        print(f"❌ 找不到 agvcui-bulma-extend.css 文件: {css_path}")
        return False
    
    try:
        with open(css_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        print("\n🔍 檢查 CSS 樣式...")
        
        # 檢查必要的樣式
        checks = [
            ('navbar-burger 樣式', r'\.navbar-burger'),
            ('響應式媒體查詢', r'@media screen and \(max-width: 1023px\)'),
            ('下拉選單顯示控制', r'\.navbar-item\.has-dropdown\.is-active'),
        ]
        
        all_passed = True
        for name, pattern in checks:
            if re.search(pattern, content, re.IGNORECASE):
                print(f"  ✅ {name}: 找到")
            else:
                print(f"  ❌ {name}: 未找到")
                all_passed = False
        
        return all_passed
        
    except Exception as e:
        print(f"❌ 讀取文件時發生錯誤: {e}")
        return False

def main():
    """主測試函數"""
    print("🧪 開始測試 Navbar 響應式設計...")
    print("=" * 50)
    
    results = []
    
    # 測試 HTML 結構
    results.append(test_navbar_structure())
    
    # 測試 JavaScript 功能
    results.append(test_navbar_js())
    
    # 測試 CSS 樣式
    results.append(test_css_styles())
    
    print("\n" + "=" * 50)
    print("📊 測試結果總結:")
    
    if all(results):
        print("🎉 所有測試通過！Navbar 響應式設計已正確實現。")
        print("\n✨ 功能說明:")
        print("  • 在桌面螢幕上：顯示完整的水平導航選單")
        print("  • 在平板/手機螢幕上：顯示漢堡選單按鈕")
        print("  • 點擊漢堡選單：展開/收起導航選單")
        print("  • 下拉選單：在小螢幕上垂直展開")
        return True
    else:
        failed_count = len([r for r in results if not r])
        print(f"❌ {failed_count} 個測試失敗，請檢查上述問題。")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
