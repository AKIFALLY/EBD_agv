#!/usr/bin/env python3
"""
AGVCUI Navbar 桌面專用設計測試（含 Bulma 框架覆寫）

這個測試檢查 AGVCUI 的導航欄是否正確實現了桌面專用設計：
1. HTML 結構包含必要的導航元素
2. JavaScript 功能正常運作
3. CSS 樣式包含必要的 Bulma 框架覆寫

重要說明：
- 主要目標：移除平板設備支援，專注於桌面環境
- 保留必要的媒體查詢：用於覆寫 Bulma 框架在 1023px 以下隱藏 navbar 的預設行為
- 支援桌面解析度：1366x768, 1920x1080 等標準桌面尺寸
- 區分概念：「Bulma 覆寫」≠「平板支援」

測試涵蓋：
- 漢堡選單按鈕的存在和功能
- 導航選單在桌面環境下的正常顯示
- Bulma 框架覆寫的正確實作
- 下拉選單在所有桌面尺寸下的正常顯示

執行方式：
python test_navbar_responsive.py
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
    """測試 CSS 樣式是否符合桌面專用設計（保留必要的 Bulma 覆寫）"""

    css_path = Path(__file__).parent.parent / "static" / "css" / "agvcui-bulma-extend.css"

    if not css_path.exists():
        print(f"❌ 找不到 agvcui-bulma-extend.css 文件: {css_path}")
        return False

    try:
        with open(css_path, 'r', encoding='utf-8') as f:
            content = f.read()

        print("\n🔍 檢查 CSS 樣式...")

        # 檢查必要的桌面樣式
        required_checks = [
            ('navbar-burger 樣式', r'\.navbar-burger'),
            ('下拉選單顯示控制', r'\.navbar-item\.has-dropdown\.is-active'),
            ('Bulma 覆寫媒體查詢', r'@media screen and \(max-width: 1023px\)'),
            ('navbar-menu 樣式', r'\.navbar-menu'),
            ('navbar-dropdown 樣式', r'\.navbar-dropdown'),
        ]

        # 檢查註釋是否說明了 Bulma 覆寫的目的
        comment_checks = [
            ('Bulma 覆寫說明註釋', r'覆寫.*bulma|bulma.*覆寫'),
        ]

        all_passed = True

        # 檢查必要樣式
        for name, pattern in required_checks:
            if re.search(pattern, content, re.IGNORECASE):
                print(f"  ✅ {name}: 找到")
            else:
                print(f"  ❌ {name}: 未找到")
                all_passed = False

        # 檢查註釋說明
        for name, pattern in comment_checks:
            if re.search(pattern, content, re.IGNORECASE):
                print(f"  ✅ {name}: 找到")
            else:
                print(f"  ⚠️ {name}: 建議添加說明註釋")
                # 註釋缺失不算測試失敗，只是建議

        # 驗證媒體查詢內容是否正確
        # 使用更精確的正則表達式來匹配整個媒體查詢區塊
        media_query_pattern = r'@media screen and \(max-width: 1023px\)\s*\{(.*?)\n\}'
        media_query_match = re.search(media_query_pattern, content, re.DOTALL)

        if media_query_match:
            media_content = media_query_match.group(1)

            # 檢查媒體查詢內是否包含必要的 navbar 樣式
            navbar_styles = [
                (r'\.navbar-menu\s*\{', '.navbar-menu 樣式規則'),
                (r'\.navbar-item\.has-dropdown\s+\.navbar-dropdown\s*\{', '.navbar-item.has-dropdown .navbar-dropdown 樣式規則'),
                (r'\.navbar-item\.has-dropdown\.is-active\s+\.navbar-dropdown\s*\{', '.navbar-item.has-dropdown.is-active .navbar-dropdown 樣式規則'),
                (r'\.navbar-dropdown\s*\{', '.navbar-dropdown 樣式規則'),
                (r'\.navbar-dropdown\s+\.navbar-item\s*\{', '.navbar-dropdown .navbar-item 樣式規則')
            ]

            print("  🔍 檢查媒體查詢內容:")
            for pattern, description in navbar_styles:
                if re.search(pattern, media_content):
                    print(f"    ✅ {description}: 存在於媒體查詢中")
                else:
                    print(f"    ❌ {description}: 媒體查詢中缺失")
                    all_passed = False

            # 額外檢查：確認媒體查詢包含關鍵的 Bulma 覆寫邏輯
            key_properties = [
                (r'display:\s*none', 'display: none (隱藏下拉選單)'),
                (r'display:\s*block', 'display: block (顯示活動下拉選單)'),
                (r'box-shadow:', 'box-shadow (選單陰影)')
            ]

            print("  🔍 檢查關鍵 CSS 屬性:")
            for pattern, description in key_properties:
                if re.search(pattern, media_content):
                    print(f"    ✅ {description}: 找到")
                else:
                    print(f"    ❌ {description}: 缺失")
                    all_passed = False
        else:
            print("  ❌ 無法解析媒體查詢內容")
            all_passed = False

        return all_passed

    except Exception as e:
        print(f"❌ 讀取文件時發生錯誤: {e}")
        return False

def main():
    """主測試函數"""
    print("🧪 開始測試 Navbar 桌面專用設計（含 Bulma 覆寫）...")
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
        print("🎉 所有測試通過！Navbar 桌面專用設計已正確實現。")
        print("\n✨ 功能說明:")
        print("  • 桌面環境優化：針對標準桌面解析度設計")
        print("  • Bulma 框架覆寫：防止較小桌面螢幕 navbar 被隱藏")
        print("  • 支援解析度：1366x768, 1920x1080 等桌面解析度")
        print("  • 下拉選單：在所有桌面尺寸下正常顯示")
        print("  • 移除平板支援：簡化維護，專注桌面體驗")
        return True
    else:
        failed_count = len([r for r in results if not r])
        print(f"❌ {failed_count} 個測試失敗，請檢查上述問題。")
        return False

if __name__ == "__main__":
    print("=" * 60)
    print("📝 重要說明：")
    print("此測試已更新以反映 AGVCUI 的桌面專用設計策略：")
    print("• 移除了平板設備支援以簡化維護")
    print("• 保留了必要的 Bulma 框架覆寫 (@media max-width: 1023px)")
    print("• 該媒體查詢用於防止較小桌面螢幕 (如 1366x768) 的 navbar 被隱藏")
    print("• 區分概念：'Bulma 覆寫' ≠ '平板支援'")
    print("=" * 60)
    print()

    success = main()
    sys.exit(0 if success else 1)
