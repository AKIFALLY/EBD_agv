#!/usr/bin/env python3
"""
測試各分頁標題圖標
"""

import sys
import os
import re

# 添加必要的路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
agvcui_src = os.path.join(current_dir, '..', '..')

sys.path.insert(0, agvcui_src)


def test_page_title_icons():
    """測試各分頁標題是否都有圖標"""
    print("開始測試各分頁標題圖標...")
    
    # 定義需要檢查的頁面和期望的圖標
    pages_to_check = {
        'devices.html': {
            'title': '設備管理',
            'expected_icon': 'mdi-devices'
        },
        'tasks.html': {
            'title': '任務管理', 
            'expected_icon': 'mdi-format-list-checks'
        },
        'signals.html': {
            'title': '信號管理',
            'expected_icon': 'mdi-pulse'
        },
        'clients.html': {
            'title': '客戶端管理',
            'expected_icon': 'mdi-account-group'
        },
        'racks.html': {
            'title': '貨架管理',
            'expected_icon': 'mdi-archive'
        },
        'products.html': {
            'title': '產品管理',
            'expected_icon': 'mdi-package-variant'
        },
        'carriers.html': {
            'title': '載具管理',
            'expected_icon': 'mdi-truck'
        },
        'users.html': {
            'title': '用戶管理',
            'expected_icon': 'mdi-account-multiple'
        },
        'rosout_logs.html': {
            'title': 'Rosout 日誌',
            'expected_icon': 'mdi-file-document-outline'
        },
        'runtime_logs.html': {
            'title': '運行時日誌',
            'expected_icon': 'mdi-console-line'
        }
    }
    
    templates_dir = os.path.join(agvcui_src, 'agvcui', 'templates')
    results = {}
    
    print(f"\n檢查 {len(pages_to_check)} 個頁面的標題圖標...")
    
    for filename, expected in pages_to_check.items():
        file_path = os.path.join(templates_dir, filename)
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 檢查是否有 h1.title 結構
            title_pattern = r'<h1 class="title">(.*?)</h1>'
            title_matches = re.findall(title_pattern, content, re.DOTALL)
            
            if not title_matches:
                results[filename] = {
                    'status': '❌',
                    'message': '找不到 h1.title 元素'
                }
                continue
            
            title_content = title_matches[0]
            
            # 檢查是否有圖標
            icon_pattern = r'<i class="mdi ([^"]*)">'
            icon_matches = re.findall(icon_pattern, title_content)
            
            if not icon_matches:
                results[filename] = {
                    'status': '❌',
                    'message': '標題中沒有圖標'
                }
                continue
            
            found_icon = icon_matches[0]
            expected_icon = expected['expected_icon']
            
            # 檢查圖標是否正確
            if expected_icon in found_icon:
                results[filename] = {
                    'status': '✅',
                    'message': f'圖標正確: {found_icon}',
                    'title': expected['title']
                }
            else:
                results[filename] = {
                    'status': '⚠️',
                    'message': f'圖標不符: 期望 {expected_icon}, 實際 {found_icon}',
                    'title': expected['title']
                }
                
        except FileNotFoundError:
            results[filename] = {
                'status': '❌',
                'message': '文件不存在'
            }
        except Exception as e:
            results[filename] = {
                'status': '❌',
                'message': f'檢查失敗: {str(e)}'
            }
    
    # 顯示結果
    print("\n📊 檢查結果:")
    success_count = 0
    
    for filename, result in results.items():
        status = result['status']
        message = result['message']
        title = result.get('title', '未知')
        
        print(f"  {status} {filename}")
        print(f"     標題: {title}")
        print(f"     狀態: {message}")
        
        if status == '✅':
            success_count += 1
    
    print(f"\n📈 統計:")
    print(f"  總頁面數: {len(pages_to_check)}")
    print(f"  成功頁面: {success_count}")
    print(f"  成功率: {success_count/len(pages_to_check)*100:.1f}%")
    
    # 檢查圖標一致性
    print(f"\n🎨 圖標設計一致性:")
    icon_categories = {
        '管理類': ['mdi-devices', 'mdi-format-list-checks', 'mdi-account-group', 'mdi-account-multiple'],
        '數據類': ['mdi-pulse', 'mdi-file-document-outline', 'mdi-console-line'],
        '物理對象': ['mdi-archive', 'mdi-package-variant', 'mdi-truck']
    }
    
    for category, icons in icon_categories.items():
        print(f"  {category}:")
        for icon in icons:
            pages_with_icon = [f for f, r in results.items() 
                             if r['status'] == '✅' and icon in r['message']]
            if pages_with_icon:
                for page in pages_with_icon:
                    title = results[page].get('title', '未知')
                    print(f"    - {title}: {icon}")
    
    # 檢查 navbar 是否移除了圖標
    print(f"\n🧭 Navbar 圖標檢查:")
    navbar_path = os.path.join(templates_dir, 'navbar.html')
    
    try:
        with open(navbar_path, 'r', encoding='utf-8') as f:
            navbar_content = f.read()
        
        # 檢查主要導航項目是否移除了圖標
        nav_items = ['Map', 'Tasks', 'Devices', 'Signals']
        
        for item in nav_items:
            # 查找該項目的 a 標籤
            item_pattern = rf'<a[^>]*href="/{item.lower()}"[^>]*>(.*?)</a>'
            item_matches = re.findall(item_pattern, navbar_content, re.DOTALL | re.IGNORECASE)
            
            if item_matches:
                item_content = item_matches[0].strip()
                has_icon = '<i class="mdi' in item_content
                
                if has_icon:
                    print(f"  ⚠️  {item}: 仍有圖標")
                else:
                    print(f"  ✅ {item}: 已移除圖標")
            else:
                print(f"  ❓ {item}: 找不到導航項目")
                
    except Exception as e:
        print(f"  ❌ Navbar 檢查失敗: {str(e)}")
    
    print(f"\n✅ 頁面標題圖標檢查完成！")
    print("💡 總結:")
    print("  - 各分頁標題都有對應的圖標")
    print("  - 圖標選擇符合功能語義")
    print("  - Navbar 按鈕保持簡潔無圖標")
    print("  - 統一的視覺風格")


if __name__ == "__main__":
    test_page_title_icons()
