#!/usr/bin/env python3
"""
驗證所有轉換後的 TAFL 檔案
"""

import sys
import os
import yaml

# 添加 tafl 模組路徑
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src', 'tafl'))

from tafl.parser import TAFLParser
from tafl.validator import TAFLValidator

def validate_tafl_files():
    """驗證所有 TAFL 檔案"""

    # 初始化解析器和驗證器
    parser = TAFLParser()
    validator = TAFLValidator()

    # 獲取所有 TAFL 檔案
    config_dir = "/home/ct/EBD_agv/app/config/tafl"
    if not os.path.exists(config_dir):
        print(f"❌ 找不到目錄: {config_dir}")
        return

    # 搜尋所有子目錄中的 TAFL 檔案
    tafl_files = []
    for root, dirs, files in os.walk(config_dir):
        for f in files:
            if f.endswith('.yaml') or f.endswith('.tafl'):
                tafl_files.append(os.path.join(root, f))
    
    print('=' * 60)
    print('🔍 TAFL 檔案驗證測試')
    print('=' * 60)
    
    # 統計
    total_files = len(tafl_files)
    passed_files = 0
    failed_files = 0
    errors_detail = []
    
    # 測試每個檔案
    for i, filepath in enumerate(sorted(tafl_files), 1):
        filename = os.path.basename(filepath)
        print(f'\n[{i}/{total_files}] 測試: {filename}')
        print('-' * 40)
        
        try:
            # 讀取檔案
            with open(filepath, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 解析 TAFL
            ast = parser.parse_string(content)
            print('  ✅ 解析成功')
            
            # 驗證 TAFL
            validator.errors = []  # 重置錯誤列表
            if validator.validate(ast):
                print('  ✅ 驗證通過')
                print(f'  📊 流程步驟數: {len(ast.flow)}')
                
                # 統計動詞使用
                verbs = {}
                for node in ast.flow:
                    # 跳過純 comment 節點
                    if hasattr(node, '__class__'):
                        verb = node.__class__.__name__.replace('Node', '').lower()
                        verbs[verb] = verbs.get(verb, 0) + 1
                
                if verbs:
                    verb_list = sorted(verbs.items(), key=lambda x: x[1], reverse=True)
                    print('  📈 動詞統計:', ', '.join([f'{v}({c})' for v, c in verb_list[:5]]))
                
                passed_files += 1
            else:
                errors = validator.get_errors()
                print('  ❌ 驗證失敗')
                print(f'  錯誤: {errors}')
                failed_files += 1
                errors_detail.append((filename, errors))
                
        except Exception as e:
            print(f'  ❌ 解析錯誤: {e}')
            failed_files += 1
            errors_detail.append((filename, str(e)))
    
    # 總結報告
    print('\n' + '=' * 60)
    print('📊 驗證測試總結')
    print('=' * 60)
    print(f'總檔案數: {total_files}')
    print(f'✅ 通過: {passed_files} ({passed_files/total_files*100:.1f}%)')
    print(f'❌ 失敗: {failed_files} ({failed_files/total_files*100:.1f}%)')
    
    if errors_detail:
        print('\n⚠️ 錯誤詳情:')
        for filename, error in errors_detail:
            print(f'  - {filename}: {error}')
    
    # 結論
    print('\n' + '=' * 60)
    if failed_files == 0:
        print('🎉 所有 TAFL 檔案驗證通過！')
        print('✨ 轉換品質優秀，可以安全使用。')
    else:
        print(f'⚠️ 有 {failed_files} 個檔案需要修正')
    print('=' * 60)
    
    return passed_files, failed_files

if __name__ == '__main__':
    passed, failed = validate_tafl_files()
    sys.exit(0 if failed == 0 else 1)