#!/bin/bash
# Flow Functions 更新工具
# 從動態生成的快取更新手動維護的 flow_functions.yaml
# 
# 與 sync-static-fallback.sh 的區別：
# - sync-static-fallback.sh: 更新 Linear Flow Designer 內嵌的靜態函數
# - update-flow-functions.sh: 更新 /app/config/wcs/flow_functions.yaml 參考文件

set -e

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 檔案路徑
CACHE_FILE="$ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml"
TARGET_FILE="/home/ct/RosAGV/app/config/wcs/flow_functions.yaml"
BACKUP_DIR="/home/ct/RosAGV/app/config/wcs/backups"

# 顯示標題
echo -e "${BLUE}=====================================${NC}"
echo -e "${BLUE}   Flow Functions 更新工具${NC}"
echo -e "${BLUE}=====================================${NC}"
echo ""

# 檢查快取檔案是否存在
if [ ! -f "$CACHE_FILE" ]; then
    echo -e "${YELLOW}⚠️  快取檔案不存在: $CACHE_FILE${NC}"
    echo -e "${YELLOW}   請先啟動 Linear Flow Designer 或執行 Flow WCS 來生成快取${NC}"
    
    # 嘗試觸發快取生成
    echo ""
    echo -e "${BLUE}嘗試從 Flow WCS API 獲取函數列表...${NC}"
    
    # 檢查 API 是否可用
    if curl -s -o /dev/null -w "%{http_code}" http://localhost:8000/api/flow/functions | grep -q "200"; then
        echo -e "${GREEN}✅ API 可用，正在獲取函數列表...${NC}"
        
        # 獲取函數列表並保存到快取
        curl -s http://localhost:8000/api/flow/functions | python3 -c "
import sys
import json
import yaml
from datetime import datetime

data = json.load(sys.stdin)
if data.get('success'):
    # 轉換為 YAML 格式
    output = {
        'meta': {
            'version': '2.0.0',
            'system': 'flow_wcs',
            'description': 'Flow WCS 函數庫定義',
            'updated_at': datetime.now().isoformat(),
            'source': 'flow_wcs_api'
        },
        'functions': data.get('functions', {})
    }
    
    # 保存到快取檔案
    with open('$CACHE_FILE', 'w', encoding='utf-8') as f:
        yaml.dump(output, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
    
    print('✅ 快取檔案已生成')
else:
    print('❌ API 返回錯誤')
    sys.exit(1)
"
        if [ $? -ne 0 ]; then
            echo -e "${RED}❌ 無法生成快取檔案${NC}"
            exit 1
        fi
    else
        echo -e "${RED}❌ Flow WCS API 不可用${NC}"
        echo -e "${YELLOW}   請確保 AGVC 系統正在運行${NC}"
        exit 1
    fi
fi

# 顯示快取檔案資訊
echo -e "${BLUE}📦 快取檔案資訊:${NC}"
if [ -f "$CACHE_FILE" ]; then
    CACHE_TIME=$(stat -c %y "$CACHE_FILE" | cut -d' ' -f1,2 | cut -d'.' -f1)
    CACHE_SIZE=$(stat -c %s "$CACHE_FILE")
    echo -e "   位置: $CACHE_FILE"
    echo -e "   更新時間: $CACHE_TIME"
    echo -e "   大小: $CACHE_SIZE bytes"
    
    # 顯示函數統計
    echo ""
    echo -e "${BLUE}📊 函數統計:${NC}"
    python3 -c "
import yaml
with open('$CACHE_FILE', 'r') as f:
    data = yaml.safe_load(f)
    functions = data.get('functions', {})
    total = 0
    for category, funcs in functions.items():
        count = len(funcs)
        total += count
        print(f'   {category}: {count} 個函數')
    print(f'   總計: {total} 個函數')
"
fi

# 檢查目標檔案
echo ""
echo -e "${BLUE}📄 目標檔案資訊:${NC}"
if [ -f "$TARGET_FILE" ]; then
    TARGET_TIME=$(stat -c %y "$TARGET_FILE" | cut -d' ' -f1,2 | cut -d'.' -f1)
    TARGET_SIZE=$(stat -c %s "$TARGET_FILE")
    echo -e "   位置: $TARGET_FILE"
    echo -e "   更新時間: $TARGET_TIME"
    echo -e "   大小: $TARGET_SIZE bytes"
else
    echo -e "${YELLOW}   目標檔案不存在，將創建新檔案${NC}"
fi

# 比較差異
if [ -f "$TARGET_FILE" ]; then
    echo ""
    echo -e "${BLUE}🔍 比較差異:${NC}"
    
    # 使用 Python 比較函數差異
    python3 << EOF
import yaml
import sys

def load_functions(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            return data.get('functions', {})
    except:
        return {}

cache_funcs = load_functions('$CACHE_FILE')
target_funcs = load_functions('$TARGET_FILE')

# 統計差異
cache_set = set()
target_set = set()

for cat, funcs in cache_funcs.items():
    for func in funcs:
        cache_set.add(func.get('name', ''))

for cat, funcs in target_funcs.items():
    for func in funcs:
        target_set.add(func.get('name', ''))

new_funcs = cache_set - target_set
removed_funcs = target_set - cache_set
common_funcs = cache_set & target_set

if new_funcs:
    print(f"   ✨ 新增函數: {len(new_funcs)} 個")
    for func in sorted(new_funcs)[:5]:
        print(f"      + {func}")
    if len(new_funcs) > 5:
        print(f"      ... 還有 {len(new_funcs)-5} 個")

if removed_funcs:
    print(f"   ⚠️  移除函數: {len(removed_funcs)} 個")
    for func in sorted(removed_funcs)[:5]:
        print(f"      - {func}")
    if len(removed_funcs) > 5:
        print(f"      ... 還有 {len(removed_funcs)-5} 個")

if common_funcs:
    print(f"   ✅ 保持不變: {len(common_funcs)} 個函數")

if not new_funcs and not removed_funcs:
    print("   ✅ 沒有差異，檔案已經是最新的")
EOF
fi

# 詢問是否更新
echo ""
echo -e "${YELLOW}❓ 是否要用快取更新手動維護的檔案？${NC}"
echo -e "   這將會："
echo -e "   1. 備份當前的 flow_functions.yaml"
echo -e "   2. 用快取內容替換手動維護的版本"
echo -e "   3. 保留快取中的所有函數定義"
echo ""
read -p "確認更新？(y/N): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    # 創建備份目錄
    mkdir -p "$BACKUP_DIR"
    
    # 備份當前檔案
    if [ -f "$TARGET_FILE" ]; then
        BACKUP_FILE="$BACKUP_DIR/flow_functions_$(date +%Y%m%d_%H%M%S).yaml"
        cp "$TARGET_FILE" "$BACKUP_FILE"
        echo -e "${GREEN}✅ 已備份到: $BACKUP_FILE${NC}"
    fi
    
    # 複製快取到目標
    cp "$CACHE_FILE" "$TARGET_FILE"
    
    # 更新 meta 資訊
    python3 << EOF
import yaml
from datetime import datetime

with open('$TARGET_FILE', 'r', encoding='utf-8') as f:
    data = yaml.safe_load(f)

# 更新 meta
data['meta'] = data.get('meta', {})
data['meta']['version'] = '2.0.0'
data['meta']['system'] = 'flow_wcs'
data['meta']['description'] = 'Flow WCS 函數庫定義'
data['meta']['updated'] = datetime.now().strftime('%Y-%m-%d')
data['meta']['source'] = 'Updated from cache by update-flow-functions.sh'

# 添加驗證規則（如果不存在）
if 'validation' not in data:
    data['validation'] = {
        'required_sections': ['meta', 'flow', 'workflow'],
        'flow_fields': ['id', 'name'],
        'step_fields': {
            'required': ['id', 'exec'],
            'optional': ['params', 'store', 'skip_if', 'skip_if_not']
        }
    }

# 寫回檔案
with open('$TARGET_FILE', 'w', encoding='utf-8') as f:
    yaml.dump(data, f, allow_unicode=True, sort_keys=False, default_flow_style=False)

print("✅ Meta 資訊已更新")
EOF
    
    echo -e "${GREEN}✅ 更新完成！${NC}"
    echo -e "${GREEN}   flow_functions.yaml 已更新為最新版本${NC}"
    
    # 顯示更新後的統計
    echo ""
    echo -e "${BLUE}📊 更新後的函數統計:${NC}"
    python3 -c "
import yaml
with open('$TARGET_FILE', 'r') as f:
    data = yaml.safe_load(f)
    functions = data.get('functions', {})
    total = 0
    for category, funcs in functions.items():
        count = len(funcs)
        total += count
        print(f'   {category}: {count} 個函數')
    print(f'   總計: {total} 個函數')
"
else
    echo -e "${YELLOW}⚠️  取消更新${NC}"
    exit 0
fi

echo ""
echo -e "${BLUE}=====================================${NC}"
echo -e "${GREEN}✨ 完成！${NC}"
echo -e "${BLUE}=====================================${NC}"