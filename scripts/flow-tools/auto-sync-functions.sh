#!/bin/bash
# Flow Functions 自動同步工具
# 當您更新 flow_executor.py 添加新的 @flow_function 後，執行此腳本自動同步所有相關檔案

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROSAGV_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 顯示標題
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}   Flow Functions 自動同步工具${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

# 步驟 1: 重啟 Flow WCS 服務以載入新的裝飾器
echo -e "${BLUE}📦 步驟 1: 重啟 Flow WCS 服務${NC}"
echo -e "   重新載入 @flow_function 裝飾器..."

# 檢查 Flow WCS 是否運行中
if pgrep -f "flow_executor" > /dev/null; then
    echo -e "${YELLOW}   正在重啟 Flow WCS...${NC}"
    pkill -f "flow_executor" || true
    sleep 2
fi

# 在背景啟動 Flow WCS (如果在容器內)
if [ -f "/app/flow_wcs_ws/src/flow_wcs/flow_wcs/flow_executor.py" ]; then
    echo -e "   在容器內啟動 Flow WCS..."
    cd /app/flow_wcs_ws
    python3 -m flow_wcs.flow_executor &
    FLOW_PID=$!
    sleep 3
    echo -e "${GREEN}   ✅ Flow WCS 已啟動 (PID: $FLOW_PID)${NC}"
else
    echo -e "${YELLOW}   ⚠️  請確保 Flow WCS 正在運行${NC}"
fi

echo ""

# 步驟 2: 觸發 API 生成新的快取
echo -e "${BLUE}📦 步驟 2: 從 API 生成新的快取${NC}"

# 不需要刪除舊快取，直接覆蓋即可
# rm -f $ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml
# echo -e "   已清除舊快取"

# 呼叫 Linear Flow Designer API 觸發快取生成
echo -e "   正在從 API 獲取最新函數列表..."
if curl -s http://localhost:8001/linear-flow/api/functions > /dev/null 2>&1; then
    echo -e "${GREEN}   ✅ 已觸發快取生成${NC}"
elif curl -s http://localhost:8000/api/flow/functions > /dev/null 2>&1; then
    # 直接從 Flow WCS API 生成快取
    curl -s http://localhost:8000/api/flow/functions | python3 -c "
import sys
import json
import yaml
from datetime import datetime

try:
    data = json.load(sys.stdin)
    if data.get('success'):
        output = {
            'meta': {
                'version': '2.0.0',
                'system': 'flow_wcs',
                'updated_at': datetime.now().isoformat(),
                'source': 'flow_wcs_api'
            },
            'functions': data.get('functions', {})
        }
        with open('$ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml', 'w') as f:
            yaml.dump(output, f, allow_unicode=True, sort_keys=False)
        print('   ✅ 快取已從 API 生成')
except Exception as e:
    print(f'   ❌ 錯誤: {e}')
"
fi

# 檢查快取是否生成成功
if [ -f "$ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml" ]; then
    echo -e "${GREEN}   ✅ 快取檔案已生成${NC}"
    
    # 顯示新函數統計
    python3 -c "
import yaml
with open('$ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml', 'r') as f:
    data = yaml.safe_load(f)
    functions = data.get('functions', {})
    total = 0
    for category, funcs in functions.items():
        total += len(funcs)
    print(f'   📊 共載入 {total} 個函數')
"
else
    echo -e "${RED}   ❌ 快取生成失敗${NC}"
    exit 1
fi

echo ""

# 步驟 3: 更新手動維護的 YAML 檔案
echo -e "${BLUE}📦 步驟 3: 更新手動維護的 flow_functions.yaml${NC}"

TARGET_FILE="$ROSAGV_DIR/app/config/wcs/flow_functions.yaml"
BACKUP_DIR="$ROSAGV_DIR/app/config/wcs/backups"

# 建立備份
if [ -f "$TARGET_FILE" ]; then
    mkdir -p "$BACKUP_DIR"
    BACKUP_FILE="$BACKUP_DIR/flow_functions_$(date +%Y%m%d_%H%M%S).yaml"
    cp "$TARGET_FILE" "$BACKUP_FILE"
    echo -e "   已備份到: $BACKUP_FILE"
fi

# 複製快取到目標
cp $ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml "$TARGET_FILE"

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
data['meta']['description'] = 'Flow WCS 函數庫定義 (自動更新)'
data['meta']['updated'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
data['meta']['source'] = 'Auto-synced from @flow_function decorators'

# 寫回檔案
with open('$TARGET_FILE', 'w', encoding='utf-8') as f:
    yaml.dump(data, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
EOF

echo -e "${GREEN}   ✅ flow_functions.yaml 已更新${NC}"

echo ""

# 步驟 4: 更新 Linear Flow Designer 內嵌靜態函數
echo -e "${BLUE}📦 步驟 4: 更新 Linear Flow Designer 內嵌函數${NC}"

# 執行 sync-static-fallback.sh
if [ -f "$ROSAGV_DIR/scripts/sync-static-fallback.sh" ]; then
    echo -e "   正在更新內嵌靜態函數..."
    "$ROSAGV_DIR/scripts/sync-static-fallback.sh" sync
    echo -e "${GREEN}   ✅ 內嵌函數已更新${NC}"
else
    echo -e "${YELLOW}   ⚠️  sync-static-fallback.sh 不存在，跳過此步驟${NC}"
fi

echo ""

# 步驟 5: 顯示更新結果
echo -e "${BLUE}📊 更新結果摘要${NC}"
echo -e "${BLUE}────────────────────────────${NC}"

# 統計函數數量
python3 << EOF
import yaml

def count_functions(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            functions = data.get('functions', {})
            counts = {}
            total = 0
            for category, funcs in functions.items():
                counts[category] = len(funcs)
                total += len(funcs)
            return counts, total
    except:
        return {}, 0

# 統計各檔案
cache_counts, cache_total = count_functions('$ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml')
yaml_counts, yaml_total = count_functions('$TARGET_FILE')

print("📦 更新完成統計:")
print(f"  ✅ 動態快取: {cache_total} 個函數")
print(f"  ✅ 手動維護: {yaml_total} 個函數")
print(f"  ✅ 內嵌函數: 已同步更新")
print("")
print("📂 函數分類:")
for category in sorted(set(cache_counts.keys()) | set(yaml_counts.keys())):
    count = cache_counts.get(category, 0)
    if count > 0:
        print(f"  • {category}: {count} 個函數")
EOF

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}✨ 所有檔案已自動同步完成！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${CYAN}提示: 您現在可以使用更新後的函數了${NC}"
echo -e "${CYAN}  • Linear Flow Designer: http://localhost:8001/linear-flow${NC}"
echo -e "${CYAN}  • 參考文檔: /app/config/wcs/flow_functions.yaml${NC}"

# 清理背景進程
if [ ! -z "$FLOW_PID" ]; then
    echo ""
    echo -e "${YELLOW}清理: 停止測試用的 Flow WCS 進程...${NC}"
    kill $FLOW_PID 2>/dev/null || true
fi