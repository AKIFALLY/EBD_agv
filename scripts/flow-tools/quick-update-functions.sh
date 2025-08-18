#!/bin/bash
# 快速更新 flow_functions.yaml - 超簡化版本

set -e

# 顏色定義
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# 檔案路徑（都在同一個目錄！）
WCS_DIR="/home/ct/RosAGV/app/config/wcs"
CACHE_FILE="$WCS_DIR/flow_functions_cache.yaml"
TARGET_FILE="$WCS_DIR/flow_functions.yaml"
BACKUP_FILE="$WCS_DIR/flow_functions.yaml.backup"

echo -e "${BLUE}🚀 快速更新 flow_functions.yaml${NC}"
echo "================================"

# 檢查快取檔案
if [ ! -f "$CACHE_FILE" ]; then
    echo -e "${RED}❌ 快取檔案不存在: $CACHE_FILE${NC}"
    echo "請先執行 Linear Flow Designer 或 flow_wcs API 來生成快取"
    exit 1
fi

# 備份原檔案
if [ -f "$TARGET_FILE" ]; then
    echo -e "${BLUE}📦 備份原檔案...${NC}"
    cp "$TARGET_FILE" "$BACKUP_FILE"
fi

# 直接複製（最簡單的方式！）
echo -e "${GREEN}✅ 更新檔案...${NC}"
cp "$CACHE_FILE" "$TARGET_FILE"

# 顯示結果
echo -e "${GREEN}✅ 更新完成！${NC}"
echo ""
echo "檔案資訊:"
echo "  來源: $CACHE_FILE"
echo "  目標: $TARGET_FILE"
echo "  備份: $BACKUP_FILE"

# 顯示函數統計
if command -v yq &> /dev/null; then
    echo ""
    echo -e "${BLUE}📊 函數統計:${NC}"
    
    # 分類統計
    TOTAL=0
    for category in $(yq '.functions | keys | .[]' "$TARGET_FILE" 2>/dev/null); do
        COUNT=$(yq ".functions.$category | length" "$TARGET_FILE" 2>/dev/null)
        echo "  $category: $COUNT 個函數"
        TOTAL=$((TOTAL + COUNT))
    done
    echo "  總計: $TOTAL 個函數"
fi

echo ""
echo -e "${GREEN}提示: 如果需要更複雜的合併邏輯，請使用 update-flow-functions.sh${NC}"