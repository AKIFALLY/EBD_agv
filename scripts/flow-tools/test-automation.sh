#!/bin/bash
# 測試 Flow Functions 自動化工具

echo "======================================"
echo "   Flow Functions 自動化測試"
echo "======================================"
echo ""

# 顏色定義
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 測試結果計數
PASS=0
FAIL=0

# 測試函數
test_feature() {
    local name="$1"
    local command="$2"
    
    echo -n "測試: $name ... "
    
    if $command > /dev/null 2>&1; then
        echo -e "${GREEN}✓ 通過${NC}"
        ((PASS++))
    else
        echo -e "${RED}✗ 失敗${NC}"
        ((FAIL++))
    fi
}

# 開始測試
echo "📋 執行自動化測試..."
echo ""

# 1. 檢查腳本是否存在且可執行
test_feature "dev-flow-functions.sh 存在" "[ -x scripts/flow-tools/dev-flow-functions.sh ]"
test_feature "auto-sync-functions.sh 存在" "[ -x scripts/flow-tools/auto-sync-functions.sh ]"
test_feature "flow-functions-manager.sh 存在" "[ -x scripts/flow-tools/flow-functions-manager.sh ]"
test_feature "update-flow-functions.sh 存在" "[ -x scripts/flow-tools/update-flow-functions.sh ]"

# 2. 檢查關鍵檔案
test_feature "flow_executor.py 存在" "[ -f app/flow_wcs_ws/src/flow_wcs/flow_wcs/flow_executor.py ]"
test_feature "flow_functions.yaml 存在" "[ -f app/config/wcs/flow_functions.yaml ]"
test_feature "linear_flow_designer.py 存在" "[ -f app/web_api_ws/src/agvcui/agvcui/routers/linear_flow_designer.py ]"

# 3. 測試語法驗證
test_feature "Python 語法驗證" "./scripts/flow-tools/dev-flow-functions.sh validate"

# 4. 檢查 @flow_function 數量
echo ""
echo "📊 @flow_function 統計:"
DECORATOR_COUNT=$(grep -c "@flow_function" app/flow_wcs_ws/src/flow_wcs/flow_wcs/flow_executor.py)
echo "   發現 $DECORATOR_COUNT 個裝飾器"

# 5. 檢查 Git Hook
test_feature "Git Hook 已安裝" "[ -x .git/hooks/pre-commit ]"

# 6. 測試工具幫助
test_feature "dev-flow-functions.sh help" "./scripts/flow-tools/dev-flow-functions.sh help"
test_feature "flow-functions-manager.sh help" "./scripts/flow-tools/flow-functions-manager.sh help"

# 總結
echo ""
echo "======================================"
echo "測試結果："
echo -e "  ${GREEN}通過: $PASS${NC}"
echo -e "  ${RED}失敗: $FAIL${NC}"

if [ $FAIL -eq 0 ]; then
    echo ""
    echo -e "${GREEN}✨ 所有測試通過！自動化工具準備就緒。${NC}"
    echo ""
    echo "使用方法："
    echo "  1. 編輯: ./scripts/flow-tools/dev-flow-functions.sh edit"
    echo "  2. 更新: ./scripts/flow-tools/dev-flow-functions.sh workflow"
    echo "  3. 查看: ./scripts/flow-tools/flow-functions-manager.sh status"
else
    echo ""
    echo -e "${YELLOW}⚠️  有 $FAIL 個測試失敗，請檢查問題。${NC}"
fi
echo "======================================"