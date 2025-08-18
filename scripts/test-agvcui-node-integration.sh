#!/bin/bash
# 測試 AGVCUI 節點管理整合

echo "================================================"
echo "       AGVCUI 節點管理整合測試"
echo "================================================"

# 顏色定義
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 配置
AGVCUI_URL="http://localhost:8001"
API_URL="http://localhost:8000"

# 檢查函數
check_service() {
    local url=$1
    local service=$2
    
    if curl -s -o /dev/null -w "%{http_code}" "$url" | grep -q "200\|302"; then
        echo -e "${GREEN}✓${NC} $service 運行正常"
        return 0
    else
        echo -e "${RED}✗${NC} $service 無法訪問"
        return 1
    fi
}

# 1. 檢查 Web API 服務
echo -e "\n${YELLOW}1. 檢查 Web API 服務${NC}"
echo "-----------------------------------"
check_service "$API_URL/health" "Web API (Port 8000)"
check_service "$API_URL/api/nodes/health" "Node API Endpoint"

# 2. 檢查 AGVCUI 服務
echo -e "\n${YELLOW}2. 檢查 AGVCUI 服務${NC}"
echo "-----------------------------------"
check_service "$AGVCUI_URL/" "AGVCUI (Port 8001)"

# 3. 測試 AGVCUI 節點管理頁面
echo -e "\n${YELLOW}3. 測試 AGVCUI 節點管理頁面${NC}"
echo "-----------------------------------"

# 測試節點管理主頁面
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" "$AGVCUI_URL/nodes/")
if [ "$HTTP_CODE" = "200" ] || [ "$HTTP_CODE" = "302" ]; then
    echo -e "${GREEN}✓${NC} 節點管理頁面可訪問"
    
    # 測試頁面內容
    CONTENT=$(curl -s "$AGVCUI_URL/nodes/")
    if echo "$CONTENT" | grep -q "節點管理"; then
        echo -e "${GREEN}✓${NC} 頁面包含正確的標題"
    else
        echo -e "${YELLOW}⚠${NC} 頁面標題可能不正確"
    fi
else
    echo -e "${RED}✗${NC} 節點管理頁面無法訪問 (HTTP $HTTP_CODE)"
fi

# 4. 測試 AGVCUI API 代理
echo -e "\n${YELLOW}4. 測試 AGVCUI API 代理${NC}"
echo "-----------------------------------"

# 測試狀態 API
API_RESPONSE=$(curl -s "$AGVCUI_URL/nodes/api/status")
if [ -n "$API_RESPONSE" ]; then
    echo -e "${GREEN}✓${NC} API 代理運作正常"
    
    # 檢查回應格式
    if echo "$API_RESPONSE" | python3 -m json.tool > /dev/null 2>&1; then
        echo -e "${GREEN}✓${NC} API 回應格式正確 (JSON)"
        
        # 顯示部分內容
        echo -e "\n${YELLOW}節點狀態摘要:${NC}"
        echo "$API_RESPONSE" | python3 -c "
import json, sys
data = json.load(sys.stdin)
nodes = data.get('nodes', [])
agvs = data.get('agvs', [])
print(f'  - 本地節點數: {len(nodes)}')
print(f'  - 遠端 AGV 數: {len(agvs)}')
if nodes:
    print(f'  - 範例節點: {nodes[0].get(\"name\", \"N/A\")} ({nodes[0].get(\"status\", \"unknown\")})')
"
    else
        echo -e "${YELLOW}⚠${NC} API 回應不是有效的 JSON"
    fi
else
    echo -e "${RED}✗${NC} API 代理無回應"
fi

# 5. 測試節點控制 API (只測試格式，不實際執行)
echo -e "\n${YELLOW}5. 測試節點控制 API 格式${NC}"
echo "-----------------------------------"

# 測試啟動 API (使用 OPTIONS 方法檢查)
if curl -s -X OPTIONS "$AGVCUI_URL/nodes/api/node/test_node/start" > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} 節點啟動 API 端點存在"
else
    echo -e "${YELLOW}⚠${NC} 節點啟動 API 可能未正確配置"
fi

# 測試停止 API
if curl -s -X OPTIONS "$AGVCUI_URL/nodes/api/node/test_node/stop" > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} 節點停止 API 端點存在"
else
    echo -e "${YELLOW}⚠${NC} 節點停止 API 可能未正確配置"
fi

# 測試重啟 API
if curl -s -X OPTIONS "$AGVCUI_URL/nodes/api/node/test_node/restart" > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} 節點重啟 API 端點存在"
else
    echo -e "${YELLOW}⚠${NC} 節點重啟 API 可能未正確配置"
fi

# 6. 測試頁面 JavaScript 資源
echo -e "\n${YELLOW}6. 測試頁面資源載入${NC}"
echo "-----------------------------------"

# 檢查頁面是否包含必要的 JavaScript
PAGE_CONTENT=$(curl -s "$AGVCUI_URL/nodes/")
if echo "$PAGE_CONTENT" | grep -q "refreshStatus"; then
    echo -e "${GREEN}✓${NC} 頁面包含狀態刷新功能"
fi
if echo "$PAGE_CONTENT" | grep -q "startNode"; then
    echo -e "${GREEN}✓${NC} 頁面包含節點啟動功能"
fi
if echo "$PAGE_CONTENT" | grep -q "stopNode"; then
    echo -e "${GREEN}✓${NC} 頁面包含節點停止功能"
fi
if echo "$PAGE_CONTENT" | grep -q "controlAGV"; then
    echo -e "${GREEN}✓${NC} 頁面包含 AGV 控制功能"
fi

# 7. 測試導航欄整合
echo -e "\n${YELLOW}7. 測試導航欄整合${NC}"
echo "-----------------------------------"

# 獲取任意 AGVCUI 頁面檢查導航欄
NAV_CONTENT=$(curl -s "$AGVCUI_URL/")
if echo "$NAV_CONTENT" | grep -q 'href="/nodes"'; then
    echo -e "${GREEN}✓${NC} 導航欄包含節點管理連結"
else
    echo -e "${YELLOW}⚠${NC} 導航欄可能未包含節點管理連結"
fi

# 8. 總結
echo -e "\n${YELLOW}================================================${NC}"
echo -e "${YELLOW}                測試總結${NC}"
echo -e "${YELLOW}================================================${NC}"

# 計算成功項目
SUCCESS_COUNT=$(grep -c "✓" /tmp/test_output 2>/dev/null || echo 0)
WARNING_COUNT=$(grep -c "⚠" /tmp/test_output 2>/dev/null || echo 0)
FAILURE_COUNT=$(grep -c "✗" /tmp/test_output 2>/dev/null || echo 0)

echo -e "\nAGVCUI 節點管理整合測試完成"
echo -e "提示："
echo -e "  1. 確保 Web API 服務在 Port 8000 運行"
echo -e "  2. 確保 AGVCUI 服務在 Port 8001 運行"
echo -e "  3. 訪問 ${YELLOW}http://localhost:8001/nodes${NC} 查看節點管理界面"
echo -e "  4. 界面會自動每 30 秒刷新節點狀態"

echo -e "\n如需手動啟動服務："
echo -e "  ${YELLOW}# 在容器內啟動 Web API${NC}"
echo -e "  docker compose -f docker-compose.agvc.yml exec agvc_server bash -c \"source /app/setup.bash && agvc_source && python3 /app/web_api_ws/src/web_api/web_api/api_server.py\""
echo -e ""
echo -e "  ${YELLOW}# 在容器內啟動 AGVCUI${NC}"
echo -e "  docker compose -f docker-compose.agvc.yml exec agvc_server bash -c \"source /app/setup.bash && agvc_source && python3 /app/web_api_ws/src/agvcui/agvcui/agvc_ui_server.py\""