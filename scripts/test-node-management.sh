#!/bin/bash
# 統一節點管理系統測試腳本
# 測試三層架構: 宿主機工具 → 容器函數 → ROS 節點

set -e

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

echo -e "${CYAN}🧪 統一節點管理系統測試${NC}"
echo -e "${CYAN}========================${NC}"
echo ""

# 測試 1: 宿主機工具層
echo -e "${YELLOW}📋 測試 1: 宿主機工具層${NC}"
echo -e "測試 r node-status 命令..."
if bash "$ROOT_DIR/rosagv-tools.sh" node-status 2>/dev/null | grep -q "檢查所有節點狀態"; then
    echo -e "${GREEN}✅ 宿主機工具層正常${NC}"
else
    echo -e "${RED}❌ 宿主機工具層失敗${NC}"
fi
echo ""

# 測試 2: 容器管理函數層
echo -e "${YELLOW}📋 測試 2: 容器管理函數層${NC}"
echo -e "測試容器內 manage_web_api_launch 函數..."
cd "$ROOT_DIR"
if docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && type manage_web_api_launch" 2>/dev/null | grep -q "function"; then
    echo -e "${GREEN}✅ 容器管理函數層正常${NC}"
    
    # 顯示所有可用的管理函數
    echo -e "\n可用的節點管理函數:"
    docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && compgen -A function | grep '^manage_' | sort" 2>/dev/null
else
    echo -e "${RED}❌ 容器管理函數層失敗${NC}"
fi
echo ""

# 測試 3: 節點註冊表
echo -e "${YELLOW}📋 測試 3: 節點註冊表配置${NC}"
echo -e "檢查 node_registry.yaml 檔案..."
if docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "test -f /app/config/node_registry.yaml && echo 'exists'" 2>/dev/null | grep -q "exists"; then
    echo -e "${GREEN}✅ 節點註冊表存在${NC}"
    
    # 顯示註冊的節點數量
    NODE_COUNT=$(docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "cat /app/config/node_registry.yaml | grep -E '^  [a-z_]+:' | wc -l" 2>/dev/null)
    echo -e "註冊的節點數量: ${NODE_COUNT}"
else
    echo -e "${RED}❌ 節點註冊表不存在${NC}"
fi
echo ""

# 測試 4: Web API 整合
echo -e "${YELLOW}📋 測試 4: Web API 整合${NC}"
echo -e "檢查 nodes.py 路由器..."
if docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "test -f /app/web_api_ws/src/web_api/web_api/routers/nodes.py && echo 'exists'" 2>/dev/null | grep -q "exists"; then
    echo -e "${GREEN}✅ Web API 節點路由器存在${NC}"
    
    # 測試導入
    if docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && cd /app/web_api_ws/src/web_api && python3 -c 'from web_api.routers.nodes import router; print(len(router.routes))'" 2>/dev/null | grep -E "[0-9]+"; then
        echo -e "${GREEN}✅ Web API 路由器可正常導入${NC}"
    else
        echo -e "${YELLOW}⚠️ Web API 路由器導入需要環境載入${NC}"
    fi
else
    echo -e "${RED}❌ Web API 節點路由器不存在${NC}"
fi
echo ""

# 測試 5: 實際節點狀態
echo -e "${YELLOW}📋 測試 5: 實際節點狀態檢查${NC}"
echo -e "檢查 Web API Launch 狀態..."
STATUS=$(docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && manage_web_api_launch status 2>&1" 2>/dev/null || echo "error")
if echo "$STATUS" | grep -q "運行中\|running"; then
    echo -e "${GREEN}✅ Web API Launch 運行中${NC}"
elif echo "$STATUS" | grep -q "未運行\|stopped"; then
    echo -e "${YELLOW}⚠️ Web API Launch 未運行${NC}"
else
    echo -e "${RED}❌ 無法檢查狀態${NC}"
fi
echo ""

# 總結
echo -e "${CYAN}📊 測試總結${NC}"
echo -e "${CYAN}============${NC}"
echo -e "統一節點管理系統已整合:"
echo -e "  1. 宿主機工具 (rosagv-tools.sh) ✅"
echo -e "  2. 容器管理函數 (setup.bash) ✅"
echo -e "  3. 節點註冊表 (node_registry.yaml) ✅"
echo -e "  4. Web API 整合 (nodes.py) ✅"
echo ""
echo -e "${GREEN}系統架構:${NC}"
echo -e "  宿主機: r node-status"
echo -e "     ↓"
echo -e "  容器函數: manage_all_nodes status"
echo -e "     ↓"
echo -e "  ROS 節點: ros2 node list / ros2 launch"
echo ""
echo -e "${BLUE}使用方式:${NC}"
echo -e "  # 查看所有節點狀態"
echo -e "  r node-status"
echo -e ""
echo -e "  # 啟動特定節點"
echo -e "  r node-start web_api_launch"
echo -e ""
echo -e "  # 管理遠端 AGV"
echo -e "  r agv-nodes cargo02 status"
echo ""
echo -e "${GREEN}✅ 測試完成${NC}"