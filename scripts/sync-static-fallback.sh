#!/bin/bash
# Sync Static Fallback Script
# 用於定期同步 Linear Flow Designer 的靜態備援函數

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROSAGV_DIR="$(dirname "$SCRIPT_DIR")"

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 同步函數
sync_static_fallback() {
    echo -e "${BLUE}🔄 同步靜態備援函數...${NC}"
    
    # 在 AGVC 容器內執行同步腳本
    docker compose -f "$ROSAGV_DIR/docker-compose.agvc.yml" exec -T agvc_server bash -c "
        cd /app/web_api_ws/src/agvcui/agvcui/routers && 
        python3 sync_static_fallback.py
    " 2>/dev/null
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ 同步成功${NC}"
        return 0
    else
        echo -e "${RED}❌ 同步失敗${NC}"
        return 1
    fi
}

# 設置定時任務
setup_cron() {
    echo -e "${BLUE}設置定時任務...${NC}"
    
    # 檢查 crontab 是否已有此任務
    if crontab -l 2>/dev/null | grep -q "sync-static-fallback.sh"; then
        echo -e "${YELLOW}⚠️ 定時任務已存在${NC}"
        return 0
    fi
    
    # 添加定時任務（每天凌晨 2 點執行）
    (crontab -l 2>/dev/null; echo "0 2 * * * $SCRIPT_DIR/sync-static-fallback.sh auto >> /tmp/sync_static_fallback.log 2>&1") | crontab -
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ 定時任務設置成功（每天凌晨 2:00 執行）${NC}"
    else
        echo -e "${RED}❌ 定時任務設置失敗${NC}"
        return 1
    fi
}

# 移除定時任務
remove_cron() {
    echo -e "${BLUE}移除定時任務...${NC}"
    
    crontab -l 2>/dev/null | grep -v "sync-static-fallback.sh" | crontab -
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ 定時任務已移除${NC}"
    else
        echo -e "${RED}❌ 移除失敗${NC}"
        return 1
    fi
}

# 顯示狀態
show_status() {
    echo -e "${BLUE}📊 靜態備援同步狀態${NC}"
    echo "================================"
    
    # 檢查快取檔案
    if [ -f "$ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml" ]; then
        CACHE_TIME=$(stat -c %y "$ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml" 2>/dev/null | cut -d'.' -f1)
        echo -e "快取檔案更新時間: ${GREEN}$CACHE_TIME${NC}"
    else
        echo -e "快取檔案: ${RED}不存在${NC}"
    fi
    
    # 檢查靜態備援 JSON
    if [ -f "$ROSAGV_DIR/app/config/wcs/static_fallback_functions.json" ]; then
        STATIC_TIME=$(stat -c %y "$ROSAGV_DIR/app/config/wcs/static_fallback_functions.json" 2>/dev/null | cut -d'.' -f1)
        echo -e "靜態備援更新時間: ${GREEN}$STATIC_TIME${NC}"
    else
        echo -e "靜態備援 JSON: ${YELLOW}不存在${NC}"
    fi
    
    # 檢查定時任務
    if crontab -l 2>/dev/null | grep -q "sync-static-fallback.sh"; then
        echo -e "定時任務: ${GREEN}已設置${NC}"
        echo "執行時間: 每天凌晨 2:00"
    else
        echo -e "定時任務: ${YELLOW}未設置${NC}"
    fi
    
    # 檢查日誌
    if [ -f "/tmp/sync_static_fallback.log" ]; then
        echo -e "\n最近同步記錄:"
        tail -n 5 /tmp/sync_static_fallback.log
    fi
}

# 顯示幫助
show_help() {
    echo "Linear Flow Designer 靜態備援同步工具"
    echo ""
    echo "用法: $0 [命令]"
    echo ""
    echo "命令:"
    echo "  sync        立即執行同步"
    echo "  setup       設置定時任務（每天凌晨 2:00）"
    echo "  remove      移除定時任務"
    echo "  status      顯示同步狀態"
    echo "  auto        自動模式（由 cron 調用）"
    echo "  help        顯示此幫助信息"
    echo ""
    echo "範例:"
    echo "  $0 sync     # 立即同步靜態備援"
    echo "  $0 setup    # 設置每日自動同步"
    echo "  $0 status   # 查看同步狀態"
}

# 主程式
case "$1" in
    sync)
        sync_static_fallback
        ;;
    setup)
        setup_cron
        ;;
    remove)
        remove_cron
        ;;
    status)
        show_status
        ;;
    auto)
        # 自動模式，不顯示彩色輸出
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] 開始自動同步..."
        sync_static_fallback
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] 同步完成"
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        if [ -z "$1" ]; then
            show_status
            echo ""
            echo "使用 '$0 help' 查看更多選項"
        else
            echo -e "${RED}未知命令: $1${NC}"
            echo ""
            show_help
            exit 1
        fi
        ;;
esac