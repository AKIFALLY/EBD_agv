#!/bin/bash
# RosAGV 測試案例執行腳本
# 提供便利的測試執行入口

# 設置顏色輸出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 檢查是否在容器內
if [ ! -d "/app" ]; then
    echo -e "${RED}❌ 此腳本必須在 AGVC 容器內執行${NC}"
    echo -e "${YELLOW}請使用: docker compose -f docker-compose.agvc.yml exec agvc_server bash${NC}"
    exit 1
fi

# 顯示可用測試案例
show_help() {
    echo -e "${BLUE}🧪 RosAGV 測試案例執行工具${NC}"
    echo -e "${BLUE}================================${NC}"
    echo ""
    echo -e "${GREEN}可用的測試案例:${NC}"
    echo -e "  ${YELLOW}lifecycle${NC}    - OPUI 任務完整生命週期測試 (推薦)"
    echo -e "  ${YELLOW}flow${NC}         - OPUI-AI WCS-RCS 流程驗證測試"
    echo -e "  ${YELLOW}conditions${NC}   - AI WCS 決策條件檢查"
    echo -e "  ${YELLOW}machine${NC}      - 機台停車位狀態檢查"
    echo ""
    echo -e "${GREEN}使用方式:${NC}"
    echo -e "  ${BLUE}./run_tests.sh <test_name>${NC}"
    echo -e "  ${BLUE}./run_tests.sh --help${NC}"
    echo ""
    echo -e "${GREEN}範例:${NC}"
    echo -e "  ${BLUE}./run_tests.sh lifecycle${NC}     # 執行完整生命週期測試"
    echo -e "  ${BLUE}./run_tests.sh flow${NC}          # 執行流程驗證測試"
}

# 執行測試案例
run_test() {
    local test_name=$1
    local test_path="/app/test-cases/opui-task-lifecycle"
    
    case $test_name in
        "lifecycle")
            echo -e "${GREEN}🚀 執行 OPUI 任務完整生命週期測試...${NC}"
            cd $test_path
            python3 test_task_lifecycle.py
            ;;
        "flow")
            echo -e "${GREEN}🚀 執行 OPUI-AI WCS-RCS 流程驗證測試...${NC}"
            cd $test_path
            python3 test_opui_rcs_wcs_flow.py
            ;;
        "conditions")
            echo -e "${GREEN}🚀 執行 AI WCS 決策條件檢查...${NC}"
            cd $test_path
            python3 check_wcs_conditions.py
            ;;
        "machine")
            echo -e "${GREEN}🚀 執行機台停車位狀態檢查...${NC}"
            cd $test_path
            python3 check_machine_status.py
            ;;
        *)
            echo -e "${RED}❌ 未知的測試案例: $test_name${NC}"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# 主程式邏輯
main() {
    if [ $# -eq 0 ] || [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
        show_help
        exit 0
    fi
    
    echo -e "${BLUE}🔬 RosAGV 測試案例執行器${NC}"
    echo -e "${BLUE}========================${NC}"
    echo ""
    
    # 檢查 ROS 2 環境
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${YELLOW}⚠️  ROS 2 環境未載入，正在載入工作空間...${NC}"
        source /opt/ros/jazzy/setup.bash
        if [ -f "/app/setup.bash" ]; then
            source /app/setup.bash
        fi
    fi
    
    run_test $1
}

# 執行主程式
main "$@"