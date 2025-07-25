#!/bin/bash
# RosAGV 全面系統健康檢查工具
# 版本: 1.0
# 說明: 檢查容器、服務、網路、配置完整性，生成系統健康報告

# ============================================================================
# 初始化和設定
# ============================================================================

# 獲取腳本目錄和專案根目錄
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 載入 docker-tools 以使用其函數
DOCKER_TOOLS_DIR="$PROJECT_ROOT/scripts/docker-tools"
if [ -f "$DOCKER_TOOLS_DIR/docker-tools.sh" ]; then
    source "$DOCKER_TOOLS_DIR/docker-tools.sh" >/dev/null 2>&1
fi

# 配置檔案路徑
HARDWARE_MAPPING="$PROJECT_ROOT/app/config/hardware_mapping.yaml"
ZENOH_CONFIG="$PROJECT_ROOT/app/routerconfig.json5"
AGV_COMPOSE="$PROJECT_ROOT/docker-compose.yml"
AGVC_COMPOSE="$PROJECT_ROOT/docker-compose.agvc.yml"

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# 檢查結果統計
TOTAL_CHECKS=0
PASSED_CHECKS=0
WARNING_CHECKS=0
FAILED_CHECKS=0

# ============================================================================
# 輔助函數
# ============================================================================

show_header() {
    echo -e "${CYAN}🏥 RosAGV 全面系統健康檢查${NC}"
    echo -e "${CYAN}==============================${NC}"
    echo -e "${BLUE}檢查時間: $(date '+%Y-%m-%d %H:%M:%S')${NC}"
    echo ""
}

show_help() {
    show_header
    echo -e "${YELLOW}用法:${NC}"
    echo "  $(basename $0) [選項]"
    echo ""
    echo -e "${YELLOW}選項:${NC}"
    echo -e "  ${GREEN}--quick${NC}        - 快速檢查 (僅檢查關鍵項目)"
    echo -e "  ${GREEN}--full${NC}         - 完整檢查 (預設)"
    echo -e "  ${GREEN}--report${NC}       - 生成詳細報告檔案"
    echo -e "  ${GREEN}--fix${NC}          - 嘗試修復發現的問題"
    echo -e "  ${GREEN}--cron${NC}         - 定期檢查模式 (簡化輸出)"
    echo ""
    echo -e "${YELLOW}範例:${NC}"
    echo "  $(basename $0)                    # 執行完整檢查"
    echo "  $(basename $0) --quick            # 快速檢查關鍵服務"
    echo "  $(basename $0) --report           # 生成報告到檔案"
    echo "  $(basename $0) --fix              # 檢查並嘗試修復問題"
}

# 檢查結果記錄函數
record_check() {
    local status="$1"
    local message="$2"
    local fix_command="$3"
    
    ((TOTAL_CHECKS++))
    
    case "$status" in
        "PASS")
            ((PASSED_CHECKS++))
            print_success "$message"
            ;;
        "WARN")
            ((WARNING_CHECKS++))
            print_warning "$message"
            if [ -n "$fix_command" ] && [ "$FIX_MODE" = "true" ]; then
                echo -e "${YELLOW}  → 嘗試修復: $fix_command${NC}"
            fi
            ;;
        "FAIL")
            ((FAILED_CHECKS++))
            print_error "$message"
            if [ -n "$fix_command" ] && [ "$FIX_MODE" = "true" ]; then
                echo -e "${YELLOW}  → 嘗試修復: $fix_command${NC}"
            fi
            ;;
    esac
}

print_status() {
    local icon="$1"
    local color="$2"
    local message="$3"
    echo -e "${color}${icon} ${message}${NC}"
}

print_success() {
    print_status "✅" "$GREEN" "$1"
}

print_error() {
    print_status "❌" "$RED" "$1"
}

print_warning() {
    print_status "⚠️" "$YELLOW" "$1"
}

print_info() {
    print_status "ℹ️" "$BLUE" "$1"
}

# ============================================================================
# 系統環境檢查
# ============================================================================

check_system_environment() {
    echo -e "\n${CYAN}1️⃣ 系統環境檢查${NC}"
    echo -e "${CYAN}===============${NC}"
    
    # Docker 服務
    if docker info >/dev/null 2>&1; then
        local docker_version=$(docker --version | awk '{print $3}' | sed 's/,$//')
        record_check "PASS" "Docker 服務運行正常 (版本: $docker_version)"
    else
        record_check "FAIL" "Docker 服務未運行" "sudo systemctl start docker"
    fi
    
    # Docker Compose
    if command -v docker-compose >/dev/null 2>&1 || docker compose version >/dev/null 2>&1; then
        record_check "PASS" "Docker Compose 已安裝"
    else
        record_check "FAIL" "Docker Compose 未安裝"
    fi
    
    # 系統資源
    local mem_total=$(free -m | awk 'NR==2{print $2}')
    local mem_available=$(free -m | awk 'NR==2{print $7}')
    local mem_percent=$((100 - (mem_available * 100 / mem_total)))
    
    if [ $mem_percent -lt 80 ]; then
        record_check "PASS" "記憶體使用正常 ($mem_percent%)"
    elif [ $mem_percent -lt 90 ]; then
        record_check "WARN" "記憶體使用偏高 ($mem_percent%)"
    else
        record_check "FAIL" "記憶體使用過高 ($mem_percent%)"
    fi
    
    # 磁碟空間
    local disk_usage=$(df -h / | awk 'NR==2{print $5}' | sed 's/%//')
    if [ $disk_usage -lt 80 ]; then
        record_check "PASS" "磁碟空間充足 (使用 $disk_usage%)"
    elif [ $disk_usage -lt 90 ]; then
        record_check "WARN" "磁碟空間偏少 (使用 $disk_usage%)"
    else
        record_check "FAIL" "磁碟空間不足 (使用 $disk_usage%)"
    fi
}

# ============================================================================
# 配置檔案檢查
# ============================================================================

check_configuration_files() {
    echo -e "\n${CYAN}2️⃣ 配置檔案檢查${NC}"
    echo -e "${CYAN}===============${NC}"
    
    # Docker Compose 檔案
    if [ -f "$AGV_COMPOSE" ]; then
        if docker compose -f "$AGV_COMPOSE" config >/dev/null 2>&1; then
            record_check "PASS" "AGV Docker Compose 配置有效"
        else
            record_check "FAIL" "AGV Docker Compose 配置無效"
        fi
    else
        record_check "FAIL" "AGV Docker Compose 檔案不存在"
    fi
    
    if [ -f "$AGVC_COMPOSE" ]; then
        if docker compose -f "$AGVC_COMPOSE" config >/dev/null 2>&1; then
            record_check "PASS" "AGVC Docker Compose 配置有效"
        else
            record_check "FAIL" "AGVC Docker Compose 配置無效"
        fi
    else
        record_check "FAIL" "AGVC Docker Compose 檔案不存在"
    fi
    
    # 硬體映射配置
    if [ -f "$HARDWARE_MAPPING" ]; then
        if python3 -c "import yaml; yaml.safe_load(open('$HARDWARE_MAPPING'))" 2>/dev/null; then
            record_check "PASS" "硬體映射配置格式正確"
        else
            record_check "FAIL" "硬體映射配置格式錯誤"
        fi
    else
        record_check "WARN" "硬體映射配置不存在"
    fi
    
    # Zenoh 配置
    if [ -f "$ZENOH_CONFIG" ]; then
        # JSON5 格式較寬鬆，簡單檢查基本結構
        if grep -q "mode.*:.*\"router\"" "$ZENOH_CONFIG" 2>/dev/null; then
            record_check "PASS" "Zenoh 配置檔案存在"
        else
            record_check "WARN" "Zenoh 配置可能不完整"
        fi
    else
        record_check "FAIL" "Zenoh 配置檔案不存在"
    fi
}

# ============================================================================
# 容器狀態檢查
# ============================================================================

check_container_status() {
    echo -e "\n${CYAN}3️⃣ 容器狀態檢查${NC}"
    echo -e "${CYAN}===============${NC}"
    
    # AGV 容器
    if docker ps -q -f name=rosagv >/dev/null 2>&1 && [ -n "$(docker ps -q -f name=rosagv)" ]; then
        local agv_status=$(docker inspect rosagv --format '{{.State.Status}}' 2>/dev/null)
        if [ "$agv_status" = "running" ]; then
            record_check "PASS" "AGV 容器運行正常"
            
            # 檢查環境變數
            local container_type=$(docker exec rosagv printenv CONTAINER_TYPE 2>/dev/null)
            if [ "$container_type" = "agv" ]; then
                record_check "PASS" "AGV 容器環境配置正確"
            else
                record_check "WARN" "AGV 容器環境配置異常"
            fi
        else
            record_check "WARN" "AGV 容器狀態異常: $agv_status"
        fi
    else
        record_check "WARN" "AGV 容器未運行" "$DOCKER_TOOLS_DIR/agv-container.sh start"
    fi
    
    # AGVC 容器群組
    local agvc_containers=("agvc_server" "postgres" "nginx")
    local agvc_running=0
    
    for container in "${agvc_containers[@]}"; do
        if docker ps -q -f name=$container >/dev/null 2>&1 && [ -n "$(docker ps -q -f name=$container)" ]; then
            ((agvc_running++))
        fi
    done
    
    if [ $agvc_running -eq ${#agvc_containers[@]} ]; then
        record_check "PASS" "AGVC 所有容器運行正常 ($agvc_running/${#agvc_containers[@]})"
    elif [ $agvc_running -gt 0 ]; then
        record_check "WARN" "AGVC 部分容器運行 ($agvc_running/${#agvc_containers[@]})" "$DOCKER_TOOLS_DIR/agvc-container.sh restart"
    else
        record_check "WARN" "AGVC 容器未運行" "$DOCKER_TOOLS_DIR/agvc-container.sh start"
    fi
}

# ============================================================================
# 網路連接檢查
# ============================================================================

check_network_connectivity() {
    echo -e "\n${CYAN}4️⃣ 網路連接檢查${NC}"
    echo -e "${CYAN}===============${NC}"
    
    # 檢查 Docker 網路
    if docker network ls | grep -q "rosagv.*bridge" >/dev/null 2>&1; then
        record_check "PASS" "Docker 網路已建立"
    else
        record_check "WARN" "Docker 網路未完全建立"
    fi
    
    # 關鍵端口檢查
    local ports=(
        "80:Nginx Web 服務"
        "7447:Zenoh Router"
        "8000:主 API 服務"
        "5432:PostgreSQL"
    )
    
    for port_info in "${ports[@]}"; do
        local port="${port_info%%:*}"
        local service="${port_info#*:}"
        
        if [ "$port" = "7447" ] || [ "$port" = "8000" ] || [ "$port" = "5432" ]; then
            # 這些服務在 bridge 網路上
            if timeout 2 bash -c "echo > /dev/tcp/192.168.100.100/$port" 2>/dev/null; then
                record_check "PASS" "Port $port ($service) 可連接"
            else
                record_check "FAIL" "Port $port ($service) 無法連接"
            fi
        else
            # Nginx 在 host 上
            if timeout 2 bash -c "echo > /dev/tcp/localhost/$port" 2>/dev/null; then
                record_check "PASS" "Port $port ($service) 可連接"
            else
                record_check "WARN" "Port $port ($service) 無法連接"
            fi
        fi
    done
}

# ============================================================================
# 服務健康檢查
# ============================================================================

check_service_health() {
    echo -e "\n${CYAN}5️⃣ 服務健康檢查${NC}"
    echo -e "${CYAN}===============${NC}"
    
    # PostgreSQL 健康檢查
    if docker ps -q -f name=postgres >/dev/null 2>&1 && [ -n "$(docker ps -q -f name=postgres)" ]; then
        if docker exec postgres pg_isready -U rosagv >/dev/null 2>&1; then
            record_check "PASS" "PostgreSQL 服務健康"
        else
            record_check "FAIL" "PostgreSQL 服務異常"
        fi
    else
        record_check "WARN" "PostgreSQL 容器未運行"
    fi
    
    # Zenoh Router 檢查
    if docker ps -q -f name=agvc_server >/dev/null 2>&1 && [ -n "$(docker ps -q -f name=agvc_server)" ]; then
        if timeout 2 bash -c "echo > /dev/tcp/192.168.100.100/7447" 2>/dev/null; then
            record_check "PASS" "Zenoh Router 服務可達"
        else
            record_check "FAIL" "Zenoh Router 服務不可達"
        fi
    fi
    
    # API 健康檢查
    if timeout 3 curl -s -o /dev/null -w "%{http_code}" http://192.168.100.100:8000/health 2>/dev/null | grep -q "200"; then
        record_check "PASS" "API 服務健康"
    else
        record_check "WARN" "API 服務無響應"
    fi
}

# ============================================================================
# ROS 2 環境檢查
# ============================================================================

check_ros2_environment() {
    if [ "$QUICK_MODE" = "true" ]; then
        return
    fi
    
    echo -e "\n${CYAN}6️⃣ ROS 2 環境檢查${NC}"
    echo -e "${CYAN}===============${NC}"
    
    # 檢查 AGV 容器中的 ROS 2
    if docker ps -q -f name=rosagv >/dev/null 2>&1 && [ -n "$(docker ps -q -f name=rosagv)" ]; then
        local ros_distro=$(docker exec rosagv bash -c 'source /app/setup.bash >/dev/null 2>&1 && echo $ROS_DISTRO' 2>/dev/null)
        if [ "$ros_distro" = "jazzy" ]; then
            record_check "PASS" "AGV ROS 2 環境正常 (Jazzy)"
        else
            record_check "WARN" "AGV ROS 2 環境異常"
        fi
        
        # 檢查 RMW 實現
        local rmw_impl=$(docker exec rosagv printenv RMW_IMPLEMENTATION 2>/dev/null)
        if [ "$rmw_impl" = "rmw_zenoh_cpp" ]; then
            record_check "PASS" "AGV RMW 配置正確 (Zenoh)"
        else
            record_check "WARN" "AGV RMW 配置異常"
        fi
    fi
    
    # 檢查 AGVC 容器中的 ROS 2
    if docker ps -q -f name=agvc_server >/dev/null 2>&1 && [ -n "$(docker ps -q -f name=agvc_server)" ]; then
        local ros_distro=$(docker exec agvc_server bash -c 'source /app/setup.bash >/dev/null 2>&1 && echo $ROS_DISTRO' 2>/dev/null)
        if [ "$ros_distro" = "jazzy" ]; then
            record_check "PASS" "AGVC ROS 2 環境正常 (Jazzy)"
        else
            record_check "WARN" "AGVC ROS 2 環境異常"
        fi
    fi
}

# ============================================================================
# 生成報告
# ============================================================================

generate_report() {
    local report_file="$PROJECT_ROOT/health_check_report_$(date +%Y%m%d_%H%M%S).txt"
    
    {
        echo "======================================"
        echo "RosAGV 系統健康檢查報告"
        echo "======================================"
        echo "檢查時間: $(date '+%Y-%m-%d %H:%M:%S')"
        echo "主機名稱: $(hostname)"
        echo ""
        echo "檢查統計:"
        echo "- 總檢查項目: $TOTAL_CHECKS"
        echo "- 通過: $PASSED_CHECKS"
        echo "- 警告: $WARNING_CHECKS"
        echo "- 失敗: $FAILED_CHECKS"
        echo ""
        echo "健康度評分: $(calculate_health_score)%"
        echo ""
        echo "======================================"
        echo ""
        
        # 重新執行所有檢查並記錄到檔案
        check_system_environment
        check_configuration_files
        check_container_status
        check_network_connectivity
        check_service_health
        check_ros2_environment
        
    } > "$report_file"
    
    echo -e "\n${GREEN}✅ 報告已生成: $report_file${NC}"
}

# ============================================================================
# 計算健康度評分
# ============================================================================

calculate_health_score() {
    local score=100
    
    # 根據檢查結果計算分數
    if [ $TOTAL_CHECKS -gt 0 ]; then
        score=$((PASSED_CHECKS * 100 / TOTAL_CHECKS))
    fi
    
    echo $score
}

# ============================================================================
# 顯示檢查摘要
# ============================================================================

show_summary() {
    echo -e "\n${CYAN}📊 健康檢查摘要${NC}"
    echo -e "${CYAN}===============${NC}"
    
    echo -e "總檢查項目: ${BLUE}$TOTAL_CHECKS${NC}"
    echo -e "✅ 通過: ${GREEN}$PASSED_CHECKS${NC}"
    echo -e "⚠️  警告: ${YELLOW}$WARNING_CHECKS${NC}"
    echo -e "❌ 失敗: ${RED}$FAILED_CHECKS${NC}"
    
    local health_score=$(calculate_health_score)
    echo ""
    
    if [ $health_score -ge 90 ]; then
        echo -e "${GREEN}🎉 系統健康度: $health_score% - 優秀${NC}"
    elif [ $health_score -ge 75 ]; then
        echo -e "${GREEN}✅ 系統健康度: $health_score% - 良好${NC}"
    elif [ $health_score -ge 60 ]; then
        echo -e "${YELLOW}⚠️  系統健康度: $health_score% - 需要關注${NC}"
    else
        echo -e "${RED}❌ 系統健康度: $health_score% - 需要處理${NC}"
    fi
    
    # 建議
    if [ $WARNING_CHECKS -gt 0 ] || [ $FAILED_CHECKS -gt 0 ]; then
        echo -e "\n${YELLOW}💡 建議操作:${NC}"
        
        if [ $FAILED_CHECKS -gt 0 ]; then
            echo -e "  • 執行 ${GREEN}$(basename $0) --fix${NC} 嘗試自動修復問題"
            echo -e "  • 查看詳細日誌了解失敗原因"
        fi
        
        if [ $WARNING_CHECKS -gt 0 ]; then
            echo -e "  • 檢查警告項目並手動處理"
            echo -e "  • 執行 ${GREEN}$(basename $0) --report${NC} 生成詳細報告"
        fi
    fi
}

# ============================================================================
# 嘗試修復問題
# ============================================================================

attempt_fixes() {
    echo -e "\n${CYAN}🔧 嘗試修復發現的問題${NC}"
    echo -e "${CYAN}=====================${NC}"
    
    # Docker 服務修復
    if ! docker info >/dev/null 2>&1; then
        echo -e "${YELLOW}嘗試啟動 Docker 服務...${NC}"
        sudo systemctl start docker
        sleep 2
        if docker info >/dev/null 2>&1; then
            print_success "Docker 服務已啟動"
        else
            print_error "Docker 服務啟動失敗"
        fi
    fi
    
    # 容器修復
    if ! docker ps -q -f name=rosagv >/dev/null 2>&1 || [ -z "$(docker ps -q -f name=rosagv)" ]; then
        echo -e "${YELLOW}嘗試啟動 AGV 容器...${NC}"
        if [ -f "$DOCKER_TOOLS_DIR/agv-container.sh" ]; then
            "$DOCKER_TOOLS_DIR/agv-container.sh" start
        fi
    fi
    
    if ! docker ps -q -f name=agvc_server >/dev/null 2>&1 || [ -z "$(docker ps -q -f name=agvc_server)" ]; then
        echo -e "${YELLOW}嘗試啟動 AGVC 系統...${NC}"
        if [ -f "$DOCKER_TOOLS_DIR/agvc-container.sh" ]; then
            "$DOCKER_TOOLS_DIR/agvc-container.sh" start
        fi
    fi
}

# ============================================================================
# 定期檢查模式
# ============================================================================

cron_check() {
    # 簡化輸出，適合 cron 任務
    local health_score=0
    
    # 執行檢查但不輸出詳細信息
    {
        check_system_environment
        check_container_status
        check_network_connectivity
        check_service_health
    } >/dev/null 2>&1
    
    health_score=$(calculate_health_score)
    
    # 只在有問題時輸出
    if [ $health_score -lt 90 ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] RosAGV Health Check: $health_score% (W:$WARNING_CHECKS/F:$FAILED_CHECKS)"
    fi
    
    # 返回狀態碼
    if [ $health_score -lt 60 ]; then
        exit 2  # 嚴重問題
    elif [ $health_score -lt 90 ]; then
        exit 1  # 有警告
    else
        exit 0  # 健康
    fi
}

# ============================================================================
# 主程式邏輯
# ============================================================================

main() {
    local MODE="full"
    local REPORT_MODE="false"
    local FIX_MODE="false"
    local CRON_MODE="false"
    local QUICK_MODE="false"
    
    # 解析參數
    while [[ $# -gt 0 ]]; do
        case $1 in
            --quick)
                QUICK_MODE="true"
                shift
                ;;
            --full)
                MODE="full"
                shift
                ;;
            --report)
                REPORT_MODE="true"
                shift
                ;;
            --fix)
                FIX_MODE="true"
                shift
                ;;
            --cron)
                CRON_MODE="true"
                shift
                ;;
            -h|--help|help)
                show_help
                exit 0
                ;;
            *)
                print_error "未知選項: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # Cron 模式特殊處理
    if [ "$CRON_MODE" = "true" ]; then
        cron_check
        exit $?
    fi
    
    # 顯示標題
    show_header
    
    # 執行檢查
    check_system_environment
    check_configuration_files
    check_container_status
    check_network_connectivity
    check_service_health
    
    if [ "$QUICK_MODE" != "true" ]; then
        check_ros2_environment
    fi
    
    # 嘗試修復
    if [ "$FIX_MODE" = "true" ] && [ $FAILED_CHECKS -gt 0 ]; then
        attempt_fixes
        
        # 重新檢查
        echo -e "\n${CYAN}🔄 重新檢查系統狀態${NC}"
        TOTAL_CHECKS=0
        PASSED_CHECKS=0
        WARNING_CHECKS=0
        FAILED_CHECKS=0
        
        check_system_environment
        check_container_status
        check_network_connectivity
        check_service_health
    fi
    
    # 顯示摘要
    show_summary
    
    # 生成報告
    if [ "$REPORT_MODE" = "true" ]; then
        generate_report
    fi
    
    # 返回狀態碼
    if [ $FAILED_CHECKS -gt 0 ]; then
        exit 2
    elif [ $WARNING_CHECKS -gt 0 ]; then
        exit 1
    else
        exit 0
    fi
}

# 如果腳本被直接執行（而非被 source）
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi