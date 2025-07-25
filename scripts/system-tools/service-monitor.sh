#!/bin/bash
# RosAGV 服務狀態監控工具
# 版本: 1.0
# 說明: 監控 Zenoh Router, PostgreSQL, Web API 等服務，提供實時監控和自動重啟功能

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

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# 監控配置
MONITOR_INTERVAL=5
ALERT_THRESHOLD=3
MAX_RESTART_ATTEMPTS=3
LOG_FILE="/tmp/rosagv_service_monitor.log"

# 服務狀態記錄
declare -A SERVICE_FAILURE_COUNT
declare -A SERVICE_RESTART_COUNT
declare -A SERVICE_LAST_STATUS

# ============================================================================
# 輔助函數
# ============================================================================

show_header() {
    echo -e "${CYAN}📡 RosAGV 服務狀態監控${NC}"
    echo -e "${CYAN}========================${NC}"
    echo ""
}

show_help() {
    show_header
    echo -e "${YELLOW}用法:${NC}"
    echo "  $(basename $0) [動作] [選項]"
    echo ""
    echo -e "${YELLOW}動作:${NC}"
    echo -e "  ${GREEN}status${NC}         - 顯示所有服務當前狀態 (預設)"
    echo -e "  ${GREEN}monitor${NC}        - 啟動實時監控模式"
    echo -e "  ${GREEN}watch${NC}          - 連續監控服務狀態"
    echo -e "  ${GREEN}restart${NC}        - 重啟指定服務"
    echo -e "  ${GREEN}check${NC}          - 檢查特定服務"
    echo -e "  ${GREEN}deps${NC}           - 檢查服務依賴關係"
    echo -e "  ${GREEN}alerts${NC}         - 查看警報歷史"
    echo ""
    echo -e "${YELLOW}選項:${NC}"
    echo -e "  ${GREEN}--interval <秒>${NC}  - 設定監控間隔 (預設: 5)"
    echo -e "  ${GREEN}--threshold <次>${NC} - 設定警報閾值 (預設: 3)"
    echo -e "  ${GREEN}--auto-restart${NC}  - 啟用自動重啟"
    echo -e "  ${GREEN}--log <檔案>${NC}    - 指定日誌檔案"
    echo ""
    echo -e "${YELLOW}服務名稱:${NC}"
    echo -e "  ${GREEN}zenoh${NC}          - Zenoh Router"
    echo -e "  ${GREEN}postgres${NC}       - PostgreSQL 資料庫"
    echo -e "  ${GREEN}nginx${NC}          - Nginx Web 服務器"
    echo -e "  ${GREEN}api${NC}            - Web API 服務"
    echo -e "  ${GREEN}agv${NC}            - AGV 容器"
    echo -e "  ${GREEN}agvc${NC}           - AGVC 容器"
    echo ""
    echo -e "${YELLOW}範例:${NC}"
    echo "  $(basename $0)                          # 顯示服務狀態"
    echo "  $(basename $0) monitor --auto-restart   # 啟動自動監控"
    echo "  $(basename $0) check zenoh              # 檢查 Zenoh 服務"
    echo "  $(basename $0) restart postgres         # 重啟 PostgreSQL"
    echo "  $(basename $0) watch --interval 10      # 每10秒監控一次"
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

log_message() {
    local level="$1"
    local message="$2"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [$level] $message" >> "$LOG_FILE"
}

# ============================================================================
# 服務檢查函數
# ============================================================================

check_docker_service() {
    if docker info >/dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

check_container_running() {
    local container_name="$1"
    docker ps -q -f name="$container_name" >/dev/null 2>&1 && \
    [ -n "$(docker ps -q -f name="$container_name")" ]
}

check_zenoh_service() {
    if check_container_running "agvc_server"; then
        timeout 3 bash -c "echo > /dev/tcp/192.168.100.100/7447" 2>/dev/null
    else
        return 1
    fi
}

check_postgres_service() {
    if check_container_running "postgres"; then
        docker exec postgres pg_isready -U rosagv >/dev/null 2>&1
    else
        return 1
    fi
}

check_nginx_service() {
    if check_container_running "nginx"; then
        timeout 3 bash -c "echo > /dev/tcp/localhost/80" 2>/dev/null
    else
        return 1
    fi
}

check_api_service() {
    if check_container_running "agvc_server"; then
        timeout 5 curl -s -o /dev/null -w "%{http_code}" http://192.168.100.100:8000/health 2>/dev/null | grep -q "200"
    else
        return 1
    fi
}

check_agv_container() {
    check_container_running "rosagv"
}

check_agvc_container() {
    check_container_running "agvc_server"
}

# ============================================================================
# 服務重啟函數
# ============================================================================

restart_service() {
    local service="$1"
    local success=false
    
    log_message "INFO" "嘗試重啟服務: $service"
    
    case "$service" in
        "zenoh")
            if [ -f "$DOCKER_TOOLS_DIR/agvc-container.sh" ]; then
                "$DOCKER_TOOLS_DIR/agvc-container.sh" restart >/dev/null 2>&1
                sleep 5
                if check_zenoh_service; then
                    success=true
                fi
            fi
            ;;
        "postgres")
            docker restart postgres >/dev/null 2>&1
            sleep 5
            if check_postgres_service; then
                success=true
            fi
            ;;
        "nginx")
            docker restart nginx >/dev/null 2>&1
            sleep 3
            if check_nginx_service; then
                success=true
            fi
            ;;
        "api")
            if [ -f "$DOCKER_TOOLS_DIR/agvc-container.sh" ]; then
                "$DOCKER_TOOLS_DIR/agvc-container.sh" restart >/dev/null 2>&1
                sleep 5
                if check_api_service; then
                    success=true
                fi
            fi
            ;;
        "agv")
            if [ -f "$DOCKER_TOOLS_DIR/agv-container.sh" ]; then
                "$DOCKER_TOOLS_DIR/agv-container.sh" restart >/dev/null 2>&1
                sleep 5
                if check_agv_container; then
                    success=true
                fi
            fi
            ;;
        "agvc")
            if [ -f "$DOCKER_TOOLS_DIR/agvc-container.sh" ]; then
                "$DOCKER_TOOLS_DIR/agvc-container.sh" restart >/dev/null 2>&1
                sleep 10
                if check_agvc_container; then
                    success=true
                fi
            fi
            ;;
        *)
            log_message "ERROR" "未知服務: $service"
            return 1
            ;;
    esac
    
    if [ "$success" = "true" ]; then
        log_message "INFO" "服務重啟成功: $service"
        SERVICE_RESTART_COUNT["$service"]=$((${SERVICE_RESTART_COUNT["$service"]:-0} + 1))
        return 0
    else
        log_message "ERROR" "服務重啟失敗: $service"
        return 1
    fi
}

# ============================================================================
# 服務狀態監控
# ============================================================================

get_service_status() {
    local service="$1"
    
    case "$service" in
        "docker")
            check_docker_service
            ;;
        "zenoh")
            check_zenoh_service
            ;;
        "postgres")
            check_postgres_service
            ;;
        "nginx")
            check_nginx_service
            ;;
        "api")
            check_api_service
            ;;
        "agv")
            check_agv_container
            ;;
        "agvc")
            check_agvc_container
            ;;
        *)
            return 2  # 未知服務
            ;;
    esac
}

display_service_status() {
    local service="$1"
    local show_details="$2"
    
    if get_service_status "$service"; then
        local status_text="運行正常"
        local restart_count=${SERVICE_RESTART_COUNT["$service"]:-0}
        
        if [ $restart_count -gt 0 ]; then
            status_text="$status_text (重啟 $restart_count 次)"
        fi
        
        print_success "$service: $status_text"
        
        if [ "$show_details" = "true" ]; then
            case "$service" in
                "postgres")
                    local connections=$(docker exec postgres psql -U rosagv -t -c "SELECT count(*) FROM pg_stat_activity;" 2>/dev/null | tr -d ' ')
                    echo -e "    連接數: ${connections:-未知}"
                    ;;
                "zenoh")
                    echo -e "    端口: 7447"
                    ;;
                "api")
                    local response_time=$(timeout 5 curl -s -o /dev/null -w "%{time_total}" http://192.168.100.100:8000/health 2>/dev/null)
                    echo -e "    響應時間: ${response_time:-未知}s"
                    ;;
            esac
        fi
    else
        local failure_count=${SERVICE_FAILURE_COUNT["$service"]:-0}
        print_error "$service: 服務異常 (失敗 $failure_count 次)"
    fi
}

# ============================================================================
# 主要功能函數
# ============================================================================

show_all_status() {
    echo -e "${CYAN}📊 服務狀態總覽${NC}"
    echo -e "${CYAN}===============${NC}"
    echo -e "檢查時間: ${BLUE}$(date '+%Y-%m-%d %H:%M:%S')${NC}"
    echo ""
    
    local services=("docker" "agv" "agvc" "postgres" "nginx" "zenoh" "api")
    local healthy_services=0
    
    for service in "${services[@]}"; do
        display_service_status "$service" "true"
        if get_service_status "$service"; then
            ((healthy_services++))
        fi
    done
    
    echo ""
    local health_percent=$((healthy_services * 100 / ${#services[@]}))
    
    if [ $health_percent -eq 100 ]; then
        echo -e "${GREEN}🎉 所有服務運行正常 ($healthy_services/${#services[@]})${NC}"
    elif [ $health_percent -ge 80 ]; then
        echo -e "${YELLOW}⚠️  部分服務異常 ($healthy_services/${#services[@]})${NC}"
    else
        echo -e "${RED}❌ 多個服務異常 ($healthy_services/${#services[@]})${NC}"
    fi
}

monitor_services() {
    local auto_restart="$1"
    
    echo -e "${CYAN}🔄 啟動服務監控模式${NC}"
    echo -e "${CYAN}===================${NC}"
    echo -e "監控間隔: ${BLUE}${MONITOR_INTERVAL}秒${NC}"
    echo -e "警報閾值: ${BLUE}${ALERT_THRESHOLD}次${NC}"
    echo -e "自動重啟: ${BLUE}$([ "$auto_restart" = "true" ] && echo "啟用" || echo "停用")${NC}"
    echo -e "日誌檔案: ${BLUE}$LOG_FILE${NC}"
    echo ""
    echo -e "${YELLOW}按 Ctrl+C 停止監控${NC}"
    echo ""
    
    local services=("zenoh" "postgres" "nginx" "api" "agv" "agvc")
    
    # 初始化服務狀態
    for service in "${services[@]}"; do
        SERVICE_FAILURE_COUNT["$service"]=0
        SERVICE_RESTART_COUNT["$service"]=0
    done
    
    log_message "INFO" "開始服務監控"
    
    while true; do
        local current_time=$(date '+%H:%M:%S')
        local status_line="[$current_time] "
        local alerts=""
        
        for service in "${services[@]}"; do
            if get_service_status "$service"; then
                # 服務正常
                SERVICE_FAILURE_COUNT["$service"]=0
                status_line+="$service:✅ "
                
                # 記錄服務恢復
                if [ "${SERVICE_LAST_STATUS["$service"]}" = "failed" ]; then
                    log_message "INFO" "服務恢復: $service"
                fi
                SERVICE_LAST_STATUS["$service"]="ok"
            else
                # 服務異常
                SERVICE_FAILURE_COUNT["$service"]=$((${SERVICE_FAILURE_COUNT["$service"]} + 1))
                status_line+="$service:❌ "
                
                local failure_count=${SERVICE_FAILURE_COUNT["$service"]}
                
                # 記錄首次失敗
                if [ "${SERVICE_LAST_STATUS["$service"]}" != "failed" ]; then
                    log_message "WARN" "服務異常: $service"
                fi
                SERVICE_LAST_STATUS["$service"]="failed"
                
                # 檢查是否達到警報閾值
                if [ $failure_count -ge $ALERT_THRESHOLD ]; then
                    alerts+="$service($failure_count) "
                    
                    # 自動重啟
                    if [ "$auto_restart" = "true" ] && [ ${SERVICE_RESTART_COUNT["$service"]:-0} -lt $MAX_RESTART_ATTEMPTS ]; then
                        echo -e "\n${YELLOW}⚠️  $service 服務持續異常，嘗試自動重啟...${NC}"
                        if restart_service "$service"; then
                            echo -e "${GREEN}✅ $service 重啟成功${NC}"
                            SERVICE_FAILURE_COUNT["$service"]=0
                        else
                            echo -e "${RED}❌ $service 重啟失敗${NC}"
                        fi
                    fi
                fi
            fi
        done
        
        # 顯示狀態行
        echo -ne "\r$status_line"
        
        # 顯示警報
        if [ -n "$alerts" ]; then
            echo -e "\n${RED}🚨 警報: $alerts${NC}"
        fi
        
        sleep $MONITOR_INTERVAL
    done
}

watch_services() {
    echo -e "${CYAN}👀 連續監控服務狀態${NC}"
    echo -e "${CYAN}===================${NC}"
    echo -e "${YELLOW}按 Ctrl+C 停止監控${NC}"
    echo ""
    
    while true; do
        clear
        echo -e "${CYAN}📊 RosAGV 服務狀態監控${NC}"
        echo -e "${CYAN}========================${NC}"
        echo -e "更新時間: $(date '+%Y-%m-%d %H:%M:%S')"
        echo ""
        
        show_all_status
        
        echo ""
        echo -e "${BLUE}下次更新: ${MONITOR_INTERVAL}秒後${NC}"
        
        sleep $MONITOR_INTERVAL
    done
}

check_single_service() {
    local service="$1"
    
    echo -e "${CYAN}🔍 檢查服務: $service${NC}"
    echo -e "${CYAN}=================${NC}"
    
    if get_service_status "$service"; then
        print_success "$service 服務運行正常"
        
        case "$service" in
            "postgres")
                echo ""
                echo -e "${YELLOW}PostgreSQL 詳細資訊:${NC}"
                docker exec postgres psql -U rosagv -c "SELECT version();" 2>/dev/null | head -3
                echo ""
                local db_size=$(docker exec postgres psql -U rosagv -t -c "SELECT pg_size_pretty(pg_database_size('rosagv'));" 2>/dev/null | tr -d ' ')
                echo -e "資料庫大小: ${db_size:-未知}"
                local connections=$(docker exec postgres psql -U rosagv -t -c "SELECT count(*) FROM pg_stat_activity;" 2>/dev/null | tr -d ' ')
                echo -e "當前連接數: ${connections:-未知}"
                ;;
            "zenoh")
                echo ""
                echo -e "${YELLOW}Zenoh Router 詳細資訊:${NC}"
                echo -e "端口: 7447"
                echo -e "協議: TCP"
                local zenoh_status=$(timeout 3 bash -c "echo > /dev/tcp/192.168.100.100/7447" 2>&1)
                echo -e "連接狀態: $([ $? -eq 0 ] && echo "正常" || echo "異常")"
                ;;
            "api")
                echo ""
                echo -e "${YELLOW}API 服務詳細資訊:${NC}"
                local api_response=$(timeout 5 curl -s http://192.168.100.100:8000/health 2>/dev/null)
                echo -e "健康檢查: $([ -n "$api_response" ] && echo "通過" || echo "失敗")"
                ;;
        esac
    else
        print_error "$service 服務異常"
        echo ""
        echo -e "${YELLOW}建議操作:${NC}"
        echo -e "  • 檢查容器狀態: docker ps"
        echo -e "  • 查看容器日誌: docker logs $service"
        echo -e "  • 重啟服務: $(basename $0) restart $service"
    fi
}

check_service_dependencies() {
    echo -e "${CYAN}🔗 服務依賴關係檢查${NC}"
    echo -e "${CYAN}===================${NC}"
    
    echo -e "${YELLOW}依賴關係圖:${NC}"
    echo ""
    echo "  Docker 服務"
    echo "  ├── AGV 容器 (rosagv)"
    echo "  └── AGVC 系統"
    echo "      ├── AGVC 容器 (agvc_server)"
    echo "      │   ├── Zenoh Router (端口 7447)"
    echo "      │   └── Web API (端口 8000-8002)"
    echo "      ├── PostgreSQL (端口 5432)"
    echo "      └── Nginx (端口 80)"
    echo ""
    
    echo -e "${YELLOW}依賴檢查結果:${NC}"
    
    # Docker 服務
    if get_service_status "docker"; then
        print_success "Docker 服務 - 正常"
        
        # AGV 容器
        if get_service_status "agv"; then
            print_success "  └── AGV 容器 - 正常"
        else
            print_warning "  └── AGV 容器 - 異常"
        fi
        
        # AGVC 系統
        if get_service_status "agvc"; then
            print_success "  └── AGVC 容器 - 正常"
            
            # Zenoh Router
            if get_service_status "zenoh"; then
                print_success "      ├── Zenoh Router - 正常"
            else
                print_error "      ├── Zenoh Router - 異常"
            fi
            
            # API 服務
            if get_service_status "api"; then
                print_success "      └── Web API - 正常"
            else
                print_error "      └── Web API - 異常"
            fi
        else
            print_error "  └── AGVC 容器 - 異常"
            print_warning "      ├── Zenoh Router - 依賴失敗"
            print_warning "      └── Web API - 依賴失敗"
        fi
        
        # PostgreSQL
        if get_service_status "postgres"; then
            print_success "  └── PostgreSQL - 正常"
        else
            print_error "  └── PostgreSQL - 異常"
        fi
        
        # Nginx
        if get_service_status "nginx"; then
            print_success "  └── Nginx - 正常"
        else
            print_warning "  └── Nginx - 異常"
        fi
    else
        print_error "Docker 服務 - 異常"
        print_error "  所有容器服務都無法運行"
    fi
}

show_alerts() {
    echo -e "${CYAN}🚨 警報歷史${NC}"
    echo -e "${CYAN}==========${NC}"
    
    if [ -f "$LOG_FILE" ]; then
        echo -e "${YELLOW}最近的警報和錯誤:${NC}"
        tail -20 "$LOG_FILE" | grep -E "(WARN|ERROR)" | while read line; do
            if echo "$line" | grep -q "ERROR"; then
                echo -e "${RED}$line${NC}"
            else
                echo -e "${YELLOW}$line${NC}"
            fi
        done
        
        echo ""
        echo -e "${YELLOW}統計資訊:${NC}"
        local total_errors=$(grep -c "ERROR" "$LOG_FILE" 2>/dev/null || echo 0)
        local total_warnings=$(grep -c "WARN" "$LOG_FILE" 2>/dev/null || echo 0)
        echo -e "錯誤總數: ${RED}$total_errors${NC}"
        echo -e "警告總數: ${YELLOW}$total_warnings${NC}"
        echo -e "日誌檔案: $LOG_FILE"
    else
        print_info "暫無警報記錄"
    fi
}

# ============================================================================
# 主程式邏輯
# ============================================================================

main() {
    local action="status"
    local auto_restart="false"
    local service=""
    
    # 解析參數
    while [[ $# -gt 0 ]]; do
        case $1 in
            status|monitor|watch|restart|check|deps|alerts)
                action="$1"
                shift
                ;;
            --interval)
                MONITOR_INTERVAL="$2"
                shift 2
                ;;
            --threshold)
                ALERT_THRESHOLD="$2"
                shift 2
                ;;
            --auto-restart)
                auto_restart="true"
                shift
                ;;
            --log)
                LOG_FILE="$2"
                shift 2
                ;;
            -h|--help|help)
                show_help
                exit 0
                ;;
            *)
                if [ -z "$service" ]; then
                    service="$1"
                else
                    print_error "未知選項: $1"
                    show_help
                    exit 1
                fi
                shift
                ;;
        esac
    done
    
    # 執行動作
    case "$action" in
        "status")
            show_all_status
            ;;
        "monitor")
            monitor_services "$auto_restart"
            ;;
        "watch")
            watch_services
            ;;
        "restart")
            if [ -z "$service" ]; then
                print_error "請指定要重啟的服務"
                show_help
                exit 1
            fi
            restart_service "$service"
            ;;
        "check")
            if [ -z "$service" ]; then
                print_error "請指定要檢查的服務"
                show_help
                exit 1
            fi
            check_single_service "$service"
            ;;
        "deps")
            check_service_dependencies
            ;;
        "alerts")
            show_alerts
            ;;
        *)
            print_error "未知動作: $action"
            show_help
            exit 1
            ;;
    esac
}

# 信號處理
trap 'echo -e "\n${YELLOW}監控已停止${NC}"; log_message "INFO" "監控停止"; exit 0' INT TERM

# 如果腳本被直接執行（而非被 source）
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi