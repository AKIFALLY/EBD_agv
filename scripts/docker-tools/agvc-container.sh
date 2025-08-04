#!/bin/bash
# RosAGV AGVC 容器專用管理工具
# 版本: 1.0
# 說明: AGVC 容器的啟動、停止、進入和狀態檢查工具

# ============================================================================
# 初始化和設定
# ============================================================================

# 獲取腳本目錄和專案根目錄
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Docker Compose 檔案路徑
COMPOSE_FILE="$PROJECT_ROOT/docker-compose.agvc.yml"
CONTAINER_NAME="agvc_server"
SERVICE_NAME="agvc_server"

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# ============================================================================
# 輔助函數
# ============================================================================

show_header() {
    echo -e "${CYAN}🖥️ RosAGV AGVC 容器管理工具${NC}"
    echo -e "${CYAN}==============================${NC}"
    echo ""
}

show_help() {
    show_header
    echo -e "${YELLOW}用法:${NC}"
    echo "  $(basename $0) [動作]"
    echo ""
    echo -e "${YELLOW}可用動作:${NC}"
    echo -e "  ${GREEN}start${NC}     - 啟動 AGVC 系統 (所有服務)"
    echo -e "  ${GREEN}stop${NC}      - 停止 AGVC 系統 (所有服務)"
    echo -e "  ${GREEN}restart${NC}   - 重啟 AGVC 系統"
    echo -e "  ${GREEN}status${NC}    - 檢查 AGVC 系統狀態"
    echo -e "  ${GREEN}logs${NC}      - 顯示 AGVC 容器日誌"
    echo -e "  ${GREEN}exec${NC}      - 進入 AGVC 容器 (自動載入 agvc_source)"
    echo -e "  ${GREEN}shell${NC}     - 進入 AGVC 容器 shell"
    echo -e "  ${GREEN}health${NC}    - AGVC 系統健康檢查"
    echo -e "  ${GREEN}services${NC}  - 檢查所有 AGVC 服務狀態"
    echo -e "  ${GREEN}overview${NC}  - 顯示 AGVC 系統概況 (預設)"
    echo ""
    echo -e "${YELLOW}範例:${NC}"
    echo "  $(basename $0) start"
    echo "  $(basename $0) exec"
    echo "  $(basename $0) logs agvc_server -f"
    echo "  $(basename $0) health"
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
# 容器狀態檢查函數
# ============================================================================

check_docker_running() {
    if ! docker info >/dev/null 2>&1; then
        print_error "Docker 服務未運行，請先啟動 Docker"
        return 1
    fi
    return 0
}

check_compose_file() {
    if [ ! -f "$COMPOSE_FILE" ]; then
        print_error "找不到 Docker Compose 檔案: $COMPOSE_FILE"
        return 1
    fi
    return 0
}

get_all_services_status() {
    docker compose -f "$COMPOSE_FILE" ps --format "table" 2>/dev/null
}

get_service_status() {
    local service="$1"
    docker compose -f "$COMPOSE_FILE" ps --format "table" --filter "service=$service" 2>/dev/null | tail -n +2
}

is_service_running() {
    local service="$1"
    local status=$(docker compose -f "$COMPOSE_FILE" ps -q "$service" 2>/dev/null)
    [ -n "$status" ] && docker compose -f "$COMPOSE_FILE" ps "$service" 2>/dev/null | grep -q "$service.*Up"
}

is_agvc_container_running() {
    is_service_running "$SERVICE_NAME"
}

# ============================================================================
# 服務檢查函數
# ============================================================================

check_postgres_service() {
    if is_service_running "postgres"; then
        # 檢查 PostgreSQL 連接
        if docker exec postgres pg_isready -U agvc >/dev/null 2>&1; then
            print_success "PostgreSQL: 正常運行"
            return 0
        else
            print_warning "PostgreSQL: 容器運行但服務異常"
            return 1
        fi
    else
        print_error "PostgreSQL: 未運行"
        return 1
    fi
}

check_nginx_service() {
    if is_service_running "nginx"; then
        # 檢查 Nginx 端口
        if curl -s -o /dev/null -w "%{http_code}" http://localhost:80 | grep -q "200\|301\|302"; then
            print_success "Nginx: 正常運行 (Port 80)"
            return 0
        else
            print_warning "Nginx: 容器運行但端口無響應"
            return 1
        fi
    else
        print_error "Nginx: 未運行"
        return 1
    fi
}

check_agvc_server_service() {
    if is_agvc_container_running; then
        # 檢查環境變數
        local container_type=$(docker exec "$CONTAINER_NAME" printenv CONTAINER_TYPE 2>/dev/null)
        if [ "$container_type" = "agvc" ]; then
            print_success "AGVC Server: 正常運行"
            return 0
        else
            print_warning "AGVC Server: 容器運行但環境異常"
            return 1
        fi
    else
        print_error "AGVC Server: 未運行"
        return 1
    fi
}

check_api_endpoints() {
    if ! is_agvc_container_running; then
        print_error "API 端點: AGVC 容器未運行"
        return 1
    fi
    
    local healthy_endpoints=0
    local total_endpoints=3
    
    # 檢查端口 8000 (主 API)
    if timeout 3 bash -c "echo > /dev/tcp/192.168.100.100/8000" 2>/dev/null; then
        print_success "API 端點: 8000 (主 API) 可連接"
        ((healthy_endpoints++))
    else
        print_error "API 端點: 8000 (主 API) 無法連接"
    fi
    
    # 檢查端口 8001 (AGVCUI)
    if timeout 3 bash -c "echo > /dev/tcp/192.168.100.100/8001" 2>/dev/null; then
        print_success "API 端點: 8001 (AGVCUI) 可連接"
        ((healthy_endpoints++))
    else
        print_warning "API 端點: 8001 (AGVCUI) 無法連接"
    fi
    
    # 檢查端口 8002 (OPUI)
    if timeout 3 bash -c "echo > /dev/tcp/192.168.100.100/8002" 2>/dev/null; then
        print_success "API 端點: 8002 (OPUI) 可連接"
        ((healthy_endpoints++))
    else
        print_warning "API 端點: 8002 (OPUI) 無法連接"
    fi
    
    if [ $healthy_endpoints -eq $total_endpoints ]; then
        return 0
    elif [ $healthy_endpoints -gt 0 ]; then
        return 1
    else
        return 2
    fi
}

check_zenoh_service() {
    if ! is_agvc_container_running; then
        print_error "Zenoh Router: AGVC 容器未運行"
        return 1
    fi
    
    # 檢查 Zenoh Router 端口
    if timeout 3 bash -c "echo > /dev/tcp/192.168.100.100/7447" 2>/dev/null; then
        print_success "Zenoh Router: 正常運行 (Port 7447)"
        return 0
    else
        print_error "Zenoh Router: 端口 7447 無法連接"
        return 1
    fi
}

# ============================================================================
# 主要功能函數
# ============================================================================

agvc_start() {
    print_info "啟動 AGVC 系統..."
    
    # 檢查是否已經運行
    local running_services=$(docker compose -f "$COMPOSE_FILE" ps -q 2>/dev/null | wc -l)
    if [ "$running_services" -gt 0 ]; then
        print_warning "部分 AGVC 服務已在運行，將重新啟動所有服務"
    fi
    
    docker compose -f "$COMPOSE_FILE" up -d
    if [ $? -eq 0 ]; then
        sleep 5  # 等待服務啟動
        print_success "AGVC 系統啟動成功"
        
        # 快速檢查關鍵服務
        echo ""
        print_info "檢查關鍵服務狀態..."
        check_agvc_server_service
        check_postgres_service
        check_nginx_service
    else
        print_error "AGVC 系統啟動失敗"
        return 1
    fi
}

agvc_stop() {
    print_info "停止 AGVC 系統..."
    
    local running_services=$(docker compose -f "$COMPOSE_FILE" ps -q 2>/dev/null | wc -l)
    if [ "$running_services" -eq 0 ]; then
        print_warning "AGVC 系統未在運行"
        return 0
    fi
    
    docker compose -f "$COMPOSE_FILE" down
    if [ $? -eq 0 ]; then
        print_success "AGVC 系統停止成功"
    else
        print_error "AGVC 系統停止失敗"
        return 1
    fi
}

agvc_restart() {
    print_info "重啟 AGVC 系統..."
    agvc_stop
    sleep 3
    agvc_start
}

agvc_status() {
    echo -e "${CYAN}📊 AGVC 系統狀態${NC}"
    echo -e "${CYAN}===============${NC}"
    
    local status_output=$(get_all_services_status)
    if [ -n "$status_output" ]; then
        echo "$status_output"
    else
        print_warning "AGVC 系統未啟動"
        return 1
    fi
    
    echo ""
    
    # 檢查網路配置
    local network_info=$(docker network ls | grep "agvc.*bridge")
    if [ -n "$network_info" ]; then
        print_success "Docker 網路: bridge_network 已建立"
    else
        print_warning "Docker 網路: bridge_network 未建立"
    fi
    
    # 檢查 AGVC 容器 IP
    if is_agvc_container_running; then
        local agvc_ip=$(docker inspect "$CONTAINER_NAME" --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' 2>/dev/null)
        if [ "$agvc_ip" = "192.168.100.100" ]; then
            print_success "AGVC 容器 IP: $agvc_ip (正確)"
        else
            print_warning "AGVC 容器 IP: $agvc_ip (異常，應為 192.168.100.100)"
        fi
    fi
}

agvc_logs() {
    local service="${1:-$SERVICE_NAME}"
    shift
    local log_args="$@"
    
    print_info "顯示 $service 服務日誌..."
    
    # 驗證服務名稱
    local valid_services="agvc_server nginx postgres"
    if ! echo "$valid_services" | grep -q "$service"; then
        print_error "無效的服務名稱: $service"
        print_info "有效的服務: $valid_services"
        return 1
    fi
    
    if [ -n "$log_args" ]; then
        docker compose -f "$COMPOSE_FILE" logs $log_args "$service"
    else
        docker compose -f "$COMPOSE_FILE" logs --tail=50 "$service"
    fi
}

agvc_exec() {
    if ! is_agvc_container_running; then
        print_error "AGVC 容器未在運行，無法進入"
        print_info "請先使用 '$(basename $0) start' 啟動系統"
        return 1
    fi
    
    print_info "進入 AGVC 容器並自動載入 agvc_source..."
    
    # 進入容器並自動執行 agvc_source
    docker exec -it "$CONTAINER_NAME" bash -c "
        echo -e '\033[0;32m🖥️ 進入 AGVC 容器環境\033[0m'
        echo -e '\033[0;36m載入 AGVC 開發環境...\033[0m'
        source /app/setup.bash && agvc_source
        echo -e '\033[0;32m✅ AGVC 環境載入完成\033[0m'
        exec bash
    "
}

agvc_shell() {
    if ! is_agvc_container_running; then
        print_error "AGVC 容器未在運行，無法進入"
        print_info "請先使用 '$(basename $0) start' 啟動系統"
        return 1
    fi
    
    print_info "進入 AGVC 容器 shell..."
    docker exec -it "$CONTAINER_NAME" bash
}

agvc_services() {
    echo -e "${CYAN}🔧 AGVC 服務狀態檢查${NC}"
    echo -e "${CYAN}===================${NC}"
    
    echo -e "\n${YELLOW}1. 核心服務${NC}"
    check_agvc_server_service
    check_postgres_service
    check_nginx_service
    
    echo -e "\n${YELLOW}2. 通訊服務${NC}"
    check_zenoh_service
    
    echo -e "\n${YELLOW}3. API 端點${NC}"
    check_api_endpoints
    
    echo -e "\n${YELLOW}4. 容器網路${NC}"
    if is_agvc_container_running; then
        local agvc_ip=$(docker inspect "$CONTAINER_NAME" --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' 2>/dev/null)
        local nginx_ip=$(docker inspect "nginx" --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' 2>/dev/null)
        
        print_info "AGVC 容器 IP: $agvc_ip"
        print_info "Nginx 容器 IP: $nginx_ip"
    else
        print_error "容器未運行，無法檢查網路配置"
    fi
}

agvc_health() {
    echo -e "${CYAN}🏥 AGVC 系統健康檢查${NC}"
    echo -e "${CYAN}=====================${NC}"
    
    local health_score=0
    local total_checks=8
    
    # 1. 檢查核心容器
    echo -e "\n${YELLOW}1. 核心容器檢查${NC}"
    if is_agvc_container_running; then
        print_success "AGVC 容器運行正常"
        ((health_score++))
    else
        print_error "AGVC 容器未運行"
    fi
    
    if is_service_running "postgres"; then
        print_success "PostgreSQL 容器運行正常"
        ((health_score++))
    else
        print_error "PostgreSQL 容器未運行"
    fi
    
    if is_service_running "nginx"; then
        print_success "Nginx 容器運行正常"
        ((health_score++))
    else
        print_error "Nginx 容器未運行"
    fi
    
    if is_agvc_container_running; then
        # 2. 檢查環境變數
        echo -e "\n${YELLOW}2. 環境變數檢查${NC}"
        local container_type=$(docker exec "$CONTAINER_NAME" printenv CONTAINER_TYPE 2>/dev/null)
        local rmw_impl=$(docker exec "$CONTAINER_NAME" printenv RMW_IMPLEMENTATION 2>/dev/null)
        
        if [ "$container_type" = "agvc" ]; then
            print_success "CONTAINER_TYPE: $container_type"
            ((health_score++))
        else
            print_error "CONTAINER_TYPE 不正確: $container_type"
        fi
        
        if [ "$rmw_impl" = "rmw_zenoh_cpp" ]; then
            print_success "RMW_IMPLEMENTATION: $rmw_impl"
            ((health_score++))
        else
            print_error "RMW_IMPLEMENTATION 不正確: $rmw_impl"
        fi
        
        # 3. 檢查關鍵服務
        echo -e "\n${YELLOW}3. 關鍵服務檢查${NC}"
        if check_postgres_service >/dev/null 2>&1; then
            ((health_score++))
        fi
        
        if check_zenoh_service >/dev/null 2>&1; then
            ((health_score++))
        fi
        
        # 4. 檢查開發環境
        echo -e "\n${YELLOW}4. 開發環境檢查${NC}"
        if docker exec "$CONTAINER_NAME" test -f /app/setup.bash 2>/dev/null; then
            print_success "setup.bash 存在"
            ((health_score++))
        else
            print_error "setup.bash 不存在"
        fi
    fi
    
    # 健康度評分
    echo -e "\n${CYAN}📊 健康度評分${NC}"
    local percentage=$((health_score * 100 / total_checks))
    
    if [ $percentage -ge 85 ]; then
        print_success "健康度: $health_score/$total_checks ($percentage%) - 系統健康"
    elif [ $percentage -ge 60 ]; then
        print_warning "健康度: $health_score/$total_checks ($percentage%) - 系統部分異常"
    else
        print_error "健康度: $health_score/$total_checks ($percentage%) - 系統異常"
    fi
}

agvc_overview() {
    show_header
    
    echo -e "${YELLOW}📋 AGVC 系統概況${NC}"
    echo ""
    
    # 檢查系統狀態
    local running_services=$(docker compose -f "$COMPOSE_FILE" ps -q 2>/dev/null | wc -l)
    local total_services=3  # agvc_server, postgres, nginx
    
    if [ "$running_services" -eq "$total_services" ]; then
        print_success "狀態: 所有服務運行中 ($running_services/$total_services)"
    elif [ "$running_services" -gt 0 ]; then
        print_warning "狀態: 部分服務運行中 ($running_services/$total_services)"
    else
        print_warning "狀態: 系統未啟動"
        print_info "使用 '$(basename $0) start' 啟動系統"
    fi
    
    if [ "$running_services" -gt 0 ]; then
        echo ""
        print_info "系統配置:"
        print_info "• AGVC 容器 IP: 192.168.100.100"
        print_info "• 主要端口: 7447(Zenoh), 8000-8002(API), 5432(DB)"
        print_info "• 網路模式: bridge (隔離網路)"
        print_info "• Web 介面: http://localhost:80"
        
        # 快速服務檢查
        echo ""
        echo -e "${YELLOW}🔧 快速服務檢查${NC}"
        check_agvc_server_service
        check_postgres_service
        check_zenoh_service
    fi
    
    echo ""
    echo -e "${YELLOW}🔧 常用操作${NC}"
    echo -e "  進入容器: ${GREEN}$(basename $0) exec${NC}"
    echo -e "  查看日誌: ${GREEN}$(basename $0) logs agvc_server -f${NC}"
    echo -e "  健康檢查: ${GREEN}$(basename $0) health${NC}"
    echo -e "  服務狀態: ${GREEN}$(basename $0) services${NC}"
}

# ============================================================================
# 主程式邏輯
# ============================================================================

main() {
    # 檢查基本依賴
    if ! check_docker_running || ! check_compose_file; then
        exit 1
    fi
    
    local action="${1:-overview}"
    
    case "$action" in
        "start")
            agvc_start
            ;;
        "stop")
            agvc_stop
            ;;
        "restart")
            agvc_restart
            ;;
        "status")
            agvc_status
            ;;
        "logs")
            shift
            agvc_logs "$@"
            ;;
        "exec")
            agvc_exec
            ;;
        "shell")
            agvc_shell
            ;;
        "health")
            agvc_health
            ;;
        "services")
            agvc_services
            ;;
        "overview"|"")
            agvc_overview
            ;;
        "help"|"-h"|"--help")
            show_help
            ;;
        *)
            print_error "未知動作: $action"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# 如果腳本被直接執行（而非被 source）
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi