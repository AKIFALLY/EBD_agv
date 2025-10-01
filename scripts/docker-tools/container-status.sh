#!/bin/bash
# RosAGV 容器狀態檢查工具
# 版本: 1.0
# 說明: 檢查所有容器運行狀態、端口佔用和資源使用情況

# ============================================================================
# 初始化和設定
# ============================================================================

# 獲取腳本目錄和專案根目錄
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Docker Compose 檔案路徑
AGV_COMPOSE_FILE="$PROJECT_ROOT/docker-compose.yml"
AGVC_COMPOSE_FILE="$PROJECT_ROOT/docker-compose.agvc.yml"

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
    echo -e "${CYAN}📊 RosAGV 容器狀態檢查工具${NC}"
    echo -e "${CYAN}==============================${NC}"
    echo ""
}

show_help() {
    show_header
    echo -e "${YELLOW}用法:${NC}"
    echo "  $(basename $0) [動作]"
    echo ""
    echo -e "${YELLOW}可用動作:${NC}"
    echo -e "  ${GREEN}all${NC}       - 檢查所有容器狀態 (預設)"
    echo -e "  ${GREEN}agv${NC}       - 僅檢查 AGV 容器狀態"
    echo -e "  ${GREEN}agvc${NC}      - 僅檢查 AGVC 容器狀態"
    echo -e "  ${GREEN}ports${NC}     - 檢查端口佔用情況"
    echo -e "  ${GREEN}resources${NC} - 檢查容器資源使用"
    echo -e "  ${GREEN}network${NC}   - 檢查網路配置"
    echo -e "  ${GREEN}health${NC}    - 健康狀態判斷"
    echo -e "  ${GREEN}summary${NC}   - 簡要狀態摘要"
    echo ""
    echo -e "${YELLOW}範例:${NC}"
    echo "  $(basename $0)           # 檢查所有容器"
    echo "  $(basename $0) agv       # 僅檢查 AGV"
    echo "  $(basename $0) ports     # 檢查端口"
    echo "  $(basename $0) health    # 健康檢查"
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
# 基礎檢查函數
# ============================================================================

check_docker_running() {
    if ! docker info >/dev/null 2>&1; then
        print_error "Docker 服務未運行"
        return 1
    fi
    return 0
}

check_compose_files() {
    local files_exist=0
    
    if [ -f "$AGV_COMPOSE_FILE" ]; then
        ((files_exist++))
    else
        print_warning "AGV Compose 檔案不存在: $AGV_COMPOSE_FILE"
    fi
    
    if [ -f "$AGVC_COMPOSE_FILE" ]; then
        ((files_exist++))
    else
        print_warning "AGVC Compose 檔案不存在: $AGVC_COMPOSE_FILE"
    fi
    
    return $((2 - files_exist))
}

# ============================================================================
# 容器狀態檢查函數
# ============================================================================

get_agv_containers() {
    if [ -f "$AGV_COMPOSE_FILE" ]; then
        # 只顯示實際運行的容器
        local output=$(docker compose -f "$AGV_COMPOSE_FILE" ps --format "table" 2>/dev/null)
        # 檢查是否有實際的容器數據（不只是標題行）
        local line_count=$(echo "$output" | wc -l)
        if [ $line_count -gt 1 ]; then
            echo "$output"
        fi
    fi
}

get_agvc_containers() {
    if [ -f "$AGVC_COMPOSE_FILE" ]; then
        # 只顯示實際運行的容器
        local output=$(docker compose -f "$AGVC_COMPOSE_FILE" ps --format "table" 2>/dev/null)
        # 檢查是否有實際的容器數據（不只是標題行）
        local line_count=$(echo "$output" | wc -l)
        if [ $line_count -gt 1 ]; then
            echo "$output"
        fi
    fi
}

is_container_running() {
    local container_name="$1"
    docker inspect "$container_name" --format '{{.State.Running}}' 2>/dev/null | grep -q "true"
}

# 檢查容器是否存在（不管是否運行）
container_exists() {
    local container_name="$1"
    docker inspect "$container_name" >/dev/null 2>&1
}

get_container_uptime() {
    local container_name="$1"
    docker inspect "$container_name" --format '{{.State.StartedAt}}' 2>/dev/null | xargs -I {} date -d {} "+%Y-%m-%d %H:%M:%S" 2>/dev/null
}

get_container_ip() {
    local container_name="$1"
    docker inspect "$container_name" --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' 2>/dev/null
}

# ============================================================================
# 端口檢查函數
# ============================================================================

check_port_status() {
    local port="$1"
    local description="$2"
    
    if timeout 2 bash -c "echo > /dev/tcp/localhost/$port" 2>/dev/null; then
        print_success "Port $port ($description): 可連接"
        return 0
    else
        print_error "Port $port ($description): 無法連接"
        return 1
    fi
}

check_agvc_ports() {
    echo -e "${YELLOW}🌐 AGVC 端口檢查${NC}"
    echo -e "${YELLOW}===============${NC}"
    
    local healthy_ports=0
    local total_ports=6
    
    # HTTP 端口
    if check_port_status "80" "Nginx Web"; then ((healthy_ports++)); fi
    
    # API 端口 (需要檢查 bridge 網路)
    if timeout 2 bash -c "echo > /dev/tcp/192.168.100.100/7447" 2>/dev/null; then
        print_success "Port 7447 (Zenoh Router): 可連接"
        ((healthy_ports++))
    else
        print_error "Port 7447 (Zenoh Router): 無法連接"
    fi
    
    if timeout 2 bash -c "echo > /dev/tcp/192.168.100.100/8000" 2>/dev/null; then
        print_success "Port 8000 (主 API): 可連接"
        ((healthy_ports++))
    else
        print_error "Port 8000 (主 API): 無法連接"
    fi
    
    if timeout 2 bash -c "echo > /dev/tcp/192.168.100.100/8001" 2>/dev/null; then
        print_success "Port 8001 (AGVCUI): 可連接"
        ((healthy_ports++))
    else
        print_warning "Port 8001 (AGVCUI): 無法連接"
    fi
    
    if timeout 2 bash -c "echo > /dev/tcp/192.168.100.100/8002" 2>/dev/null; then
        print_success "Port 8002 (OPUI): 可連接"
        ((healthy_ports++))
    else
        print_warning "Port 8002 (OPUI): 無法連接"
    fi
    
    if timeout 2 bash -c "echo > /dev/tcp/192.168.100.100/5432" 2>/dev/null; then
        print_success "Port 5432 (PostgreSQL): 可連接"
        ((healthy_ports++))
    else
        print_error "Port 5432 (PostgreSQL): 無法連接"
    fi
    
    echo ""
    print_info "端口健康度: $healthy_ports/$total_ports"
    return $((total_ports - healthy_ports))
}

check_agv_ports() {
    echo -e "${YELLOW}🚗 AGV 端口檢查${NC}"
    echo -e "${YELLOW}===============${NC}"
    
    # AGV 使用 host 網路，主要檢查 Zenoh 連接能力
    if is_container_running "rosagv"; then
        print_success "AGV 容器: 使用 host 網路模式"
        
        # 檢查是否能連接到 AGVC 的 Zenoh Router
        if timeout 2 bash -c "echo > /dev/tcp/192.168.100.100/7447" 2>/dev/null; then
            print_success "與 AGVC Zenoh Router 連接: 正常"
        else
            print_warning "與 AGVC Zenoh Router 連接: 異常"
        fi
    else
        print_error "AGV 容器未運行"
    fi
}

# ============================================================================
# 資源使用檢查函數
# ============================================================================

check_container_resources() {
    echo -e "${YELLOW}💻 容器資源使用${NC}"
    echo -e "${YELLOW}===============${NC}"
    
    # 檢查是否有運行中的容器
    local running_containers=$(docker ps --format "table {{.Names}}\t{{.CPU}}\t{{.MemUsage}}\t{{.Status}}" --filter "name=rosagv" --filter "name=agvc_server" --filter "name=nginx" --filter "name=postgres" 2>/dev/null)
    
    if [ -n "$running_containers" ]; then
        echo "$running_containers"
        echo ""
        
        # 總體資源使用
        local total_containers=$(docker ps -q --filter "name=rosagv" --filter "name=agvc_server" --filter "name=nginx" --filter "name=postgres" | wc -l)
        print_info "運行中的 RosAGV 相關容器: $total_containers 個"
        
        # 檢查系統資源
        local cpu_usage=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)
        local mem_usage=$(free | grep Mem | awk '{printf "%.1f", $3/$2 * 100.0}')
        
        print_info "系統 CPU 使用率: ${cpu_usage}%"
        print_info "系統記憶體使用率: ${mem_usage}%"
    else
        print_warning "沒有運行中的 RosAGV 相關容器"
    fi
}

# ============================================================================
# 網路配置檢查函數
# ============================================================================

check_network_config() {
    echo -e "${YELLOW}🌐 網路配置檢查${NC}"
    echo -e "${YELLOW}===============${NC}"
    
    # 檢查 Docker 網路
    local agvc_network=$(docker network ls | grep "agvc.*bridge")
    if [ -n "$agvc_network" ]; then
        print_success "AGVC Bridge 網路: 已建立"
        
        # 檢查網路詳細資訊
        local network_name=$(echo "$agvc_network" | awk '{print $2}')
        local network_info=$(docker network inspect "$network_name" --format '{{range .IPAM.Config}}{{.Subnet}}{{end}}' 2>/dev/null)
        if [ -n "$network_info" ]; then
            print_info "網路子網: $network_info"
        fi
    else
        print_warning "AGVC Bridge 網路: 未建立"
    fi
    
    # 檢查容器 IP 配置
    echo ""
    print_info "容器 IP 配置:"
    
    if is_container_running "agvc_server"; then
        local agvc_ip=$(get_container_ip "agvc_server")
        if [ "$agvc_ip" = "192.168.100.100" ]; then
            print_success "• agvc_server: $agvc_ip (正確)"
        else
            print_warning "• agvc_server: $agvc_ip (異常，應為 192.168.100.100)"
        fi
    else
        print_warning "• agvc_server: 容器未運行"
    fi
    
    if is_container_running "nginx"; then
        local nginx_ip=$(get_container_ip "nginx")
        print_info "• nginx: $nginx_ip"
    else
        print_warning "• nginx: 容器未運行"
    fi
    
    if is_container_running "postgres"; then
        local postgres_ip=$(get_container_ip "postgres")
        print_info "• postgres: $postgres_ip"
    else
        print_warning "• postgres: 容器未運行"
    fi
    
    if is_container_running "rosagv"; then
        print_info "• rosagv: host 網路模式 (與宿主機共享)"
    else
        print_warning "• rosagv: 容器未運行"
    fi
}

# ============================================================================
# 健康狀態判斷函數
# ============================================================================

check_intelligent_health() {
    echo -e "${CYAN}🏥 健康狀態判斷${NC}"
    echo -e "${CYAN}==================${NC}"
    
    local total_score=0
    local max_score=0
    
    # AGV 系統健康檢查
    echo -e "\n${YELLOW}🚗 AGV 系統${NC}"
    if [ -f "$AGV_COMPOSE_FILE" ]; then
        ((max_score += 3))
        if is_container_running "rosagv"; then
            print_success "AGV 容器: 運行中"
            ((total_score += 2))
            
            # 檢查環境配置
            local container_type=$(docker exec "rosagv" printenv CONTAINER_TYPE 2>/dev/null)
            if [ "$container_type" = "agv" ]; then
                print_success "AGV 環境配置: 正確"
                ((total_score += 1))
            else
                print_warning "AGV 環境配置: 異常"
            fi
        else
            print_error "AGV 容器: 未運行"
        fi
    else
        print_warning "AGV 系統: 配置檔案不存在"
    fi
    
    # AGVC 系統健康檢查
    echo -e "\n${YELLOW}🖥️ AGVC 系統${NC}"
    if [ -f "$AGVC_COMPOSE_FILE" ]; then
        ((max_score += 9))  # 3個容器 + 3個核心服務 + 3個端口
        
        # 容器狀態
        local agvc_running=0
        if is_container_running "agvc_server"; then
            print_success "AGVC 容器: 運行中"
            ((total_score += 1))
            ((agvc_running++))
        else
            print_error "AGVC 容器: 未運行"
        fi
        
        if is_container_running "postgres"; then
            print_success "PostgreSQL 容器: 運行中"
            ((total_score += 1))
            ((agvc_running++))
        else
            print_error "PostgreSQL 容器: 未運行"
        fi
        
        if is_container_running "nginx"; then
            print_success "Nginx 容器: 運行中"
            ((total_score += 1))
            ((agvc_running++))
        else
            print_error "Nginx 容器: 未運行"
        fi
        
        # 服務檢查 (只有在容器運行時才檢查)
        if [ $agvc_running -gt 0 ]; then
            # PostgreSQL 服務
            if docker exec postgres pg_isready -U agvc >/dev/null 2>&1; then
                print_success "PostgreSQL 服務: 正常"
                ((total_score += 1))
            else
                print_warning "PostgreSQL 服務: 異常"
            fi
            
            # Zenoh Router
            if timeout 2 bash -c "echo > /dev/tcp/192.168.100.100/7447" 2>/dev/null; then
                print_success "Zenoh Router: 正常"
                ((total_score += 1))
            else
                print_warning "Zenoh Router: 異常"
            fi
            
            # Web 服務
            if timeout 2 bash -c "echo > /dev/tcp/localhost/80" 2>/dev/null; then
                print_success "Web 服務: 正常"
                ((total_score += 1))
            else
                print_warning "Web 服務: 異常"
            fi
            
            # API 端點
            local api_healthy=0
            if timeout 2 bash -c "echo > /dev/tcp/192.168.100.100/8000" 2>/dev/null; then ((api_healthy++)); fi
            if timeout 2 bash -c "echo > /dev/tcp/192.168.100.100/8001" 2>/dev/null; then ((api_healthy++)); fi
            if timeout 2 bash -c "echo > /dev/tcp/192.168.100.100/8002" 2>/dev/null; then ((api_healthy++)); fi
            
            if [ $api_healthy -ge 2 ]; then
                print_success "API 端點: 正常 ($api_healthy/3)"
                ((total_score += 2))
            elif [ $api_healthy -eq 1 ]; then
                print_warning "API 端點: 部分正常 ($api_healthy/3)"
                ((total_score += 1))
            else
                print_error "API 端點: 異常 ($api_healthy/3)"
            fi
        fi
    else
        print_warning "AGVC 系統: 配置檔案不存在"
    fi
    
    # 系統間通訊檢查
    echo -e "\n${YELLOW}🔗 系統間通訊${NC}"
    ((max_score += 2))
    if is_container_running "rosagv" && is_container_running "agvc_server"; then
        # AGV 到 AGVC 通訊
        if timeout 2 bash -c "echo > /dev/tcp/192.168.100.100/7447" 2>/dev/null; then
            print_success "AGV ↔ AGVC 通訊: 正常"
            ((total_score += 2))
        else
            print_warning "AGV ↔ AGVC 通訊: 異常"
        fi
    else
        print_warning "AGV ↔ AGVC 通訊: 容器未完全運行"
    fi
    
    # 計算健康度百分比
    local percentage=0
    if [ $max_score -gt 0 ]; then
        percentage=$((total_score * 100 / max_score))
    fi
    
    echo -e "\n${CYAN}📊 系統健康度評分${NC}"
    echo -e "${CYAN}==================${NC}"
    
    if [ $percentage -ge 90 ]; then
        print_success "整體健康度: $total_score/$max_score ($percentage%) - 系統優秀"
    elif [ $percentage -ge 75 ]; then
        print_success "整體健康度: $total_score/$max_score ($percentage%) - 系統良好"
    elif [ $percentage -ge 50 ]; then
        print_warning "整體健康度: $total_score/$max_score ($percentage%) - 系統需要關注"
    else
        print_error "整體健康度: $total_score/$max_score ($percentage%) - 系統異常"
    fi
    
    # 提供建議
    echo -e "\n${YELLOW}💡 建議操作${NC}"
    if [ $percentage -lt 75 ]; then
        if ! is_container_running "rosagv" && [ -f "$AGV_COMPOSE_FILE" ]; then
            echo -e "  • 啟動 AGV 系統: ${GREEN}scripts/docker-tools/agv-container.sh start${NC}"
        fi
        if ! is_container_running "agvc_server" && [ -f "$AGVC_COMPOSE_FILE" ]; then
            echo -e "  • 啟動 AGVC 系統: ${GREEN}scripts/docker-tools/agvc-container.sh start${NC}"
        fi
        if [ $percentage -lt 50 ]; then
            echo -e "  • 檢查 Docker 服務和配置檔案"
            echo -e "  • 查看容器日誌以了解具體問題"
        fi
    else
        echo -e "  • 系統狀態良好，繼續正常運行"
    fi
}

# ============================================================================
# 狀態摘要函數
# ============================================================================

show_status_summary() {
    echo -e "${CYAN}📋 RosAGV 系統狀態摘要${NC}"
    echo -e "${CYAN}=======================${NC}"
    
    # AGV 狀態
    echo -e "\n${YELLOW}🚗 AGV 系統${NC}"
    if [ -f "$AGV_COMPOSE_FILE" ]; then
        if is_container_running "rosagv"; then
            local uptime=$(get_container_uptime "rosagv")
            print_success "狀態: 運行中 (啟動時間: $uptime)"
        else
            print_warning "狀態: 未運行"
        fi
    else
        print_error "配置檔案不存在"
    fi
    
    # AGVC 狀態
    echo -e "\n${YELLOW}🖥️ AGVC 系統${NC}"
    if [ -f "$AGVC_COMPOSE_FILE" ]; then
        local running_services=0
        local total_services=3
        
        if is_container_running "agvc_server"; then ((running_services++)); fi
        if is_container_running "postgres"; then ((running_services++)); fi
        if is_container_running "nginx"; then ((running_services++)); fi
        
        if [ $running_services -eq $total_services ]; then
            print_success "狀態: 完全運行 ($running_services/$total_services 服務)"
        elif [ $running_services -gt 0 ]; then
            print_warning "狀態: 部分運行 ($running_services/$total_services 服務)"
        else
            print_warning "狀態: 未運行"
        fi
    else
        print_error "配置檔案不存在"
    fi
    
    # 快速網路檢查
    echo -e "\n${YELLOW}🌐 網路連接${NC}"
    local network_health=0
    local total_checks=3
    
    if timeout 1 bash -c "echo > /dev/tcp/localhost/80" 2>/dev/null; then
        ((network_health++))
    fi
    if timeout 1 bash -c "echo > /dev/tcp/192.168.100.100/7447" 2>/dev/null; then
        ((network_health++))
    fi
    if timeout 1 bash -c "echo > /dev/tcp/192.168.100.100/8000" 2>/dev/null; then
        ((network_health++))
    fi
    
    if [ $network_health -eq $total_checks ]; then
        print_success "網路狀態: 正常 ($network_health/$total_checks 端點)"
    elif [ $network_health -gt 0 ]; then
        print_warning "網路狀態: 部分正常 ($network_health/$total_checks 端點)"
    else
        print_error "網路狀態: 異常 ($network_health/$total_checks 端點)"
    fi
}

# ============================================================================
# 主要檢查函數
# ============================================================================

check_all_containers() {
    show_header

    echo -e "${YELLOW}📊 所有容器狀態${NC}"
    echo -e "${YELLOW}===============${NC}"

    # 檢查 AGV 容器（只有在實際存在時才顯示）
    if [ -f "$AGV_COMPOSE_FILE" ] && container_exists "rosagv"; then
        echo -e "\n${CYAN}🚗 AGV 容器${NC}"
        local agv_status=$(get_agv_containers)
        if [ -n "$agv_status" ]; then
            echo "$agv_status"
        else
            print_warning "AGV 容器未啟動"
        fi
    fi

    # 檢查 AGVC 容器（只有在實際存在時才顯示）
    if [ -f "$AGVC_COMPOSE_FILE" ] && (container_exists "agvc_server" || container_exists "postgres" || container_exists "nginx"); then
        echo -e "\n${CYAN}🖥️ AGVC 容器${NC}"
        local agvc_status=$(get_agvc_containers)
        if [ -n "$agvc_status" ]; then
            echo "$agvc_status"
        else
            print_warning "AGVC 容器未啟動"
        fi
    fi
    
    # 快速健康檢查
    echo -e "\n${CYAN}🏥 快速健康檢查${NC}"
    local healthy_containers=0
    local total_containers=0

    # 只檢查實際存在的 AGV 容器
    if [ -f "$AGV_COMPOSE_FILE" ] && container_exists "rosagv"; then
        ((total_containers++))
        if is_container_running "rosagv"; then
            ((healthy_containers++))
            print_success "rosagv: 運行正常"
        else
            print_warning "rosagv: 未運行"
        fi
    fi

    # 只檢查實際存在的 AGVC 容器
    if [ -f "$AGVC_COMPOSE_FILE" ]; then
        if container_exists "agvc_server"; then
            ((total_containers++))
            if is_container_running "agvc_server"; then
                ((healthy_containers++))
                print_success "agvc_server: 運行正常"
            else
                print_warning "agvc_server: 未運行"
            fi
        fi

        if container_exists "postgres"; then
            ((total_containers++))
            if is_container_running "postgres"; then
                ((healthy_containers++))
                print_success "postgres: 運行正常"
            else
                print_warning "postgres: 未運行"
            fi
        fi

        if container_exists "nginx"; then
            ((total_containers++))
            if is_container_running "nginx"; then
                ((healthy_containers++))
                print_success "nginx: 運行正常"
            else
                print_warning "nginx: 未運行"
            fi
        fi
    fi

    echo ""
    print_info "容器健康度: $healthy_containers/$total_containers"
}

check_agv_only() {
    echo -e "${CYAN}🚗 AGV 容器狀態${NC}"
    echo -e "${CYAN}===============${NC}"
    
    if [ ! -f "$AGV_COMPOSE_FILE" ]; then
        print_error "AGV Compose 檔案不存在: $AGV_COMPOSE_FILE"
        return 1
    fi
    
    local agv_status=$(get_agv_containers)
    if [ -n "$agv_status" ]; then
        echo "$agv_status"
        echo ""
        
        if is_container_running "rosagv"; then
            local uptime=$(get_container_uptime "rosagv")
            print_success "容器啟動時間: $uptime"
            
            local container_type=$(docker exec "rosagv" printenv CONTAINER_TYPE 2>/dev/null)
            print_info "容器類型: $container_type"
            print_info "網路模式: host"
        fi
    else
        print_warning "AGV 容器未啟動"
    fi
}

check_agvc_only() {
    echo -e "${CYAN}🖥️ AGVC 容器狀態${NC}"
    echo -e "${CYAN}=================${NC}"
    
    if [ ! -f "$AGVC_COMPOSE_FILE" ]; then
        print_error "AGVC Compose 檔案不存在: $AGVC_COMPOSE_FILE"
        return 1
    fi
    
    local agvc_status=$(get_agvc_containers)
    if [ -n "$agvc_status" ]; then
        echo "$agvc_status"
        echo ""
        
        # 詳細服務狀態
        echo -e "${YELLOW}服務詳細狀態:${NC}"
        if is_container_running "agvc_server"; then
            local uptime=$(get_container_uptime "agvc_server")
            local ip=$(get_container_ip "agvc_server")
            print_success "agvc_server: 運行中 (IP: $ip, 啟動: $uptime)"
        else
            print_warning "agvc_server: 未運行"
        fi
        
        if is_container_running "postgres"; then
            local ip=$(get_container_ip "postgres")
            print_success "postgres: 運行中 (IP: $ip)"
        else
            print_warning "postgres: 未運行"
        fi
        
        if is_container_running "nginx"; then
            local ip=$(get_container_ip "nginx")
            print_success "nginx: 運行中 (IP: $ip)"
        else
            print_warning "nginx: 未運行"
        fi
    else
        print_warning "AGVC 容器未啟動"
    fi
}

# ============================================================================
# 主程式邏輯
# ============================================================================

main() {
    # 檢查基本依賴
    if ! check_docker_running; then
        exit 1
    fi
    
    local action="${1:-all}"
    
    case "$action" in
        "all"|"")
            check_all_containers
            ;;
        "agv")
            check_agv_only
            ;;
        "agvc")
            check_agvc_only
            ;;
        "ports")
            echo -e "${CYAN}🌐 端口佔用檢查${NC}"
            echo -e "${CYAN}===============${NC}"
            check_agvc_ports
            echo ""
            check_agv_ports
            ;;
        "resources")
            check_container_resources
            ;;
        "network")
            check_network_config
            ;;
        "health")
            check_intelligent_health
            ;;
        "summary")
            show_status_summary
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
