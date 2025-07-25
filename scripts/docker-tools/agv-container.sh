#!/bin/bash
# RosAGV AGV 容器專用管理工具
# 版本: 1.0
# 說明: AGV 容器的啟動、停止、進入和狀態檢查工具

# ============================================================================
# 初始化和設定
# ============================================================================

# 獲取腳本目錄和專案根目錄
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Docker Compose 檔案路徑
COMPOSE_FILE="$PROJECT_ROOT/docker-compose.yml"
CONTAINER_NAME="rosagv"
SERVICE_NAME="rosagv"

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
    echo -e "${CYAN}🚗 RosAGV AGV 容器管理工具${NC}"
    echo -e "${CYAN}============================${NC}"
    echo ""
}

show_help() {
    show_header
    echo -e "${YELLOW}用法:${NC}"
    echo "  $(basename $0) [動作]"
    echo ""
    echo -e "${YELLOW}可用動作:${NC}"
    echo -e "  ${GREEN}start${NC}     - 啟動 AGV 容器"
    echo -e "  ${GREEN}stop${NC}      - 停止 AGV 容器"
    echo -e "  ${GREEN}restart${NC}   - 重啟 AGV 容器"
    echo -e "  ${GREEN}status${NC}    - 檢查 AGV 容器狀態"
    echo -e "  ${GREEN}logs${NC}      - 顯示 AGV 容器日誌"
    echo -e "  ${GREEN}exec${NC}      - 進入 AGV 容器 (自動載入 agv_source)"
    echo -e "  ${GREEN}shell${NC}     - 進入 AGV 容器 shell"
    echo -e "  ${GREEN}health${NC}    - AGV 系統健康檢查"
    echo -e "  ${GREEN}overview${NC}  - 顯示 AGV 容器概況 (預設)"
    echo ""
    echo -e "${YELLOW}範例:${NC}"
    echo "  $(basename $0) start"
    echo "  $(basename $0) exec"
    echo "  $(basename $0) logs -f"
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

get_container_status() {
    docker compose -f "$COMPOSE_FILE" ps --format "table" --filter "service=$SERVICE_NAME" 2>/dev/null | tail -n +2
}

is_container_running() {
    local status=$(docker compose -f "$COMPOSE_FILE" ps -q "$SERVICE_NAME" 2>/dev/null)
    [ -n "$status" ] && docker inspect "$CONTAINER_NAME" --format '{{.State.Running}}' 2>/dev/null | grep -q "true"
}

# ============================================================================
# 主要功能函數
# ============================================================================

agv_start() {
    print_info "啟動 AGV 容器..."
    
    if is_container_running; then
        print_warning "AGV 容器已經在運行中"
        return 0
    fi
    
    docker compose -f "$COMPOSE_FILE" up -d "$SERVICE_NAME"
    if [ $? -eq 0 ]; then
        sleep 2
        if is_container_running; then
            print_success "AGV 容器啟動成功"
        else
            print_error "AGV 容器啟動失敗"
            return 1
        fi
    else
        print_error "AGV 容器啟動失敗"
        return 1
    fi
}

agv_stop() {
    print_info "停止 AGV 容器..."
    
    if ! is_container_running; then
        print_warning "AGV 容器未在運行"
        return 0
    fi
    
    docker compose -f "$COMPOSE_FILE" stop "$SERVICE_NAME"
    if [ $? -eq 0 ]; then
        print_success "AGV 容器停止成功"
    else
        print_error "AGV 容器停止失敗"
        return 1
    fi
}

agv_restart() {
    print_info "重啟 AGV 容器..."
    agv_stop
    sleep 2
    agv_start
}

agv_status() {
    echo -e "${CYAN}📊 AGV 容器狀態${NC}"
    echo -e "${CYAN}===============${NC}"
    
    local status_output=$(get_container_status)
    if [ -n "$status_output" ]; then
        echo "$status_output"
    else
        print_warning "AGV 容器未啟動"
    fi
    
    echo ""
    
    # 檢查容器詳細狀態
    if is_container_running; then
        print_success "容器運行狀態: 正常運行"
        
        # 檢查網路模式
        local network_mode=$(docker inspect "$CONTAINER_NAME" --format '{{.HostConfig.NetworkMode}}' 2>/dev/null)
        print_info "網路模式: $network_mode"
        
        # 檢查設備掛載
        local devices=$(docker inspect "$CONTAINER_NAME" --format '{{range .HostConfig.Devices}}{{.PathOnHost}}:{{.PathInContainer}} {{end}}' 2>/dev/null)
        if [ -n "$devices" ]; then
            print_info "設備掛載: $devices"
        fi
        
        # 檢查環境變數
        local container_type=$(docker exec "$CONTAINER_NAME" printenv CONTAINER_TYPE 2>/dev/null)
        if [ "$container_type" = "agv" ]; then
            print_success "容器類型: $container_type"
        else
            print_warning "容器類型異常: $container_type"
        fi
    else
        print_error "容器運行狀態: 未運行"
    fi
}

agv_logs() {
    local log_args="$@"
    
    print_info "顯示 AGV 容器日誌..."
    
    if ! is_container_running; then
        print_warning "AGV 容器未在運行，顯示最近的日誌"
    fi
    
    if [ -n "$log_args" ]; then
        docker compose -f "$COMPOSE_FILE" logs $log_args "$SERVICE_NAME"
    else
        docker compose -f "$COMPOSE_FILE" logs --tail=50 "$SERVICE_NAME"
    fi
}

agv_exec() {
    if ! is_container_running; then
        print_error "AGV 容器未在運行，無法進入"
        print_info "請先使用 '$(basename $0) start' 啟動容器"
        return 1
    fi
    
    print_info "進入 AGV 容器並自動載入 agv_source..."
    
    # 進入容器並自動執行 agv_source
    docker exec -it "$CONTAINER_NAME" bash -c "
        echo -e '\033[0;32m🚗 進入 AGV 容器環境\033[0m'
        echo -e '\033[0;36m載入 AGV 開發環境...\033[0m'
        source /app/setup.bash && agv_source
        echo -e '\033[0;32m✅ AGV 環境載入完成\033[0m'
        exec bash
    "
}

agv_shell() {
    if ! is_container_running; then
        print_error "AGV 容器未在運行，無法進入"
        print_info "請先使用 '$(basename $0) start' 啟動容器"
        return 1
    fi
    
    print_info "進入 AGV 容器 shell..."
    docker exec -it "$CONTAINER_NAME" bash
}

agv_health() {
    echo -e "${CYAN}🏥 AGV 系統健康檢查${NC}"
    echo -e "${CYAN}===================${NC}"
    
    local health_score=0
    local total_checks=5
    
    # 1. 檢查容器運行狀態
    echo -e "\n${YELLOW}1. 容器運行狀態${NC}"
    if is_container_running; then
        print_success "容器正常運行"
        ((health_score++))
    else
        print_error "容器未運行"
    fi
    
    if is_container_running; then
        # 2. 檢查環境變數
        echo -e "\n${YELLOW}2. 環境變數檢查${NC}"
        local container_type=$(docker exec "$CONTAINER_NAME" printenv CONTAINER_TYPE 2>/dev/null)
        local rmw_impl=$(docker exec "$CONTAINER_NAME" printenv RMW_IMPLEMENTATION 2>/dev/null)
        
        if [ "$container_type" = "agv" ]; then
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
        
        # 3. 檢查設備掛載
        echo -e "\n${YELLOW}3. 設備掛載檢查${NC}"
        if docker exec "$CONTAINER_NAME" test -d /dev/input 2>/dev/null; then
            local input_devices=$(docker exec "$CONTAINER_NAME" ls /dev/input 2>/dev/null | wc -l)
            if [ "$input_devices" -gt 0 ]; then
                print_success "輸入設備掛載正常 ($input_devices 個設備)"
                ((health_score++))
            else
                print_warning "輸入設備目錄存在但無設備"
            fi
        else
            print_error "輸入設備掛載失敗"
        fi
        
        # 4. 檢查 setup.bash
        echo -e "\n${YELLOW}4. 開發環境檢查${NC}"
        if docker exec "$CONTAINER_NAME" test -f /app/setup.bash 2>/dev/null; then
            print_success "setup.bash 存在"
            ((health_score++))
        else
            print_error "setup.bash 不存在"
        fi
        
        # 5. 檢查網路模式
        echo -e "\n${YELLOW}5. 網路配置檢查${NC}"
        local network_mode=$(docker inspect "$CONTAINER_NAME" --format '{{.HostConfig.NetworkMode}}' 2>/dev/null)
        if [ "$network_mode" = "host" ]; then
            print_success "網路模式: host (正確)"
        else
            print_error "網路模式不正確: $network_mode"
        fi
    fi
    
    # 健康度評分
    echo -e "\n${CYAN}📊 健康度評分${NC}"
    local percentage=$((health_score * 100 / total_checks))
    
    if [ $percentage -ge 80 ]; then
        print_success "健康度: $health_score/$total_checks ($percentage%) - 系統健康"
    elif [ $percentage -ge 60 ]; then
        print_warning "健康度: $health_score/$total_checks ($percentage%) - 系統部分異常"
    else
        print_error "健康度: $health_score/$total_checks ($percentage%) - 系統異常"
    fi
}

agv_overview() {
    show_header
    
    echo -e "${YELLOW}📋 AGV 容器概況${NC}"
    echo ""
    
    # 基本狀態
    if is_container_running; then
        print_success "狀態: 運行中"
        
        # 容器資訊
        local created=$(docker inspect "$CONTAINER_NAME" --format '{{.Created}}' 2>/dev/null | cut -d'T' -f1)
        local image=$(docker inspect "$CONTAINER_NAME" --format '{{.Config.Image}}' 2>/dev/null)
        
        print_info "映像檔: $image"
        print_info "建立時間: $created"
        print_info "網路模式: host"
        print_info "設備掛載: /dev/input"
        
    else
        print_warning "狀態: 未運行"
        print_info "使用 '$(basename $0) start' 啟動容器"
    fi
    
    echo ""
    echo -e "${YELLOW}🔧 常用操作${NC}"
    echo -e "  進入容器: ${GREEN}$(basename $0) exec${NC}"
    echo -e "  查看日誌: ${GREEN}$(basename $0) logs -f${NC}"
    echo -e "  健康檢查: ${GREEN}$(basename $0) health${NC}"
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
            agv_start
            ;;
        "stop")
            agv_stop
            ;;
        "restart")
            agv_restart
            ;;
        "status")
            agv_status
            ;;
        "logs")
            shift
            agv_logs "$@"
            ;;
        "exec")
            agv_exec
            ;;
        "shell")
            agv_shell
            ;;
        "health")
            agv_health
            ;;
        "overview"|"")
            agv_overview
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